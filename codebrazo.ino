#include <Arduino.h>
#include <Wire.h>
#include <Bluepad32.h>
#include <Adafruit_PWMServoDriver.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct ConfigHardware {
  uint8_t pinSDA;
  uint8_t pinSCL;
  uint32_t frecI2C;// Velocidad de comunicación (400000 = 400 kHz, para respuesta rápida del mando)
  uint8_t dirPCA9685;// Dirección identificadora de la placa PCA en el bus I2C (por defecto 0x40)
  uint16_t frecPWM;// Frecuencia eléctrica de los servomotores (50 Hz es el estándar obligatorio)
  uint32_t osciladorPCA;// Velocidad del reloj interno de la PCA (25 MHz, necesario para que calcule bien los ángulos)
  uint16_t reintentoPCAms;// Tiempo (en milisegundos) que espera antes de intentar reconectar si la PCA falla o se desconecta
};

static const ConfigHardware HW = { 9, 8, 400000, 0x40, 50, 25000000, 2000 };

struct ConfigControl {
  uint16_t periodoControlMs;
  float dtMaximo;
  uint16_t periodoRefrescoMs;
  uint16_t retardoArranqueMs;
  float velHome;               // <-- VELOCIDAD DE REGRESO A HOME (reducida a 25)
  float toleranciaHome;
  float umbralCancelaHome;
  uint16_t antirreboteMs;
  bool homeAlDesconectar;
};

static const ConfigControl CTRL = { 10, 0.05f, 250, 1200, 25.0f, 0.6f, 0.25f, 180, false };

struct ConfigCurva {
  float zonaMuertaStick;
  float zonaMuertaGatillo;
  float rangoStick;
  float rangoGatillo;
  float exponente;
  float fraccionVelMin;
};

static const ConfigCurva CURVA = { 45.0f, 50.0f, 512.0f, 1023.0f, 1.35f, 0.02f };

struct ConfigBotones {
  bool cerrarConR1, cerrarConR2, abrirConL1, abrirConL2, rollConDPad, rollIzqConX, rollDerConB;
};

static const ConfigBotones BTN = { true, false, true, false, true, true, true };

enum FuenteEntrada {
  FUENTE_STICK_IZQ_X = 0, FUENTE_STICK_IZQ_Y, FUENTE_STICK_DER_X, FUENTE_STICK_DER_Y, FUENTE_DPAD_H, FUENTE_GATILLOS_PINZA
};

enum IndiceEje { EJE_BASE = 0, EJE_HOMBRO, EJE_CODO, EJE_MUNECA_PITCH, EJE_MUNECA_ROLL, EJE_PINZA, NUM_EJES };

struct ConfigServo {
  const char* nombre;
  uint8_t canal;
  int8_t canalEspejo;
  uint8_t angMin, angMax, angHome;
  bool invertir;
  float velMax; // <-- VELOCIDAD MÁXIMA DEL SERVO (grados/seg)
  float acel;   // <-- ACELERACIÓN DEL SERVO (suavidad de arranque)
  uint16_t pulso0us, pulso180us;
  FuenteEntrada fuente;
};

// --- AJUSTE DE VELOCIDADES ---
// velMax reducida (ej. Base de 220 a 100)
// acel reducida (ej. Base de 1400 a 400 para arranques más suaves)
//  nombre  canal  espejo  min  max  home  inv velMax   acel    p0us  p180us  fuente
static const ConfigServo SERVOS[NUM_EJES] = {
  { "Base",         0, -1,  0, 180, 90, false, 100.0f,  400.0f, 500, 2515, FUENTE_STICK_IZQ_X  },
  { "Hombro",       1, -1, 15, 165, 90, true,   70.0f,  300.0f, 610, 2535, FUENTE_STICK_IZQ_Y  },
  { "Codo",         2, -1, 15, 165, 90, true,   80.0f,  350.0f, 610, 2535, FUENTE_STICK_DER_Y  },
  { "Muneca-Pitch", 3, -1, 10, 170, 90, false, 100.0f,  400.0f, 610, 2535, FUENTE_STICK_DER_X  },
  { "Muneca-Roll",  4, -1, 10, 170, 90, false,  80.0f,  350.0f, 610, 2535, FUENTE_DPAD_H       },
  { "Pinza",        5,  6, 50, 115, 75, false,  90.0f,  500.0f, 610, 2535, FUENTE_GATILLOS_PINZA }
};

struct EstadoServo {
  float posicion, velocidad;
  uint16_t pulsoActual;
  bool pulsoValido;
};

struct EntradasMando {
  bool valido;
  int32_t axisX, axisY, axisRX, axisRY, brake, throttle;
  uint8_t dpad;
  bool a, b, x, y, l1, r1, l2, r2, start, select, system;
};

struct DetectorFlanco { bool anterior; unsigned long tCambio; };

enum FaseArranque { FASE_ARRANCANDO = 0, FASE_LISTO };
enum { PASOS_LUT = 100 };

static Adafruit_PWMServoDriver pwm(HW.dirPCA9685);
static EstadoServo estadoServo[NUM_EJES];
static EntradasMando entradas;
static ControllerPtr mandos[BP32_MAX_GAMEPADS];
static DetectorFlanco flancoStart = { false, 0 };

static bool pcaLista = false;
static bool modoHome = false;
static FaseArranque faseArranque = FASE_ARRANCANDO;

static uint16_t pulso0Ticks[NUM_EJES], pulso180Ticks[NUM_EJES];
static float lutCurva[PASOS_LUT + 1];

static unsigned long tInicioMs = 0, tUltimoTickMs = 0, tUltimoRefrescoMs = 0, tUltimoIntentoPCAMs = 0;

static inline float limitarF(float v, float lo, float hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }
static inline int limitarI(int v, int lo, int hi) { return (v < lo) ? lo : ((v > hi) ? hi : v); }

static void construirCurva() {
  for (int i = 0; i <= PASOS_LUT; i++) {
    lutCurva[i] = powf((float)i / (float)PASOS_LUT, CURVA.exponente);
  }
}

static float factorProgresivo(float magnitud, float zonaMuerta, float rango) {
  float m = fabsf(magnitud);
  if (m <= zonaMuerta) return 0.0f;
  float norm = (m - zonaMuerta) / (rango - zonaMuerta);
  if (norm > 1.0f) norm = 1.0f;
  return lutCurva[limitarI((int)(norm * PASOS_LUT + 0.5f), 0, PASOS_LUT)];
}

static uint16_t anguloAPulso(uint8_t i, uint8_t ang) {
  return (uint16_t)(pulso0Ticks[i] + ((int32_t)ang * (pulso180Ticks[i] - pulso0Ticks[i])) / 180);
}

static bool inicializarPCA() {
  if (!pwm.begin()) return false;
  pwm.setOscillatorFrequency(HW.osciladorPCA);
  pwm.setPWMFreq(HW.frecPWM);
  for (uint8_t i = 0; i < NUM_EJES; i++) estadoServo[i].pulsoValido = false;
  pcaLista = true;
  tUltimoRefrescoMs = millis();
  return true;
}

static void escribirServos(unsigned long ahora) {
  if (!pcaLista) return;
  const bool refresco = (ahora - tUltimoRefrescoMs) >= CTRL.periodoRefrescoMs;

  for (uint8_t i = 0; i < NUM_EJES; i++) {
    int ang = limitarI((int)lroundf(estadoServo[i].posicion), SERVOS[i].angMin, SERVOS[i].angMax);
    const uint16_t pulso = anguloAPulso(i, (uint8_t)ang);

    if (estadoServo[i].pulsoValido && pulso == estadoServo[i].pulsoActual && !refresco) continue;

    pwm.setPWM(SERVOS[i].canal, 0, pulso);
    if (SERVOS[i].canalEspejo >= 0) pwm.setPWM((uint8_t)SERVOS[i].canalEspejo, 0, pulso);
    
    estadoServo[i].pulsoActual = pulso;
    estadoServo[i].pulsoValido = true;
  }
  if (refresco) tUltimoRefrescoMs = ahora;
}

static ControllerPtr mandoActivo() {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (mandos[i] && mandos[i]->isConnected() && mandos[i]->isGamepad()) return mandos[i];
  }
  return nullptr;
}

static void leerEntradas(ControllerPtr ctl, EntradasMando& e) {
  e.axisX = ctl->axisX(); e.axisY = ctl->axisY(); e.axisRX = ctl->axisRX(); e.axisRY = ctl->axisRY();
  e.brake = ctl->brake(); e.throttle = ctl->throttle(); e.dpad = ctl->dpad();
  e.a = ctl->a(); e.b = ctl->b(); e.x = ctl->x(); e.y = ctl->y();
  e.l1 = ctl->l1(); e.r1 = ctl->r1(); e.l2 = ctl->l2(); e.r2 = ctl->r2();
  e.start = ctl->miscStart(); e.valido = true;
}

static float entradaEje(const EntradasMando& e, FuenteEntrada fuente) {
  switch (fuente) {
    case FUENTE_STICK_IZQ_X: {
      float f = factorProgresivo((float)e.axisX, CURVA.zonaMuertaStick, CURVA.rangoStick);
      return (e.axisX < 0) ? -f : f;
    }
    case FUENTE_STICK_IZQ_Y: {
      float f = factorProgresivo((float)e.axisY, CURVA.zonaMuertaStick, CURVA.rangoStick);
      return (e.axisY < 0) ? -f : f;
    }
    case FUENTE_STICK_DER_X: {
      float f = factorProgresivo((float)e.axisRX, CURVA.zonaMuertaStick, CURVA.rangoStick);
      return (e.axisRX < 0) ? -f : f;
    }
    case FUENTE_STICK_DER_Y: {
      float f = factorProgresivo((float)e.axisRY, CURVA.zonaMuertaStick, CURVA.rangoStick);
      return (e.axisRY < 0) ? -f : f;
    }
    case FUENTE_DPAD_H: {
      float v = 0.0f;
      if (BTN.rollConDPad) {
        if (e.dpad & DPAD_RIGHT) v += 1.0f;
        if (e.dpad & DPAD_LEFT)  v -= 1.0f;
      }
      if (BTN.rollIzqConX && e.x) v -= 1.0f;
      if (BTN.rollDerConB && e.b) v += 1.0f;
      return limitarF(v, -1.0f, 1.0f);
    }
    case FUENTE_GATILLOS_PINZA: {
      float cerrar = factorProgresivo((float)e.throttle, CURVA.zonaMuertaGatillo, CURVA.rangoGatillo);
      float abrir  = factorProgresivo((float)e.brake,    CURVA.zonaMuertaGatillo, CURVA.rangoGatillo);
      if (BTN.cerrarConR1 && e.r1) cerrar = 1.0f;
      if (BTN.abrirConL1  && e.l1) abrir  = 1.0f;
      return limitarF(cerrar - abrir, -1.0f, 1.0f);
    }
  }
  return 0.0f;
}

static bool pulsacionNueva(DetectorFlanco& d, bool actual, unsigned long ahora) {
  if (actual == d.anterior || (ahora - d.tCambio) < CTRL.antirreboteMs) return false;
  d.anterior = actual; d.tCambio = ahora;
  return actual;
}

static void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (mandos[i] == nullptr) {
      mandos[i] = ctl;
      ctl->setPlayerLEDs(1 << i);
      ctl->playDualRumble(0, 250, 0x60, 0x60);
      flancoStart.anterior = false; flancoStart.tCambio = millis();
      modoHome = false;
      return;
    }
  }
}

static void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (mandos[i] == ctl) {
      mandos[i] = nullptr;
      flancoStart.anterior = false; flancoStart.tCambio = millis();
      if (CTRL.homeAlDesconectar) modoHome = true;
      return;
    }
  }
}

static void tickControl(unsigned long ahora, float dt) {
  const float dtSeguro = (dt > 0.0001f) ? dt : 0.0001f;
  ControllerPtr ctl = mandoActivo();
  const bool hayMando = (ctl != nullptr);
  
  if (hayMando) leerEntradas(ctl, entradas);
  else { memset(&entradas, 0, sizeof(entradas)); entradas.valido = false; }

  if (hayMando && pulsacionNueva(flancoStart, entradas.start, ahora)) modoHome = !modoHome;

  if (faseArranque == FASE_ARRANCANDO && (ahora - tInicioMs) >= CTRL.retardoArranqueMs) {
    faseArranque = FASE_LISTO;
  }

  if (modoHome && hayMando && faseArranque == FASE_LISTO) {
    for (uint8_t i = 0; i < NUM_EJES; i++) {
      if (fabsf(entradaEje(entradas, SERVOS[i].fuente)) > CTRL.umbralCancelaHome) {
        modoHome = false; break;
      }
    }
  }

  bool todosEnCasa = true;

  for (uint8_t i = 0; i < NUM_EJES; i++) {
    const ConfigServo& cfg = SERVOS[i];
    EstadoServo& st = estadoServo[i];
    float velObjetivo = 0.0f;

    if (faseArranque == FASE_LISTO) {
      if (modoHome) {
        const float error = (float)cfg.angHome - st.posicion;
        if (fabsf(error) > CTRL.toleranciaHome) {
          const float v = fminf(CTRL.velHome, fabsf(error) / dtSeguro);
          velObjetivo = (error > 0.0f) ? v : -v;
          todosEnCasa = false;
        }
      } else if (hayMando && pcaLista) {
        const float u = entradaEje(entradas, cfg.fuente) * (cfg.invertir ? -1.0f : 1.0f);
        const float mag = fabsf(u);
        if (mag > 0.0001f) {
          const float velMin = cfg.velMax * CURVA.fraccionVelMin;
          const float vel = velMin + mag * (cfg.velMax - velMin);
          velObjetivo = (u < 0.0f) ? -vel : vel;
        }
      }
    }

    const float dv = cfg.acel * dtSeguro;
    if (st.velocidad < velObjetivo) st.velocidad = fminf(velObjetivo, st.velocidad + dv);
    else if (st.velocidad > velObjetivo) st.velocidad = fmaxf(velObjetivo, st.velocidad - dv);

    st.posicion += st.velocidad * dtSeguro;

    if (st.posicion <= (float)cfg.angMin) { st.posicion = (float)cfg.angMin; if (st.velocidad < 0.0f) st.velocidad = 0.0f; }
    if (st.posicion >= (float)cfg.angMax) { st.posicion = (float)cfg.angMax; if (st.velocidad > 0.0f) st.velocidad = 0.0f; }
  }

  if (modoHome && todosEnCasa) {
    modoHome = false;
    if (hayMando) ctl->playDualRumble(0, 150, 0x30, 0x30);
  }

  escribirServos(ahora);

  if (!pcaLista && (ahora - tUltimoIntentoPCAMs) >= HW.reintentoPCAms) {
    tUltimoIntentoPCAMs = ahora;
    inicializarPCA();
  }
}

void setup() {
  Serial.begin(115200);
  tInicioMs = millis();
  
  Wire.begin(HW.pinSDA, HW.pinSCL, HW.frecI2C);
  construirCurva();

  const float ticksPorUs = 4096.0f * (float)HW.frecPWM / 1000000.0f;
  for (uint8_t i = 0; i < NUM_EJES; i++) {
    pulso0Ticks[i] = (uint16_t)lroundf((float)SERVOS[i].pulso0us * ticksPorUs);
    pulso180Ticks[i] = (uint16_t)lroundf((float)SERVOS[i].pulso180us * ticksPorUs);
    estadoServo[i].posicion = (float)SERVOS[i].angHome;
    estadoServo[i].velocidad = 0.0f;
    estadoServo[i].pulsoActual = pulso0Ticks[i];
    estadoServo[i].pulsoValido = false;
  }

  inicializarPCA();

  BP32.setup(&onConnectedController, &onDisconnectedController);
  BP32.enableNewBluetoothConnections(true);

  tUltimoTickMs = millis();
  tUltimoRefrescoMs = tUltimoTickMs;
  faseArranque = FASE_ARRANCANDO;
}

void loop() {
  BP32.update();
  const unsigned long ahora = millis();
  const unsigned long transcurrido = ahora - tUltimoTickMs;

  if (transcurrido >= CTRL.periodoControlMs) {
    if (transcurrido > (unsigned long)(CTRL.periodoControlMs * 4)) tUltimoTickMs = ahora;
    else tUltimoTickMs += CTRL.periodoControlMs;

    float dt = (float)transcurrido * 0.001f;
    if (dt > CTRL.dtMaximo) dt = CTRL.dtMaximo;

    tickControl(ahora, dt);
  }
  vTaskDelay(1);
}