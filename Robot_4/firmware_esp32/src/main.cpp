#include <Arduino.h>

// ======================================================================
//  SCARA R4 - Firmware ESP32 (Arduino core 2.x o 3.x)
//  Lazo de posición de los 3 motores a 50 Hz con timer de hardware.
//
//  Protocolo serial, 115200 baudios, una línea por mensaje ('\n'):
//   PC -> ESP32
//     SET:c1,c2,c3       setpoints en cuentas para M1, M2, M3 (cierra el lazo)
//     STOP               apaga los motores (el conteo sigue). Limpia una FALLA
//     PRUEBA[:m[,v]]     prueba automática de sentido (+PWM y -PWM) con resultado
//     PWM:m,v            lazo abierto: motor m (1-3) con PWM v, para pruebas
//     MODO:LGR | MODO:P  cambia el controlador
//     ZERO               la posición actual pasa a ser la cuenta 0 (Home)
//     WD:ms              tiempo del watchdog (0 = desactivado)
//     TEL:1 | TEL:0      enciende o apaga la telemetría POS
//     INFO               imprime estado y configuración
//    Servo de la pinza (GPIO 23):
//     SERVO:grados       mueve el servo (0 a 180) con movimiento suave
//     SERVO_US:us        pulso en microsegundos (400 a 2600), para calibrar
//     SERVO:OFF          deja de mandar pulsos (el servo queda suelto)
//     PINZA:ABRIR | PINZA:CERRAR   va a los ángulos PINZA_ABIERTA / PINZA_CERRADA
//     PRUEBA_SERVO[:a,b] va de a a b dos veces (por defecto 0 y 180) y vuelve al centro
//   ESP32 -> PC
//     POS:t_ms,c1,c2,c3,pwm1,pwm2,pwm3   telemetría cada 20 ms
//     MSG:...   ERR:...
//
//  Convención: PWM positivo = las cuentas SUBEN. Si un motor no la cumple,
//  pongan invertir = true en su fila; si no, el lazo se desboca.
// ======================================================================

// ------------------------- CONFIGURACIÓN GENERAL -------------------------
const char* const  VERSION_FIRMWARE = "2026-11-18f (servo, sin proteccion de atasco ni de error grande)";
constexpr uint32_t BAUDIOS          = 115200;
constexpr uint32_t TS_US            = 20000;  // Ts = 20 ms -> 50 Hz
constexpr int      PWM_MAX          = 120;    // límite de seguridad de base y codo (máx. 255)
constexpr uint32_t PWM_FREQ_HZ      = 20000;
constexpr uint8_t  PWM_BITS         = 8;
constexpr int      PIN_STBY         = -1;     // GPIO del STBY de los TB6612, o -1 si va fijo a 3.3 V
constexpr uint32_t WATCHDOG_MS      = 500;    // sin comandos durante este tiempo -> motores apagados
// Protección de atasco: si un motor tiene PWM y sus cuentas no avanzan en T_ATASCO_MS -> FALLA.
// APAGADA en la versión final para que no interrumpa la tarea. Para volver a activarla: true.
constexpr bool     PROTECCION_ATASCO = false;
// Protección de error grande: si |error| > errorMax de un motor -> FALLA.
// APAGADA en la versión final: si el robot real se atrasa, sigue intentando alcanzar a Gazebo.
// Para volver a activarla: true.
constexpr bool     PROTECCION_ERROR  = false;
constexpr uint32_t T_ATASCO_MS      = 1000;   // PWM aplicado sin que avancen las cuentas -> FALLA
constexpr int32_t  MIN_AVANCE_ATASCO = 30;    // cuentas que tiene que avanzar en T_ATASCO_MS
                                              // (a PWM 55 avanzan más de 150; el ruido, 1 a 5)

// ------------------------- SERVO DE LA PINZA -------------------------
// Conector H1 SERVO: 1 = señal (GPIO 23), 2 = 5 V, 3 = GND.
constexpr int      PIN_SERVO           = 23;
constexpr uint8_t  CANAL_SERVO         = 6;      // canal LEDC propio: los motores usan 0-2 (otro timer)
constexpr uint32_t SERVO_FREQ_HZ       = 50;     // un pulso cada 20 ms, el estándar de los servos
constexpr uint8_t  SERVO_BITS          = 16;
// CALIBRAR (paso 3): pulso que da 0 grados y 180 grados en SU servo.
constexpr int      SERVO_US_MIN        = 500;
constexpr int      SERVO_US_MAX        = 2500;
constexpr int      SERVO_US_LIMITE_INF = 400;    // SERVO_US no deja salir de este rango
constexpr int      SERVO_US_LIMITE_SUP = 2600;
// Movimiento suave: el servo no salta de golpe (pide menos corriente al 7805).
constexpr float    SERVO_GRADOS_S      = 90.0f;
constexpr uint32_t T_PAUSA_SERVO_MS    = 1000;   // pausa en cada punto de PRUEBA_SERVO
// CALIBRAR (paso 5): ángulos de la pinza abierta y cerrada sobre la tapa.
constexpr int      PINZA_ABIERTA       = 105;    // 120 abría de más (choca con el tope)
constexpr int      PINZA_CERRADA       = 60;

enum TipoControl : uint8_t { CTRL_P, CTRL_LGR };
constexpr TipoControl CONTROL_INICIAL = CTRL_P;  // cambiar en caliente con MODO:LGR

// ------------------------- CONFIGURACIÓN POR MOTOR -------------------------
struct ConfigMotor {
  const char* nombre;
  bool  activo;            // false = se ignora (no necesita pines)
  int   pinPwm, pinIn1, pinIn2, pinEncA, pinEncB;
  bool  invertir;          // true si PWM positivo hace BAJAR las cuentas
  float cuentasPorUnidad;  // cuentas/rad (q1, q2) o cuentas/mm (d3)
  // LGR:  u[k] = u[k-1] + K*(e[k] - Z*e[k-1]),  e en rad (o mm)
  float lgrK, lgrZ;
  float pwmPorU;           // 255/12 si u está en voltios; 1 si u ya es PWM
  // P:    pwm = signo(e) * (pwmMin + Kp*|e|)
  float kp;                // PWM por rad (o por mm)
  int   pwmMin;            // PWM mínimo que vence la fricción: MEDIR con PWM:m,v
  float tolerancia;        // |e| menor que esto -> motor quieto (rad o mm)
  float errorMax;          // |e| mayor que esto -> FALLA y se apaga todo (rad o mm)
};

constexpr float V_A_PWM = 255.0f / 12.0f;

// ------------------- CONSTANTES DE LOS MOTORES -------------------
// El firmware cuenta solo los flancos de subida del canal A (x1), así que
// cuentas por vuelta de salida = pulsos del encoder * reducción.
//
// Base y codo: JGB37-520 12V 12RPM, reductor 810:1, encoder de 11 pulsos.
//   11 * 810 = 8910 cuentas/vuelta -> 1418.1 cuentas/rad
//   (Cuadra con la PRUEBA: 325 cuentas en 0.8 s a PWM 80 = 2.7 RPM, por debajo
//    del máximo de 3.8 RPM a ese PWM. Con 7 pulsos saldría más rápido que lo posible.)
constexpr float CUENTAS_RAD_JGB37 = 11.0f * 810.0f / 6.2831853f;
// Codo: correa de 20 a 60 dientes -> el codo gira un tercio de lo que gira el motor.
constexpr float CUENTAS_RAD_CODO  = CUENTAS_RAD_JGB37 * 60.0f / 20.0f;   // 4254.2
//
// Eje Z: N20 12V 30RPM, encoder de 7 pulsos, reductor ~1000:1 (es la única
//   reducción que cuadra con las 1000 cuentas en 0.8 s de la PRUEBA).
//   7 * 1000 = 7000 cuentas/vuelta. Engranes 17:17 -> sin reducción.
constexpr float CUENTAS_VUELTA_Z = 7.0f * 1000.0f * 17.0f / 17.0f;
// Husillo T8x8: paso 2 mm, 4 entradas -> avanza 8 mm por vuelta (confirmado).
constexpr float AVANCE_Z_MM_POR_VUELTA = 8.0f;
constexpr float CUENTAS_MM_Z = CUENTAS_VUELTA_Z / AVANCE_Z_MM_POR_VUELTA;   // 875

// Pines de su prueba zigzag. Asignación confirmada en el robot:
//   M1 = eje Z (prismático)  Driver U7 canal A   -> unidades en mm
//   M2 = codo, por correa    Driver U7 canal B   -> unidades en rad DEL CODO
//   M3 = base                Driver U1 canal A   -> unidades en rad
// El Kp del codo es el triple que el de la base (mismo esfuerzo por rad de motor).
// El Kp del Z (80 PWM/mm) da la misma rapidez de respuesta que la base: con lo
// medido en la PRUEBA (1.4 mm/s a PWM 80) el lazo queda con la misma ganancia.
// El LGR se diseñó sin la correa: en el codo la planta tiene la mitad de ganancia.
// Para apagar un motor mientras prueban otro, pongan activo = false en su fila.
constexpr ConfigMotor CFG[3] = {
  //  nombre        activo  PWM  IN1  IN2  ENC_A ENC_B invertir cuentas/u           K        Z        pwmPorU  Kp      pwmMin tol     errMax
  { "M1 (eje Z)",  true,   16,  33,  32,  13,   27,   true,    CUENTAS_MM_Z,       6.9327f, 0.0435f, V_A_PWM, 80.0f,  35,    0.1f,   10.0f },
  { "M2 (codo)",   true,   17,  25,  26,  22,   21,   true,    CUENTAS_RAD_CODO,   6.9327f, 0.0435f, V_A_PWM, 1200.0f, 35,    0.005f, 0.80f },
  { "M3 (base)",   true,   15,   4,   2,  12,   14,   false,   CUENTAS_RAD_JGB37,  6.9327f, 0.0435f, V_A_PWM, 400.0f, 35,    0.005f, 0.80f },
};

// Límite de PWM por motor (mismo orden que CFG). El N20 del Z es de 12 V y con el
// T8x8 da solo ~4 mm/s a tope, así que va sin límite. Base y codo quedan en PWM_MAX.
constexpr int PWM_MAX_MOTOR[3] = { 255, PWM_MAX, PWM_MAX };

static_assert(!CFG[0].activo || (CFG[0].pinPwm >= 0 && CFG[0].pinIn1 >= 0 && CFG[0].pinIn2 >= 0 &&
                                 CFG[0].pinEncA >= 0 && CFG[0].pinEncB >= 0), "M1 activo sin pines: llenen su fila");
static_assert(!CFG[1].activo || (CFG[1].pinPwm >= 0 && CFG[1].pinIn1 >= 0 && CFG[1].pinIn2 >= 0 &&
                                 CFG[1].pinEncA >= 0 && CFG[1].pinEncB >= 0), "M2 activo sin pines: llenen su fila");
static_assert(!CFG[2].activo || (CFG[2].pinPwm >= 0 && CFG[2].pinIn1 >= 0 && CFG[2].pinIn2 >= 0 &&
                                 CFG[2].pinEncA >= 0 && CFG[2].pinEncB >= 0), "M3 activo sin pines: llenen su fila");

// ------------------------------- ESTADO -------------------------------
struct EstadoMotor {
  int32_t setpoint   = 0;
  float   uPrev      = 0.0f;
  float   ePrev      = 0.0f;
  int     pwmAbierto = 0;
  int     pwm        = 0;   // último PWM aplicado (convención: + = cuentas suben)
  int32_t  cuentasRefAtasco = 0;  // cuentas al empezar la ventana de la protección de atasco
  int      signoAtasco      = 0;  // sentido del PWM en esa ventana (0 = PWM bajo, no se vigila)
  uint32_t msSinAvance      = 0;  // tiempo con PWM aplicado sin que avancen las cuentas
  uint32_t intRefAtasco     = 0;  // flancos del canal A al empezar la ventana (diagnóstico)
  int      pwmMaxAtasco     = 0;  // PWM más alto aplicado en la ventana (diagnóstico)
};

enum Modo : uint8_t { ESPERA, CONTROL, LAZO_ABIERTO, FALLA, PRUEBA };
const char* const NOMBRE_MODO[] = { "ESPERA", "CONTROL", "LAZO_ABIERTO", "FALLA", "PRUEBA" };

volatile int32_t cuentas[3] = { 0, 0, 0 };
volatile uint32_t interrupciones[3] = { 0, 0, 0 };   // flancos que llegan, antes del filtro
int pinA[3] = { -1, -1, -1 };              // copias en RAM para la ISR
int pinB[3] = { -1, -1, -1 };
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

EstadoMotor est[3];

// Prueba automática de sentido: +PWM, pausa, -PWM, pausa (por cada motor)
constexpr uint32_t T_PRUEBA_GIRO_MS  = 800;
constexpr uint32_t T_PRUEBA_PAUSA_MS = 500;
struct Prueba {
  int      motor = 0;
  int      fase  = 0;      // 0: +PWM  1: pausa  2: -PWM  3: pausa
  int      pwm   = 80;
  bool     todos = false;
  uint32_t t0    = 0;
  int32_t  c0    = 0;      // cuentas al empezar
  int32_t  c1    = 0;      // cuentas después del giro +
  uint32_t int0  = 0;      // interrupciones al empezar (diagnóstico)
  uint32_t muestras = 0, altosA = 0;   // muestras del canal A con el motor encendido
} prueba;
Modo        modo          = ESPERA;
TipoControl tipoControl   = CONTROL_INICIAL;
uint32_t    watchdogMs    = WATCHDOG_MS;
uint32_t    tUltimoCmd    = 0;
bool        telemetria    = false;   // arranca en silencio; el puente manda TEL:1

hw_timer_t*       timer          = nullptr;
volatile uint32_t ticks          = 0;
uint32_t          ticksAtendidos = 0;
uint32_t          ticksPerdidos  = 0;

// ---------------------------- INTERRUPCIONES ----------------------------
void IRAM_ATTR isrEncoder(void* arg) {
  const int i = (int)(intptr_t)arg;
  interrupciones[i] = interrupciones[i] + 1;
  if (!digitalRead(pinA[i])) return;   // pico de ruido del PWM: A ya volvió a bajo
  const int paso = digitalRead(pinB[i]) ? -1 : 1;
  portENTER_CRITICAL_ISR(&mux);
  cuentas[i] = cuentas[i] + paso;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR isrTimer() {
  ticks = ticks + 1;   // solo levanta la "bandera"; el control corre en loop()
}

// ---------------- COMPATIBILIDAD ARDUINO CORE 2.x / 3.x ----------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  #define CORE_V3 1
#else
  #define CORE_V3 0
#endif

void pwmIniciar(int i) {
#if CORE_V3
  ledcAttach(CFG[i].pinPwm, PWM_FREQ_HZ, PWM_BITS);
#else
  ledcSetup(i, PWM_FREQ_HZ, PWM_BITS);       // canal LEDC = índice del motor
  ledcAttachPin(CFG[i].pinPwm, i);
#endif
}

void pwmEscribir(int i, uint32_t duty) {
#if CORE_V3
  ledcWrite(CFG[i].pinPwm, duty);
#else
  ledcWrite(i, duty);
#endif
}

void iniciarTimer() {
#if CORE_V3
  timer = timerBegin(1000000);               // reloj del timer a 1 MHz
  timerAttachInterrupt(timer, &isrTimer);
  timerAlarm(timer, TS_US, true, 0);         // alarma cada 20 ms, autorecarga
#else
  timer = timerBegin(0, 80, true);           // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(timer, &isrTimer, true);
  timerAlarmWrite(timer, TS_US, true);
  timerAlarmEnable(timer);
#endif
}

// -------------------------------- SERVO --------------------------------
// Sin realimentación: "llegó" quiere decir que el pulso ya está en el objetivo
// (el servo de verdad tarda unas décimas de segundo más).
struct EstadoServo {
  bool     encendido     = false;   // false = sin pulsos (arranca así: no se sabe dónde está)
  float    us            = 1500.0f; // pulso que se está mandando
  float    objetivoUs    = 1500.0f;
  bool     avisarLlegada = false;
  uint32_t tLlegada      = 0;
  int      pruebaPaso    = -1;      // -1 = no hay PRUEBA_SERVO en curso
  float    puntos[5]     = { 0, 0, 0, 0, 0 };   // en grados
} servo;

float gradosAUs(float g) { return SERVO_US_MIN + (SERVO_US_MAX - SERVO_US_MIN) * g / 180.0f; }
float usAGrados(float us) { return (us - SERVO_US_MIN) * 180.0f / (SERVO_US_MAX - SERVO_US_MIN); }

void servoIniciar() {
#if CORE_V3
  ledcAttachChannel(PIN_SERVO, SERVO_FREQ_HZ, SERVO_BITS, CANAL_SERVO);
  ledcWrite(PIN_SERVO, 0);
#else
  ledcSetup(CANAL_SERVO, SERVO_FREQ_HZ, SERVO_BITS);
  ledcAttachPin(PIN_SERVO, CANAL_SERVO);
  ledcWrite(CANAL_SERVO, 0);
#endif
}

void servoPulso(float us) {   // us = 0 -> sin pulsos
  const float periodoUs = 1e6f / SERVO_FREQ_HZ;
  const uint32_t duty = (uint32_t)lroundf(us * (float)(1UL << SERVO_BITS) / periodoUs);
#if CORE_V3
  ledcWrite(PIN_SERVO, duty);
#else
  ledcWrite(CANAL_SERVO, duty);
#endif
}

void servoIr(float us) {
  us = constrain(us, (float)SERVO_US_LIMITE_INF, (float)SERVO_US_LIMITE_SUP);
  if (!servo.encendido) {
    // No se sabe dónde quedó el servo: el primer movimiento va directo (rápido)
    servo.us = us;
    servo.encendido = true;
    servoPulso(us);
  }
  servo.objetivoUs = us;
  servo.avisarLlegada = true;
  Serial.printf("MSG:servo -> %.0f grados (%.0f us)\n", usAGrados(us), us);
}

void servoApagar() {
  servo.encendido = false;
  servo.avisarLlegada = false;
  servo.pruebaPaso = -1;
  servoPulso(0);
  Serial.println("MSG:servo suelto (sin pulsos)");
}

void pasoServo() {
  if (!servo.encendido) return;
  const float paso = SERVO_GRADOS_S * (TS_US / 1e6f) * (SERVO_US_MAX - SERVO_US_MIN) / 180.0f;
  servo.us += constrain(servo.objetivoUs - servo.us, -paso, paso);
  servoPulso(servo.us);

  if (servo.avisarLlegada && fabsf(servo.objetivoUs - servo.us) < 0.5f) {
    servo.avisarLlegada = false;
    servo.tLlegada = millis();
    Serial.printf("MSG:servo en %.0f grados (%.0f us)\n", usAGrados(servo.us), servo.us);
  }

  // PRUEBA_SERVO: pasa al siguiente punto después de la pausa
  if (servo.pruebaPaso >= 0 && !servo.avisarLlegada && millis() - servo.tLlegada >= T_PAUSA_SERVO_MS) {
    servo.pruebaPaso++;
    if (servo.pruebaPaso >= 5) {
      servo.pruebaPaso = -1;
      Serial.println("MSG:PRUEBA_SERVO terminada");
    } else {
      servoIr(gradosAUs(servo.puntos[servo.pruebaPaso]));
    }
  }
}

void iniciarPruebaServo(float a, float b) {
  const float pts[5] = { a, b, a, b, (a + b) / 2.0f };
  for (int i = 0; i < 5; i++) servo.puntos[i] = pts[i];
  Serial.printf("MSG:PRUEBA_SERVO %.0f -> %.0f -> %.0f -> %.0f -> %.0f grados (pausa de %lu ms)\n",
                pts[0], pts[1], pts[2], pts[3], pts[4], (unsigned long)T_PAUSA_SERVO_MS);
  servo.pruebaPaso = 0;
  servoIr(gradosAUs(pts[0]));
}

// Lee un número de toda la cadena (sin basura al final)
bool leerNumero(const char* s, float& v) {
  char* fin = nullptr;
  v = strtof(s, &fin);
  return fin != s && *fin == '\0';
}

// ------------------------------- MOTORES -------------------------------
void leerCuentas(int32_t c[3]) {
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < 3; i++) c[i] = cuentas[i];
  portEXIT_CRITICAL(&mux);
}

void aplicarPwm(int i, int pwm) {
  pwm = constrain(pwm, -PWM_MAX_MOTOR[i], PWM_MAX_MOTOR[i]);
  est[i].pwm = pwm;
  if (!CFG[i].activo) return;

  const int salida = CFG[i].invertir ? -pwm : pwm;
  if (salida > 0) {
    digitalWrite(CFG[i].pinIn1, HIGH);
    digitalWrite(CFG[i].pinIn2, LOW);
  } else if (salida < 0) {
    digitalWrite(CFG[i].pinIn1, LOW);
    digitalWrite(CFG[i].pinIn2, HIGH);
  } else {
    digitalWrite(CFG[i].pinIn1, LOW);
    digitalWrite(CFG[i].pinIn2, LOW);
  }
  pwmEscribir(i, abs(salida));
}

void apagarTodo() {
  for (int i = 0; i < 3; i++) aplicarPwm(i, 0);
}

void reiniciarControladores() {
  for (int i = 0; i < 3; i++) {
    est[i].uPrev = 0.0f;
    est[i].ePrev = 0.0f;
    est[i].signoAtasco = 0;
    est[i].msSinAvance = 0;
  }
}

// ------------------------------ CONTROL ------------------------------
int calcularPwm(int i, float e) {
  const ConfigMotor& c = CFG[i];
  EstadoMotor& s = est[i];

  if (tipoControl == CTRL_LGR) {
    // u[k] = u[k-1] + K*(e[k] - Z*e[k-1])
    const float u   = s.uPrev + c.lgrK * (e - c.lgrZ * s.ePrev);
    const float pwm = constrain(u * c.pwmPorU, (float)-PWM_MAX_MOTOR[i], (float)PWM_MAX_MOTOR[i]);
    s.uPrev = pwm / c.pwmPorU;   // anti-windup: se guarda el u YA saturado
    s.ePrev = e;
    return (int)lroundf(pwm);
  }

  // Proporcional con compensación de zona muerta
  if (fabsf(e) <= c.tolerancia) return 0;
  const float mag = fminf((float)PWM_MAX_MOTOR[i], c.pwmMin + c.kp * fabsf(e));
  return (int)lroundf(e > 0.0f ? mag : -mag);
}

void fallaAtasco(int i, int32_t cuentasAhora) {
  const EstadoMotor& s = est[i];
  const long avance = (long)(cuentasAhora - s.cuentasRefAtasco) * s.signoAtasco;
  const unsigned long flancos = (unsigned long)(interrupciones[i] - s.intRefAtasco);
  apagarTodo();
  modo = FALLA;
  Serial.printf("ERR:FALLA %s: %lu ms con PWM de hasta %d y las cuentas avanzaron %ld (minimo %ld). "
                "Flancos del canal A en ese tiempo: %lu. Manden STOP\n",
                CFG[i].nombre, (unsigned long)T_ATASCO_MS, s.pwmMaxAtasco, avance,
                (long)MIN_AVANCE_ATASCO, flancos);
  if (flancos < 10) {
    Serial.printf("MSG:  -> casi no llegaron flancos: el motor NO GIRO (sin 12 V, cable del motor suelto o "
                  "eje trabado) o el canal A (GPIO %d) no llega\n", CFG[i].pinEncA);
  } else {
    Serial.printf("MSG:  -> el canal A si llega pero las cuentas no avanzan: canal B (GPIO %d) suelto "
                  "o con ruido\n", CFG[i].pinEncB);
  }
}

void falla(int i, float e, int32_t cuentasMotor) {
  apagarTodo();
  modo = FALLA;
  Serial.printf("ERR:FALLA %s setpoint=%ld cuentas=%ld ultimo_pwm=%d error=%.3f (limite %.3f). Manden STOP\n",
                CFG[i].nombre, (long)est[i].setpoint, (long)cuentasMotor, est[i].pwm, e, CFG[i].errorMax);
}

// --------------------- PRUEBA AUTOMÁTICA DE SENTIDO ---------------------
int pwmPrueba() {
  if (prueba.fase == 0) return prueba.pwm;
  if (prueba.fase == 2) return -prueba.pwm;
  return 0;
}

int siguienteMotorActivo(int desde) {
  for (int i = desde; i < 3; i++) {
    if (CFG[i].activo) return i;
  }
  return -1;
}

void iniciarPrueba(int motor, int pwm, bool todos) {
  int32_t c[3];
  leerCuentas(c);
  prueba.motor = motor;
  prueba.fase  = 0;
  prueba.pwm   = pwm;
  prueba.todos = todos;
  prueba.t0    = millis();
  prueba.c0    = c[motor];
  prueba.int0  = interrupciones[motor];
  prueba.muestras = prueba.altosA = 0;
  modo = PRUEBA;
  Serial.printf("MSG:PRUEBA %s (PWM +%d y luego -%d)\n", CFG[motor].nombre, pwm, pwm);
}

void veredictoPrueba(int i, int32_t cFinal) {
  const long d1 = (long)(prueba.c1 - prueba.c0);   // movimiento con +PWM
  const long d2 = (long)(cFinal - prueba.c1);      // movimiento con -PWM
  char v[140];
  const bool ok = (d1 >= 20 && d2 <= -20);   // tiene que contar en LOS DOS sentidos
  if (labs(d1) < 20 && labs(d2) < 20) {
    snprintf(v, sizeof(v), "NO CUENTA: no giro, o no llega el canal A del encoder (GPIO %d)", CFG[i].pinEncA);
  } else if (ok) {
    snprintf(v, sizeof(v), "OK");
  } else if (d1 <= -20 && d2 >= 20) {
    snprintf(v, sizeof(v), "INVERTIDO: cambien 'invertir' en la fila de este motor");
  } else {
    snprintf(v, sizeof(v), "UN SOLO SENTIDO: en un sentido no conto (cable del encoder flojo, "
                           "canal B GPIO %d) o el eje llego a un tope", CFG[i].pinEncB);
  }
  Serial.printf("MSG:RESULTADO %s  +PWM: %+ld cuentas | -PWM: %+ld cuentas  =>  %s\n",
                CFG[i].nombre, d1, d2, v);

  if (ok) return;   // OK: no hace falta diagnóstico

  // Diagnóstico del canal A mientras el motor tenía PWM
  const unsigned long flancos = (unsigned long)(interrupciones[i] - prueba.int0);
  const int pctAlto = prueba.muestras ? (int)(100UL * prueba.altosA / prueba.muestras) : -1;
  Serial.printf("MSG:  diagnostico canal A (GPIO %d): flancos recibidos=%lu, en alto %d%% del tiempo\n",
                CFG[i].pinEncA, flancos, pctAlto);
  if (flancos == 0 && pctAlto >= 95) {
    Serial.println("MSG:  -> A siempre en ALTO. Si el motor SI giro: encoder sin alimentacion (5V/GND) o cable de A suelto");
  } else if (flancos == 0 && pctAlto <= 5) {
    Serial.println("MSG:  -> A siempre en BAJO. Si el motor SI giro: cable de A en corto a GND o encoder sin alimentacion");
  } else if (flancos > 0 && labs(d1) < 20 && labs(d2) < 20) {
    Serial.println("MSG:  -> llegan solo picos cortos (ruido): la senal real del encoder no llega bien");
  }
}

void pasoPrueba(const int32_t c[3]) {
  if (prueba.fase == 0 || prueba.fase == 2) {   // motor encendido: muestrear el canal A
    prueba.muestras++;
    if (digitalRead(pinA[prueba.motor])) prueba.altosA++;
  }

  const uint32_t dur = (prueba.fase % 2 == 0) ? T_PRUEBA_GIRO_MS : T_PRUEBA_PAUSA_MS;
  if (millis() - prueba.t0 < dur) return;
  prueba.t0 = millis();

  const int i = prueba.motor;
  if (prueba.fase == 1) prueba.c1 = c[i];   // ya se detuvo después del giro +
  if (prueba.fase < 3) {
    prueba.fase++;
    return;
  }

  veredictoPrueba(i, c[i]);
  const int sig = prueba.todos ? siguienteMotorActivo(i + 1) : -1;
  if (sig < 0) {
    modo = ESPERA;
    Serial.println("MSG:PRUEBA terminada");
    return;
  }
  prueba.motor = sig;
  prueba.fase  = 0;
  prueba.c0    = c[sig];
  prueba.int0  = interrupciones[sig];
  prueba.muestras = prueba.altosA = 0;
  Serial.printf("MSG:PRUEBA %s (PWM +%d y luego -%d)\n", CFG[sig].nombre, prueba.pwm, prueba.pwm);
}

void pasoControl() {
  int32_t c[3];
  leerCuentas(c);

  if (modo == PRUEBA) pasoPrueba(c);

  if ((modo == CONTROL || modo == LAZO_ABIERTO) && watchdogMs > 0 &&
      millis() - tUltimoCmd > watchdogMs) {
    apagarTodo();
    modo = ESPERA;
    Serial.println("MSG:fin de la prueba (watchdog), motores detenidos");
  }

  for (int i = 0; i < 3; i++) {
    int pwm = 0;
    if (CFG[i].activo) {
      if (modo == CONTROL) {
        const float e = (float)(est[i].setpoint - c[i]) / CFG[i].cuentasPorUnidad;
        if (PROTECCION_ERROR && fabsf(e) > CFG[i].errorMax) {
          falla(i, e, c[i]);
          return;
        }
        pwm = calcularPwm(i, e);

        // Protección: con PWM claramente aplicado, las cuentas tienen que AVANZAR en el
        // sentido del PWM. No basta con ver si cambian: el ruido de un encoder sin señal
        // las mueve 1 o 2 cuentas y así el motor podía girar sin control mucho tiempo.
        const int signo = (PROTECCION_ATASCO && abs(pwm) >= CFG[i].pwmMin + 20) ? (pwm > 0 ? 1 : -1) : 0;
        EstadoMotor& s = est[i];
        if (signo == 0 || signo != s.signoAtasco) {
          s.signoAtasco = signo;            // ventana nueva
          s.cuentasRefAtasco = c[i];
          s.msSinAvance = 0;
          s.intRefAtasco = interrupciones[i];
          s.pwmMaxAtasco = abs(pwm);
        } else if ((c[i] - s.cuentasRefAtasco) * signo >= MIN_AVANCE_ATASCO) {
          s.cuentasRefAtasco = c[i];        // sí avanzó: ventana nueva
          s.msSinAvance = 0;
          s.intRefAtasco = interrupciones[i];
          s.pwmMaxAtasco = abs(pwm);
        } else {
          s.msSinAvance += TS_US / 1000;
          if (abs(pwm) > s.pwmMaxAtasco) s.pwmMaxAtasco = abs(pwm);
          if (s.msSinAvance > T_ATASCO_MS) {
            fallaAtasco(i, c[i]);
            return;
          }
        }
      } else if (modo == LAZO_ABIERTO) {
        pwm = est[i].pwmAbierto;
      } else if (modo == PRUEBA && i == prueba.motor) {
        pwm = pwmPrueba();
      }
    }
    aplicarPwm(i, pwm);
  }

  if (telemetria) {
    Serial.printf("POS:%lu,%ld,%ld,%ld,%d,%d,%d\n", (unsigned long)millis(),
                  (long)c[0], (long)c[1], (long)c[2], est[0].pwm, est[1].pwm, est[2].pwm);
  }
}

// ------------------------------ COMANDOS ------------------------------
void imprimirInfo() {
  int32_t c[3];
  leerCuentas(c);
  Serial.printf("MSG:firmware %s\n", VERSION_FIRMWARE);
  Serial.printf("MSG:modo=%s control=%s WD=%lums ticks_perdidos=%lu proteccion_atasco=%s proteccion_error=%s\n",
                NOMBRE_MODO[modo], tipoControl == CTRL_LGR ? "LGR" : "P",
                (unsigned long)watchdogMs, (unsigned long)ticksPerdidos,
                PROTECCION_ATASCO ? "ON" : "OFF", PROTECCION_ERROR ? "ON" : "OFF");
  for (int i = 0; i < 3; i++) {
    Serial.printf("MSG:%s activo=%d invertir=%d cuentas=%ld setpoint=%ld cuentas_por_unidad=%.2f pwm_max=%d\n",
                  CFG[i].nombre, CFG[i].activo, CFG[i].invertir, (long)c[i],
                  (long)est[i].setpoint, CFG[i].cuentasPorUnidad, PWM_MAX_MOTOR[i]);
  }
  if (servo.encendido) {
    Serial.printf("MSG:servo GPIO %d en %.0f grados (%.0f us) | 0-180 grados = %d-%d us | pinza abierta=%d cerrada=%d\n",
                  PIN_SERVO, usAGrados(servo.us), servo.us, SERVO_US_MIN, SERVO_US_MAX, PINZA_ABIERTA, PINZA_CERRADA);
  } else {
    Serial.printf("MSG:servo GPIO %d suelto (sin pulsos) | 0-180 grados = %d-%d us | pinza abierta=%d cerrada=%d\n",
                  PIN_SERVO, SERVO_US_MIN, SERVO_US_MAX, PINZA_ABIERTA, PINZA_CERRADA);
  }
}

bool rechazarEnFalla() {
  static uint32_t tAviso = 0;
  if (modo != FALLA) return false;
  if (millis() - tAviso > 1000) {   // no inundar el puerto si el puente sigue mandando
    tAviso = millis();
    Serial.println("ERR:en FALLA, manden STOP primero");
  }
  return true;
}

void procesarLinea(const char* l) {
  if (strncmp(l, "SET:", 4) == 0) {
    long a, b, c;
    if (sscanf(l + 4, "%ld,%ld,%ld", &a, &b, &c) != 3) {
      Serial.println("ERR:formato esperado SET:c1,c2,c3");
      return;
    }
    if (rechazarEnFalla()) return;
    if (modo != CONTROL) {
      reiniciarControladores();
      modo = CONTROL;
      Serial.println("MSG:lazo cerrado");
    }
    est[0].setpoint = a;
    est[1].setpoint = b;
    est[2].setpoint = c;
    tUltimoCmd = millis();

  } else if (strcmp(l, "STOP") == 0) {
    apagarTodo();
    modo = ESPERA;
    Serial.println("MSG:STOP");

  } else if (strncmp(l, "PWM:", 4) == 0) {
    int m, v;
    if (sscanf(l + 4, "%d,%d", &m, &v) != 2 || m < 1 || m > 3) {
      Serial.println("ERR:formato esperado PWM:m,v  (m = 1..3)");
      return;
    }
    if (rechazarEnFalla()) return;
    if (!CFG[m - 1].activo) {
      Serial.printf("ERR:M%d no esta activo\n", m);
      return;
    }
    for (int i = 0; i < 3; i++) est[i].pwmAbierto = 0;
    est[m - 1].pwmAbierto = constrain(v, -PWM_MAX_MOTOR[m - 1], PWM_MAX_MOTOR[m - 1]);
    modo = LAZO_ABIERTO;
    tUltimoCmd = millis();
    Serial.printf("MSG:lazo abierto M%d PWM=%d\n", m, est[m - 1].pwmAbierto);

  } else if (strcmp(l, "MODO:LGR") == 0 || strcmp(l, "MODO:P") == 0) {
    tipoControl = (l[5] == 'L') ? CTRL_LGR : CTRL_P;
    reiniciarControladores();
    Serial.printf("MSG:control=%s\n", tipoControl == CTRL_LGR ? "LGR" : "P");

  } else if (strcmp(l, "ZERO") == 0) {
    portENTER_CRITICAL(&mux);
    for (int i = 0; i < 3; i++) cuentas[i] = 0;
    portEXIT_CRITICAL(&mux);
    for (int i = 0; i < 3; i++) est[i].setpoint = 0;
    reiniciarControladores();
    Serial.println("MSG:posicion actual = Home (cuentas en 0)");

  } else if (strncmp(l, "WD:", 3) == 0) {
    watchdogMs = strtoul(l + 3, nullptr, 10);
    Serial.printf("MSG:watchdog=%lums%s\n", (unsigned long)watchdogMs, watchdogMs ? "" : " (desactivado)");

  } else if (strcmp(l, "TEL:1") == 0 || strcmp(l, "TEL:0") == 0) {
    telemetria = (l[4] == '1');
    Serial.printf("MSG:telemetria=%d\n", telemetria);

  } else if (strcmp(l, "PRUEBA") == 0 || strncmp(l, "PRUEBA:", 7) == 0) {
    // PRUEBA = los tres motores | PRUEBA:m = solo el motor m | PRUEBA:m,v = con PWM v
    int m = 0, v = 80;
    if (l[6] == ':' && sscanf(l + 7, "%d,%d", &m, &v) < 1) {
      Serial.println("ERR:formato esperado PRUEBA, PRUEBA:m o PRUEBA:m,v");
      return;
    }
    if (m < 0 || m > 3) {
      Serial.println("ERR:el motor debe ser 1, 2 o 3");
      return;
    }
    if (rechazarEnFalla()) return;
    v = constrain(abs(v), 20, m == 0 ? PWM_MAX : PWM_MAX_MOTOR[m - 1]);
    if (m == 0) {
      const int primero = siguienteMotorActivo(0);
      if (primero < 0) {
        Serial.println("ERR:no hay motores activos");
        return;
      }
      iniciarPrueba(primero, v, true);
    } else if (!CFG[m - 1].activo) {
      Serial.printf("ERR:M%d no esta activo\n", m);
    } else {
      iniciarPrueba(m - 1, v, false);
    }

  } else if (strcmp(l, "INFO") == 0) {
    imprimirInfo();

  // ----- Servo (no depende del modo de los motores: funciona también en FALLA) -----
  } else if (strcmp(l, "SERVO:OFF") == 0) {
    servoApagar();

  } else if (strncmp(l, "SERVO:", 6) == 0) {
    float g;
    if (!leerNumero(l + 6, g) || g < 0.0f || g > 180.0f) {
      Serial.println("ERR:formato esperado SERVO:grados (0 a 180) o SERVO:OFF");
      return;
    }
    servo.pruebaPaso = -1;
    servoIr(gradosAUs(g));

  } else if (strncmp(l, "SERVO_US:", 9) == 0) {
    float us;
    if (!leerNumero(l + 9, us) || us < SERVO_US_LIMITE_INF || us > SERVO_US_LIMITE_SUP) {
      Serial.printf("ERR:formato esperado SERVO_US:us (%d a %d)\n", SERVO_US_LIMITE_INF, SERVO_US_LIMITE_SUP);
      return;
    }
    servo.pruebaPaso = -1;
    servoIr(us);

  } else if (strcmp(l, "PINZA:ABRIR") == 0 || strcmp(l, "PINZA:CERRAR") == 0) {
    const bool abrir = (l[6] == 'A');
    Serial.printf("MSG:pinza %s\n", abrir ? "ABRIR" : "CERRAR");
    servo.pruebaPaso = -1;
    servoIr(gradosAUs(abrir ? PINZA_ABIERTA : PINZA_CERRADA));

  } else if (strcmp(l, "PRUEBA_SERVO") == 0 || strncmp(l, "PRUEBA_SERVO:", 13) == 0) {
    float a = 0.0f, b = 180.0f;
    if (l[12] == ':' && (sscanf(l + 13, "%f,%f", &a, &b) != 2 ||
                         a < 0.0f || a > 180.0f || b < 0.0f || b > 180.0f)) {
      Serial.println("ERR:formato esperado PRUEBA_SERVO o PRUEBA_SERVO:a,b (grados de 0 a 180)");
      return;
    }
    iniciarPruebaServo(a, b);

  } else if (l[0] != '\0') {
    Serial.printf("ERR:comando desconocido '%s'\n", l);
  }
}

void leerSerial() {
  static char buf[64];
  static size_t n = 0;
  while (Serial.available()) {
    const char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (n > 0) {
        buf[n] = '\0';
        procesarLinea(buf);
        n = 0;
      }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = (char)toupper((unsigned char)ch);
    } else {
      n = 0;   // línea demasiado larga: se descarta
    }
  }
}

// ---------------------------- SETUP / LOOP ----------------------------
void setup() {
  Serial.setTxBufferSize(1024);
  Serial.begin(BAUDIOS);
  delay(300);

  if (PIN_STBY >= 0) {
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);
  }

  for (int i = 0; i < 3; i++) {
    if (!CFG[i].activo) continue;

    pinMode(CFG[i].pinIn1, OUTPUT);
    pinMode(CFG[i].pinIn2, OUTPUT);
    digitalWrite(CFG[i].pinIn1, LOW);
    digitalWrite(CFG[i].pinIn2, LOW);

    pwmIniciar(i);
    pwmEscribir(i, 0);

    // Nota: los GPIO 34-39 no tienen pull-up interno
    pinA[i] = CFG[i].pinEncA;
    pinB[i] = CFG[i].pinEncB;
    pinMode(CFG[i].pinEncA, INPUT_PULLUP);
    pinMode(CFG[i].pinEncB, INPUT_PULLUP);
    attachInterruptArg(digitalPinToInterrupt(CFG[i].pinEncA), isrEncoder, (void*)(intptr_t)i, RISING);
  }

  servoIniciar();   // sin pulsos hasta el primer comando del servo
  iniciarTimer();

  Serial.println("MSG:SCARA R4 listo");
  imprimirInfo();
  Serial.println("MSG:telemetria apagada (TEL:1 para verla)");
  Serial.println("MSG:comandos: PRUEBA  PWM:m,v  SET:c1,c2,c3  STOP  ZERO  WD:ms  MODO:P|LGR  TEL:1|0  INFO");
  Serial.println("MSG:servo: SERVO:grados  SERVO_US:us  SERVO:OFF  PINZA:ABRIR|CERRAR  PRUEBA_SERVO[:a,b]");
  Serial.println("MSG:escriban UN comando por linea y esperen la respuesta antes del siguiente");
}

void loop() {
  const uint32_t t = ticks;
  if (t != ticksAtendidos) {
    ticksPerdidos += t - ticksAtendidos - 1;   // >0 si el loop se atrasó
    ticksAtendidos = t;
    pasoControl();
    pasoServo();
  }
  leerSerial();
}
