/*
  SCARA - VERIFICACION DE CINEMATICA (ESP32 DevKit, sin librerias)
  =================================================================
  Hace home con los finales de carrera, controla la posicion de cada eje con su
  encoder y lleva la punta al punto (x, y, z) que le pidas, con la cinematica
  inversa del modelo corregido (L1 = 120 mm, L2 = 188.4 mm).

  Pines (los de tu PCB):
    Hombro (M1, driver B2): RPWM 25  LPWM 26  encoder 16/17  FC 1 -> GPIO21
    Codo   (M2, driver B1): RPWM 27  LPWM 14  encoder 18/19  FC 2 -> GPIO22
    Eje Z  (M3, driver B3): RPWM 32  LPWM 33  encoder 4/23   FC Z -> GPIO15
    EN de los drivers -> GPIO5   Servo -> GPIO13   Boton BOOT (GPIO0) = parada
    Finales de carrera: C -> 3.3V, NO -> GPIO (presionado = 3.3V)

  Coordenadas (mm): origen en el eje del hombro, sobre la mesa.
    x = hacia donde apunta el brazo 1 cuando el hombro esta en 0
    y = 90 grados a la izquierda de x (visto desde arriba), z = altura de la punta.
    Angulos en grados, positivos en sentido antihorario visto desde arriba.

  Monitor serie 115200, "Nueva linea". Escribe "ayuda" para ver los comandos.
  Ver GUIA_VERIFICACION.md para calibrar y verificar paso a paso.
*/

// ======================= CONFIGURACION (se ajusta al calibrar) =======================
// Para cada eje:
//   cpu      : cuentas del encoder por unidad (por grado o por mm)  -> se calibra con "k"
//   dirHome  : signo del PWM que mueve el eje HACIA su final de carrera (+1 o -1)
//   homePos  : valor del eje cuando el final de carrera esta activo (grados o mm)
//   alejar   : +1 si al alejarse del switch el valor sube, -1 si baja
//   rango    : cuanto se puede alejar del home (grados o mm)
//   minPWM   : PWM minimo para que el motor arranque (zona muerta)
//   maxPWM   : PWM maximo al moverse a un punto
//   kp       : PWM por unidad de error (grado o mm)
//   tol      : tolerancia para considerar que llego (grados o mm)
//   velHome  : PWM durante el home
struct Config {
  const char* nombre; const char* unidad;
  int rpwm, lpwm, encA, encB, fc, canalR, canalL;
  float cpu; int dirHome; float homePos; int alejar; float rango;
  int minPWM, maxPWM; float kp, tol; int velHome;
};

Config CFG[3] = {
  // nombre    unidad  rpwm lpwm encA encB fc  cR cL   cpu  dirH homePos alejar rango minPWM maxPWM  kp   tol  velHome
  // (cuentas en cuadratura x4)
  // switch del hombro y del codo a la IZQUIERDA visto desde arriba -> al alejarse giran en sentido horario (angulo baja)
  // Angulos en el sistema del modelo: hombro medido desde el eje +x del mundo (a lo largo de la base);
  // codo medido desde el brazo estirado. Home: hombro -92 (transportador 178), codo +88 (transportador 178).
  {"hombro",   "deg",   25,  26,  16,  17, 21, 0, 1, 13.15,  -1,  -92.0,   -1, 120.0,   330,   450, 25.0, 1.0,  500},  // 13.15 cuentas/grado (verificado: home 176 -> 90 con polea asegurada); tope derecho a ~126 grados
  {"codo",     "deg",   27,  14,  18,  19, 22, 2, 3, 22.20,  -1,   88.0,   -1, 180.0,   300,   700, 30.0, 0.5,  450},  // motor 3: 22.2 cuentas/grado (verificado 90 grados); conectado al reves -> dirHome -1
  {"z",        "mm",    32,  33,   4,  23, 15, 4, 5, 25.50,  +1,  162.0,   +1,  85.0,   350,   600, 60.0, 0.5,  350},  // home ABAJO: punta a 162 mm de la mesa; sube hasta 85 mm; 25.5 cuentas/mm (verificado 30 mm)
};

const float L1 = 120.0;        // hombro -> codo (mm)
const float L2 = 188.6;        // codo -> centro de la cremallera (mm), del modelo "Gasebo mejor" (Robrot)
const float Z0_GZ = 70.3;      // modelo Gazebo: altura de la punta (mm) con eje_z = 0  ->  z = Z0_GZ + eje_z
const bool  FC_ACTIVO_EN_HIGH = true;
const int   PIN_EN = 5, PIN_SERVO = 13, PIN_BOTON = 0, CANAL_SERVO = 8;
const int   PINZA_CERRADA = 25, PINZA_ABIERTA = 145;   // grados del servo medidos (fuera de este rango se fuerza)
const int   PWM_FREQ = 20000, PWM_BITS = 10;
const int   RAMPA = 15;                    // cambio maximo de PWM cada 5 ms
const unsigned long T_ATASCO_MS = 1000;    // sin movimiento del encoder con PWM aplicado -> impulso / falla
const int   PWM_HOME_MAX = 950;          // tope del home cuando un eje se traba
const unsigned long T_HOME_MAX_MS = 8000;  // tiempo maximo buscando el switch (por seguridad)
const unsigned long PAUSA_RUTA_MS = 3000;  // pausa en cada punto de una ruta (para marcar)
// ======================================================================================

enum Estado { SIN_HOME, HOMING, QUIETO, MOVIENDO, FALLA };
const char* NOMBRE_ESTADO[] = {"sin_home", "homing", "quieto", "moviendo", "FALLA"};

struct Eje {
  long cuentas;            // copia de lectura
  int  encSigno;           // +1/-1: para que alejarse del switch cuente positivo
  long s;                  // cuentas desde el home (positivo = alejandose)
  long sObjetivo;
  bool homed;
  int  pwm;
  int  faseHome;           // 0 nada, 1 salir del switch, 2 buscar, 3 separarse
  unsigned long tFase, tUltMov;
  long sUltMov;
  bool activo;             // controlando posicion (sigue activo para sostener la posicion)
  long crudoInicio;        // para detectar el sentido del encoder durante el home
  int  intentos;
  long sMarca;             // referencia para calibrar con 'marca' + 'k'
  bool enPos;              // ya llego: no corrige hasta que el error pase de 2 x tol (evita zumbar)
  int  impulso;            // PWM extra cuando el eje se traba (friccion)
  bool reseteado;          // ya se reinicio el driver en este atasco
  bool repetir;            // home repetido: medir cuanto se desvio la cuenta
  int  pwmHome;            // PWM actual del home (sube solo si se traba)
};

volatile long encCnt[3] = {0, 0, 0};
Eje E[3];
Estado estado = SIN_HOME;
String motivoFalla = "";
bool reporte = false;
bool modoStream = false;   // objetivos continuos desde ROS ('qq'): sin mensajes ni reinicio de drivers
int  colaHome[3]; int nColaHome = 0;

// ruta (cuadrado u otros puntos)
float ruta[8][3]; int nRuta = 0, iRuta = -1; unsigned long tLlegada = 0; bool esperandoPausa = false;

#ifndef ESP_ARDUINO_VERSION_MAJOR
#define ESP_ARDUINO_VERSION_MAJOR 2
#endif
String logPwm = "";
void pwmIniciar(int pin, int canal, int freq, int bits) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // canal fijo: motores en canales 0..5 (grupo 0, comparten el timer de 20 kHz);
  // el servo en el canal 8 (grupo 1, timer propio) para que su 50 Hz no cambie el de los motores
  bool ok = ledcAttachChannel(pin, freq, bits, canal);
  logPwm += " " + String(pin) + (ok ? ":ok" : ":FALLO");
#else
  ledcSetup(canal, freq, bits); ledcAttachPin(pin, canal);
#endif
}
void pwmEscribir(int pin, int canal, int v) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(pin, v);
#else
  ledcWrite(canal, v);
#endif
}

// Encoders en cuadratura x4: interrupcion en A y en B, y una tabla de estados.
// Un pulso falso (ruido) se cancela solo, y los saltos invalidos (A y B a la vez) se ignoran.
DRAM_ATTR static const int8_t TABLA_Q[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile uint8_t encEst[3] = {0, 0, 0};
void IRAM_ATTR encPaso(int i, int pa, int pb) {
  uint8_t ns = (digitalRead(pa) << 1) | digitalRead(pb);
  encCnt[i] = encCnt[i] + TABLA_Q[(encEst[i] << 2) | ns];
  encEst[i] = ns;
}
// SOLO_A: para un encoder con el canal B fallando. Cuenta cada cambio de A (x2) y toma
// el sentido de la ultima orden dada al motor. (Codo: su canal B llega a ratos.)
DRAM_ATTR static const bool SOLO_A[3] = {false, false, false};   // poner true en un eje si su canal B falla
volatile int8_t dirMotor[3] = {1, 1, 1};
volatile uint8_t ultA[3] = {0, 0, 0};
volatile uint32_t tUltA[3] = {0, 0, 0};
void IRAM_ATTR encSoloA(int i, int pa) {
  uint8_t a = digitalRead(pa);
  if (a == ultA[i]) return;
  delayMicroseconds(20);                      // confirmar el nivel (los picos del PWM duran menos)
  if (digitalRead(pa) != a) return;
  uint32_t t = micros();
  if (t - tUltA[i] < 600) return;             // antirrebote: a toda velocidad un flanco real llega cada ~2.5 ms
  tUltA[i] = t; ultA[i] = a;
  encCnt[i] = encCnt[i] + dirMotor[i];
}
void IRAM_ATTR enc0() { encPaso(0, 16, 17); }
void IRAM_ATTR enc1a() { if (SOLO_A[1]) encSoloA(1, 18); else encPaso(1, 18, 19); }

// Muestreo por temporizador para los ejes SOLO_A (cada 100 us): un cambio de A solo cuenta
// si el nivel nuevo se mantiene 3 muestras seguidas. Cuenta bien tambien los flancos lentos.
volatile uint8_t filtA[3] = {0, 0, 0}, nEstable[3] = {0, 0, 0};
const int PIN_A[3] = {16, 18, 4};
void IRAM_ATTR muestreoA() {
  for (int i = 0; i < 3; i++) {
    if (!SOLO_A[i]) continue;
    uint8_t a = digitalRead(PIN_A[i]);
    if (a != filtA[i]) {
      nEstable[i] = nEstable[i] + 1;
      if (nEstable[i] >= 3) { filtA[i] = a; nEstable[i] = 0; encCnt[i] = encCnt[i] + dirMotor[i]; tUltA[i] = micros(); }
    } else nEstable[i] = 0;
  }
}
hw_timer_t *timerA = NULL;
void IRAM_ATTR enc1b() { if (!SOLO_A[1]) encPaso(1, 18, 19); }
void IRAM_ATTR enc2() { encPaso(2, 4, 23); }

// ----------------------------------------------------------------- utilidades de eje
bool fc(int i) { return digitalRead(CFG[i].fc) == (FC_ACTIVO_EN_HIGH ? HIGH : LOW); }

void salida(int i, int v) {
  Config &c = CFG[i];
  v = constrain(v, -1023, 1023);
  if (v > 0) dirMotor[i] = 1; else if (v < 0) dirMotor[i] = -1;   // si es 0 conserva el ultimo (inercia)
  if (v >= 0) { pwmEscribir(c.lpwm, c.canalL, 0); pwmEscribir(c.rpwm, c.canalR, v); }
  else        { pwmEscribir(c.rpwm, c.canalR, 0); pwmEscribir(c.lpwm, c.canalL, -v); }
}

// PWM con signo en "sentido alejar" (+ = se aleja del switch)
int pwmMotor(int i, int pAlejar) { return -CFG[i].dirHome * pAlejar; }

float posicion(int i) { return CFG[i].homePos + CFG[i].alejar * (float)E[i].s / CFG[i].cpu; }
long  sDesde(int i, float pos) { return lroundf((pos - CFG[i].homePos) * CFG[i].alejar * CFG[i].cpu); }
float limMin(int i) { return CFG[i].alejar > 0 ? CFG[i].homePos : CFG[i].homePos - CFG[i].rango; }
float limMax(int i) { return CFG[i].alejar > 0 ? CFG[i].homePos + CFG[i].rango : CFG[i].homePos; }
bool  dentro(int i, float p) { return p >= limMin(i) - 0.01 && p <= limMax(i) + 0.01; }

void leerEncoders() {
  noInterrupts();
  long c[3] = {encCnt[0], encCnt[1], encCnt[2]};
  interrupts();
  for (int i = 0; i < 3; i++) {
    long d = c[i] - E[i].cuentas;
    E[i].cuentas = c[i];
    E[i].s += d * E[i].encSigno;
  }
}

// El BTS7960 se bloquea solo si detecta sobrecorriente (p.ej. al frenar de golpe) y no vuelve
// a funcionar hasta que EN (INH) pasa por LOW. Se hace antes de cada movimiento nuevo.
void resetDrivers() {
  digitalWrite(PIN_EN, LOW);
  delay(3);
  digitalWrite(PIN_EN, HIGH);
  delay(1);
}

void pararTodo() {
  for (int i = 0; i < 3; i++) { E[i].pwm = 0; E[i].activo = false; E[i].faseHome = 0; salida(i, 0); }
  nColaHome = 0; iRuta = -1; nRuta = 0; esperandoPausa = false;
}

void falla(const String &m) {
  pararTodo();
  estado = FALLA;
  motivoFalla = m;
  Serial.println("\n!!! FALLA: " + m);
  Serial.println("    Corrige la causa y escribe 'reset'.\n");
}

// ----------------------------------------------------------------- cinematica
void fk(float q1, float q2, float &x, float &y) {
  float a = q1 * DEG_TO_RAD, b = (q1 + q2) * DEG_TO_RAD;
  x = L1 * cosf(a) + L2 * cosf(b);
  y = L1 * sinf(a) + L2 * sinf(b);
}

float envolver(float a) { while (a > 180) a -= 360; while (a < -180) a += 360; return a; }

// devuelve "" si hay solucion (en q1, q2) o el motivo
String ik(float x, float y, float &q1, float &q2) {
  float r2 = x * x + y * y;
  float c2 = (r2 - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  if (fabsf(c2) > 1) return "fuera de alcance (distancia " + String(sqrtf(r2), 1) + " mm; debe estar entre " +
                            String(fabsf(L2 - L1), 1) + " y " + String(L1 + L2, 1) + ")";
  float mejor = 1e9; bool hay = false;
  float q1a = posicion(0), q2a = posicion(1);
  for (int s = -1; s <= 1; s += 2) {
    float b = s * acosf(c2);
    float a = atan2f(y, x) - atan2f(L2 * sinf(b), L1 + L2 * cosf(b));
    float A = envolver(a * RAD_TO_DEG), B = b * RAD_TO_DEG;
    for (int k = -1; k <= 1; k++) {            // probar A, A+360, A-360 dentro de los limites
      float Ak = A + 360.0 * k;
      if (dentro(0, Ak) && dentro(1, B)) {
        float costo = fabsf(Ak - q1a) + fabsf(B - q2a);
        if (costo < mejor) { mejor = costo; q1 = Ak; q2 = B; hay = true; }
      }
    }
  }
  if (!hay) return "el punto solo se alcanza fuera de los limites del hombro/codo";
  return "";
}

// ----------------------------------------------------------------- movimientos
bool moverEje(int i, float pos) {
  if (!E[i].homed) { Serial.printf("  %s sin home\n", CFG[i].nombre); return false; }
  if (!dentro(i, pos)) {
    Serial.printf("  %s=%.1f fuera de limites [%.1f, %.1f] %s\n", CFG[i].nombre, pos, limMin(i), limMax(i), CFG[i].unidad);
    return false;
  }
  E[i].sObjetivo = sDesde(i, pos);
  E[i].activo = true;
  E[i].enPos = false;
  E[i].tUltMov = millis(); E[i].sUltMov = E[i].s;
  return true;
}

bool moverJoints(float a1, float a2, float z) {
  if (estado == FALLA || estado == HOMING) { Serial.println("  ocupado o en FALLA"); return false; }
  for (int i = 0; i < 3; i++) if (!E[i].homed) { Serial.println("  Primero haz 'home'"); return false; }
  if (!dentro(0, a1) || !dentro(1, a2) || !dentro(2, z)) {
    for (int i = 0; i < 3; i++) {
      float v = i == 0 ? a1 : i == 1 ? a2 : z;
      if (!dentro(i, v)) Serial.printf("  %s=%.1f fuera de [%.1f, %.1f]\n", CFG[i].nombre, v, limMin(i), limMax(i));
    }
    return false;
  }
  resetDrivers();                          // por si algun BTS7960 quedo bloqueado por sobrecorriente
  moverEje(0, a1); moverEje(1, a2); moverEje(2, z);
  estado = MOVIENDO;
  float x, y; fk(a1, a2, x, y);
  Serial.printf("-> objetivo: hombro=%.2f deg codo=%.2f deg z=%.1f mm  (punta x=%.1f y=%.1f)\n", a1, a2, z, x, y);
  return true;
}

bool moverXYZ(float x, float y, float z) {
  float q1, q2;
  String m = ik(x, y, q1, q2);
  if (m.length()) { Serial.println("  No se puede: " + m); return false; }
  return moverJoints(q1, q2, z);
}

// ----------------------------------------------------------------- home
void empezarHome(int i) {
  // si ya tenia home y el sentido del encoder es fijo (SOLO_A), se sigue contando para medir repetibilidad
  E[i].repetir = E[i].homed && SOLO_A[i];
  E[i].homed = false; E[i].activo = false;
  E[i].faseHome = fc(i) ? 1 : 2;          // si ya esta presionado, primero salir
  E[i].tFase = millis(); E[i].sUltMov = E[i].cuentas; E[i].tUltMov = millis();
  // se detecta solo durante el home; con un solo canal (SOLO_A) el sentido sale de la orden al motor
  if (!E[i].repetir) { E[i].encSigno = SOLO_A[i] ? -CFG[i].dirHome : 0; E[i].s = 0; }
  E[i].sMarca = 0; E[i].intentos = 0; E[i].pwmHome = CFG[i].velHome;
  E[i].crudoInicio = E[i].cuentas;
  Serial.printf("[home] %s...\n", CFG[i].nombre);
}

void pedirHome(int eje) {
  if (estado == FALLA) { Serial.println("  En FALLA: escribe 'reset'"); return; }
  pararTodo();
  nColaHome = 0;
  if (eje >= 0) colaHome[nColaHome++] = eje;
  else { colaHome[0] = 2; colaHome[1] = 1; colaHome[2] = 0; nColaHome = 3; }   // Z, codo, hombro
  resetDrivers();
  estado = HOMING;
  empezarHome(colaHome[0]);
}

// devuelve true cuando termino
bool pasoHome(int i) {
  Eje &e = E[i]; Config &c = CFG[i];
  unsigned long t = millis();
  long crudo = e.cuentas;
  long mov = crudo - e.crudoInicio;

  if (e.faseHome == 1) {                   // salir del switch (alejarse)
    salida(i, pwmMotor(i, c.velHome));
    // alejandose, s debe SUBIR
    if (e.encSigno == 0 && labs(mov) > 15) {
      e.encSigno = mov > 0 ? +1 : -1;
      Serial.printf("[home] %s: encoder %s\n", c.nombre, e.encSigno > 0 ? "normal" : "invertido (corregido solo)");
    }
    if (!fc(i) && t - e.tFase > 300) { e.faseHome = 2; e.tFase = t; e.crudoInicio = crudo; }
    if (t - e.tFase > 3000) { falla(String(c.nombre) + ": no logra salir del final de carrera"); return false; }
    return false;
  }
  if (e.faseHome == 2) {                   // buscar el switch
    salida(i, pwmMotor(i, -e.pwmHome));
    // si deja de avanzar 0.4 s, reiniciar el driver (proteccion de sobrecorriente del BTS7960)
    if (labs(crudo - e.sUltMov) > 2) { e.sUltMov = crudo; e.tUltMov = t; }
    else if (t - e.tUltMov > 400) {
      resetDrivers(); e.tUltMov = t;
      e.pwmHome = min(e.pwmHome + 75, PWM_HOME_MAX);   // mas fuerza solo si hace falta
      if (++e.intentos > 8) { falla(String(c.nombre) + ": no avanza hacia su final de carrera (trabado?)"); return false; }
    }
    // buscando, s debe BAJAR
    if (e.encSigno == 0 && labs(mov) > 15) {
      e.encSigno = mov < 0 ? +1 : -1;
      Serial.printf("[home] %s: encoder %s\n", c.nombre, e.encSigno > 0 ? "normal" : "invertido (corregido solo)");
    }
    if (e.encSigno == 0 && t - e.tFase > 1500) {
      falla(String(c.nombre) + ": el encoder no cuenta (revisa cables A/B, 5V y GND)"); return false;
    }
    if (fc(i) && e.encSigno == 0) {        // llego antes de saber el sentido del encoder: salir y repetir
      if (++e.intentos > 3) { falla(String(c.nombre) + ": el encoder no cuenta (revisa cables A/B, 5V y GND)"); return false; }
      e.faseHome = 1; e.tFase = t; e.crudoInicio = crudo;
      return false;
    }
    if (fc(i)) {
      salida(i, 0);
      if (e.repetir) Serial.printf("[home] %s: REPETIBILIDAD -> al tocar el switch leia %.2f %s (deberia ser %.2f): error %+.2f %s\n",
                                   c.nombre, posicion(i), c.unidad, c.homePos, posicion(i) - c.homePos, c.unidad);
      e.s = 0; e.homed = true; e.faseHome = 3; e.tFase = t;
      Serial.printf("[home] %s: final de carrera encontrado -> %s = %.1f %s\n", c.nombre, c.nombre, c.homePos, c.unidad);
      return false;
    }
    if (t - e.tFase > T_HOME_MAX_MS) {
      falla(String(c.nombre) + ": no encontro el final de carrera. Si se movio ALEJANDOSE del switch, cambia dirHome ('set " +
            String(i + 1) + " dirhome " + String(-c.dirHome) + "')");
      return false;
    }
    return false;
  }
  if (e.faseHome == 3) {                   // separarse un poco del switch
    e.faseHome = 0;
    moverEje(i, c.homePos + c.alejar * (c.unidad[0] == 'm' ? 1.0 : 3.0));
    return true;
  }
  return true;
}

// ----------------------------------------------------------------- control
void control() {
  for (int i = 0; i < 3; i++) {
    Eje &e = E[i]; Config &c = CFG[i];
    int deseado = 0;
    if (e.activo) {
      long err = e.sObjetivo - e.s;
      float errU = err / c.cpu;
      if (fabsf(errU) <= c.tol) { deseado = 0; e.enPos = true; }
      else if (e.enPos && fabsf(errU) <= 2 * c.tol) { deseado = 0; }
      else {
        float p = c.kp * errU;
        if (fabsf(p) < c.minPWM) p = p > 0 ? c.minPWM : -c.minPWM;
        deseado = constrain((int)p, -c.maxPWM, c.maxPWM);
      }
      // SOLO_A: no invertir el sentido mientras el brazo siga girando por inercia
      // (las cuentas se sumarian al reves); esperar 150 ms sin flancos
      if (SOLO_A[i] && deseado != 0 && (deseado > 0 ? 1 : -1) != (pwmMotor(i, 1) > 0 ? dirMotor[i] : -dirMotor[i])
          && micros() - tUltA[i] < 150000UL) deseado = 0;
      // seguridad: hacia el switch con el switch presionado
      if (deseado < 0 && fc(i)) { deseado = 0; e.sObjetivo = e.s; }
      // atasco: si no avanza, sube el PWM de a poco (impulso); solo es FALLA si ni al maximo se mueve
      if (deseado != 0) {
        unsigned long quieto = millis() - e.tUltMov;
        if (labs(e.s - e.sUltMov) > 2) { e.sUltMov = e.s; e.tUltMov = millis(); e.impulso = max(0, e.impulso - 10); }
        else if (quieto > 200) e.impulso = min(c.maxPWM, (int)(quieto / 200) * 50);
        if (quieto > 300 && !e.reseteado) { resetDrivers(); e.reseteado = true; }   // driver bloqueado?
        if (quieto < 50) e.reseteado = false;
        int mag = min(c.maxPWM, abs(deseado) + e.impulso);
        deseado = deseado > 0 ? mag : -mag;
        if (quieto > T_ATASCO_MS + 3000UL) {
          falla(String(c.nombre) + ": no se mueve ni con PWM maximo (trabado, sin alimentacion o encoder suelto)"); return;
        }
      } else { e.sUltMov = e.s; e.tUltMov = millis(); e.impulso = 0; }
    }
    // rampa solo al acelerar; al llegar o al cambiar de sentido frena de golpe (evita pasarse y oscilar)
    if (deseado == 0 || (long)deseado * e.pwm < 0) e.pwm = 0;
    else if (e.pwm == 0) e.pwm = (deseado > 0 ? 1 : -1) * min(abs(deseado), c.minPWM);   // arranca ya con fuerza
    else if (abs(deseado) > abs(e.pwm)) e.pwm += (deseado > 0 ? 1 : -1) * min(RAMPA, abs(deseado) - abs(e.pwm));
    else e.pwm = deseado;
    if (e.faseHome == 0) salida(i, pwmMotor(i, e.pwm));
  }
}

bool llegaron() {
  for (int i = 0; i < 3; i++) if (E[i].activo && (!E[i].enPos || E[i].pwm != 0)) return false;
  return true;
}

// ----------------------------------------------------------------- informes
void imprimirPos() {
  float a = posicion(0), b = posicion(1), z = posicion(2), x, y;
  fk(a, b, x, y);
  Serial.printf("estado=%s | hombro=%.2f deg  codo=%.2f deg  z=%.1f mm | punta (segun encoders): x=%.1f y=%.1f z=%.1f mm | FC:%d%d%d | pwm %d %d %d | activo %d%d%d\n",
                NOMBRE_ESTADO[estado], a, b, z, x, y, z, fc(0), fc(1), fc(2), E[0].pwm, E[1].pwm, E[2].pwm,
                E[0].activo, E[1].activo, E[2].activo);
}

void lineaVisor() {   // para el visor web: @,estado,q1,q2,z,t1,t2,tz,x,y,homed,fc
  float q[3], tq[3];
  for (int i = 0; i < 3; i++) {
    q[i] = posicion(i);
    tq[i] = E[i].activo ? CFG[i].homePos + CFG[i].alejar * (float)E[i].sObjetivo / CFG[i].cpu : q[i];
  }
  float x, y; fk(q[0], q[1], x, y);
  Serial.printf("@,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.1f,%.1f,%d%d%d,%d%d%d\n", NOMBRE_ESTADO[estado],
                q[0], q[1], q[2], tq[0], tq[1], tq[2], x, y,
                E[0].homed, E[1].homed, E[2].homed, fc(0), fc(1), fc(2));
}

void info() {   // para el visor web
  Serial.printf("#,%.3f,%.3f", L1, L2);
  for (int i = 0; i < 3; i++) Serial.printf(",%.2f,%.2f", limMin(i), limMax(i));
  Serial.println();
}

void imprimirCfg() {
  Serial.println("\n// ---- copia estas lineas en Config CFG[3] del codigo para guardar la calibracion ----");
  for (int i = 0; i < 3; i++) {
    Config &c = CFG[i];
    Serial.printf("  {\"%s\", \"%s\", %d, %d, %d, %d, %d, %d, %d, %.4f, %+d, %.2f, %+d, %.1f, %d, %d, %.1f, %.2f, %d},\n",
                  c.nombre, c.unidad, c.rpwm, c.lpwm, c.encA, c.encB, c.fc, c.canalR, c.canalL,
                  c.cpu, c.dirHome, c.homePos, c.alejar, c.rango, c.minPWM, c.maxPWM, c.kp, c.tol, c.velHome);
  }
  Serial.println();
}

void ayuda() {
  Serial.println(F(
    "\nCOMANDOS (mm y grados):\n"
    "  home            -> home de todos (Z, codo, hombro)   home 1 | home 2 | home z\n"
    "  xyz X Y Z       -> lleva la punta al punto, ej: xyz 200 100 100\n"
    "  --- en unidades del modelo Gazebo/ROS (radianes y metros) ---\n"
    "  gz H C D        -> joints como /scara_controller/commands: hombro(rad) codo(rad) eje_z(m)\n"
    "  gzxyz X Y Z     -> punta en metros, mismo sistema que terminal_ik_interface.py\n"
    "  gzpos           -> posicion actual en radianes y metros\n"
    "  q A1 A2 Z       -> mueve las articulaciones, ej: q 30 -45 100\n"
    "  jog E D         -> mueve el eje E (1, 2 o z) D unidades desde donde esta, ej: jog 1 10\n"
    "  pos             -> posicion actual (encoders) y punta calculada\n"
    "  cuadrado X Y L [Z] -> recorre un cuadrado de lado L con esquina en (X,Y), pausa 3 s en cada esquina\n"
    "  marca           -> guarda la posicion actual como referencia para calibrar\n"
    "  k E V           -> calibrar: el eje E se movio realmente V unidades desde la marca\n"
    "  set E campo V   -> cambiar config: cpu dirhome home alejar rango minpwm maxpwm kp tol velhome\n"
    "  cfg             -> imprime la configuracion para pegarla en el codigo\n"
    "  pwm E N ms      -> prueba directa sin control: PWM N (+ = alejarse del switch) durante ms\n"
    "  pinza G         -> servo a G grados (25 cerrada .. 145 abierta); tambien: pinza abrir / pinza cerrar\n"
    "  rep 1 / rep 0   -> envia o no la linea '@' para el visor web\n"
    "  s               -> PARAR todo (tambien el boton BOOT)   reset -> salir de FALLA\n"));
}

// ----------------------------------------------------------------- servo
void servo(int g) {
  g = constrain(g, 0, 180);
  long us = 500 + 1900L * g / 180;
  pwmEscribir(PIN_SERVO, CANAL_SERVO, (int)(us * 65535L / 20000));
}

// ----------------------------------------------------------------- comandos
int ejeDe(const String &t) {
  if (t == "1" || t == "hombro") return 0;
  if (t == "2" || t == "codo") return 1;
  if (t == "z" || t == "3") return 2;
  return -1;
}

int partir(String l, String *p, int max) {
  int n = 0; l.trim();
  while (l.length() && n < max) {
    int k = l.indexOf(' ');
    if (k < 0) { p[n++] = l; break; }
    p[n++] = l.substring(0, k);
    l = l.substring(k + 1); l.trim();
  }
  return n;
}

void comando(String l) {
  l.trim(); l.toLowerCase(); l.replace(",", " ");
  String p[6]; int n = partir(l, p, 6);
  if (n == 0) return;
  String c = p[0];
  // cualquier orden de movimiento manual sale del modo ROS (vuelven los mensajes de "llego")
  if (c != "qq" && c != "pinza" && c != "rep" && c != "pos" && c != "gzpos" && c != "info") modoStream = false;

  if (c == "s" || c == "stop") { pararTodo(); if (estado != FALLA) estado = E[0].homed && E[1].homed && E[2].homed ? QUIETO : SIN_HOME; Serial.println("-> PARADO"); }
  else if (c == "reset") { pararTodo(); estado = E[0].homed && E[1].homed && E[2].homed ? QUIETO : SIN_HOME; motivoFalla = ""; Serial.println("-> reset"); }
  else if (c == "ayuda" || c == "help" || c == "?") ayuda();
  else if (c == "home") pedirHome(n > 1 ? ejeDe(p[1]) : -1);
  else if (c == "pos") imprimirPos();
  else if (c == "info") info();
  else if (c == "cfg") imprimirCfg();
  else if (c == "rep") reporte = n > 1 && p[1] == "1";
  else if (c == "pinza" && n > 1) {
    int g;
    if (p[1] == "abrir" || p[1] == "a") g = PINZA_ABIERTA;
    else if (p[1] == "cerrar" || p[1] == "c") g = PINZA_CERRADA;
    else g = constrain((int)p[1].toInt(), PINZA_CERRADA, PINZA_ABIERTA);   // nunca forzar el servo
    servo(g); Serial.printf("-> pinza %d\n", g);
  }
  else if (c == "dbg") {
    Serial.println("PWM asignado:" + logPwm);
    Serial.printf("  EN=%d\n", digitalRead(PIN_EN));
  }
  else if (c == "pwm" && n >= 4) {   // prueba directa (lazo abierto): pwm E N ms  (N + = alejarse del switch)
    int i = ejeDe(p[1]); int v = constrain((int)p[2].toInt(), -1023, 1023); int ms = constrain((int)p[3].toInt(), 10, 2000);
    if (i < 0 || estado == HOMING) { Serial.println("  uso: pwm E N ms"); return; }
    pararTodo();
    if (v < 0 && fc(i)) { Serial.println("  el switch ya esta presionado hacia ese lado"); return; }
    leerEncoders(); long c0 = E[i].cuentas;
    if (n >= 5 && p[4] == "r") resetDrivers();
    salida(i, pwmMotor(i, v));
    unsigned long t0 = millis();
    // ademas cuenta por sondeo los flancos de A y de B y la cuenta en cuadratura x4
    static const int8_t TQ[4][4] = {{0, 1, -1, 0}, {-1, 0, 0, 1}, {1, 0, 0, -1}, {0, -1, 1, 0}};
    int ea = digitalRead(CFG[i].encA), eb = digitalRead(CFG[i].encB), st = (ea << 1) | eb;
    long fa = 0, fb = 0, q4 = 0, malos = 0;
    // version filtrada: un cambio solo cuenta si el nivel se mantiene en N lecturas seguidas (~10 us)
    const int N = 12;
    int fA = ea, fB = eb, cA = 0, cB = 0, fst = st; long ffa = 0, ffb = 0, fq4 = 0, fmalos = 0;
    bool parado = false;
    while (millis() - t0 < (unsigned long)ms + 150) {   // sigue contando 150 ms despues de parar (inercia)
      if (!parado && (millis() - t0 >= (unsigned long)ms || (v < 0 && fc(i)))) { salida(i, 0); parado = true; }
      int a = digitalRead(CFG[i].encA), b = digitalRead(CFG[i].encB);
      if (a != ea) { fa++; ea = a; }
      if (b != eb) { fb++; eb = b; }
      int ns = (a << 1) | b;
      if (ns != st) { if ((st ^ ns) == 3) malos++; else q4 += TQ[st][ns]; st = ns; }
      if (a != fA) { if (++cA >= N) { fA = a; cA = 0; ffa++; } } else cA = 0;
      if (b != fB) { if (++cB >= N) { fB = b; cB = 0; ffb++; } } else cB = 0;
      int fs = (fA << 1) | fB;
      if (fs != fst) { if ((fst ^ fs) == 3) fmalos++; else fq4 += TQ[fst][fs]; fst = fs; }
    }
    salida(i, 0); leerEncoders();
    Serial.printf("-> %s pwm %d (%s) %d ms: %+ld cuentas (programa) | sin filtro: A=%ld B=%ld x4=%+ld saltos=%ld | filtrado: A=%ld B=%ld x4=%+ld saltos=%ld | FC %d\n",
                  CFG[i].nombre, v, v >= 0 ? "alejar" : "hacia switch", ms, E[i].cuentas - c0, fa, fb, q4, malos,
                  ffa, ffb, fq4, fmalos, fc(i));
  }
  else if (c == "xyz" && n >= 4) moverXYZ(p[1].toFloat(), p[2].toFloat(), p[3].toFloat());
  // --- unidades del modelo Gazebo: hombro/codo en rad (0 = brazo estirado, + antihorario), eje_z en m ---
  else if (c == "gz" && n >= 4)
    moverJoints(p[1].toFloat() * RAD_TO_DEG, p[2].toFloat() * RAD_TO_DEG, Z0_GZ + p[3].toFloat() * 1000.0);
  else if (c == "gzxyz" && n >= 4) moverXYZ(p[1].toFloat() * 1000.0, p[2].toFloat() * 1000.0, p[3].toFloat() * 1000.0);
  // --- objetivo continuo desde ROS (puente_robot_real.py): grados, grados, mm; se recorta a los limites ---
  else if (c == "qq" && n >= 4) {
    if (estado == FALLA || estado == HOMING || !(E[0].homed && E[1].homed && E[2].homed)) return;
    float v[3] = {p[1].toFloat(), p[2].toFloat(), p[3].toFloat()};
    for (int i = 0; i < 3; i++) {
      float pos = constrain(v[i], limMin(i), limMax(i));
      long so = sDesde(i, pos);
      if (!E[i].activo || so != E[i].sObjetivo) {
        E[i].sObjetivo = so; E[i].activo = true; E[i].enPos = false;
        E[i].tUltMov = millis(); E[i].sUltMov = E[i].s;
      }
    }
    modoStream = true;
    estado = MOVIENDO;
  }
  else if (c == "gzpos") {
    float a = posicion(0), b = posicion(1), z = posicion(2), x, y; fk(a, b, x, y);
    Serial.printf("gz: hombro=%.4f rad codo=%.4f rad eje_z=%.4f m | punta x=%.4f y=%.4f z=%.4f m\n",
                  a * DEG_TO_RAD, b * DEG_TO_RAD, (z - Z0_GZ) / 1000.0, x / 1000.0, y / 1000.0, z / 1000.0);
  }
  else if (c == "q" && n >= 4) moverJoints(p[1].toFloat(), p[2].toFloat(), p[3].toFloat());
  else if (c == "jog" && n >= 3) {
    int i = ejeDe(p[1]);
    if (i < 0) { Serial.println("  eje: 1, 2 o z"); return; }
    if (estado == FALLA || estado == HOMING) { Serial.println("  ocupado o en FALLA"); return; }
    resetDrivers();
    if (moverEje(i, posicion(i) + p[2].toFloat())) { estado = MOVIENDO; Serial.printf("-> %s a %.2f %s\n", CFG[i].nombre, posicion(i) + p[2].toFloat(), CFG[i].unidad); }
  }
  else if (c == "cuadrado" && n >= 4) {
    float x = p[1].toFloat(), y = p[2].toFloat(), L = p[3].toFloat(), z = n >= 5 ? p[4].toFloat() : posicion(2);
    float pts[5][2] = {{x, y}, {x + L, y}, {x + L, y + L}, {x, y + L}, {x, y}};
    nRuta = 0;
    for (int k = 0; k < 5; k++) {
      float a, b;
      String m = ik(pts[k][0], pts[k][1], a, b);
      if (m.length()) { Serial.printf("  esquina (%.0f, %.0f): %s\n", pts[k][0], pts[k][1], m.c_str()); nRuta = 0; return; }
      ruta[nRuta][0] = pts[k][0]; ruta[nRuta][1] = pts[k][1]; ruta[nRuta][2] = z; nRuta++;
    }
    iRuta = 0; esperandoPausa = false;
    Serial.printf("-> cuadrado de %.0f mm: lados %.0f mm, diagonal %.1f mm. Marca cada esquina.\n", L, L, L * 1.41421);
    moverXYZ(ruta[0][0], ruta[0][1], ruta[0][2]);
  }
  else if (c == "marca") {
    for (int i = 0; i < 3; i++) E[i].sMarca = E[i].s;
    Serial.println("-> marca guardada en los 3 ejes. Marca en la mesa donde esta el brazo; luego usa jog y 'k E V'.");
  }
  else if (c == "k" && n >= 3) {
    int i = ejeDe(p[1]); float v = p[2].toFloat();
    if (i < 0 || !E[i].homed || fabsf(v) < 1) { Serial.println("  uso: k E V  (el eje debe tener home; V = lo que se movio desde la 'marca')"); return; }
    long mov = E[i].s - E[i].sMarca;
    float nuevo = labs(mov) / fabsf(v);
    Serial.printf("-> %s: se movio %ld cuentas en %.2f %s  =>  cpu %.4f -> %.4f\n", CFG[i].nombre, mov, v, CFG[i].unidad, CFG[i].cpu, nuevo);
    CFG[i].cpu = nuevo;
    Serial.println("   Guardado en RAM. Escribe 'cfg' para copiarlo al codigo.");
  }
  else if (c == "set" && n >= 4) {
    int i = ejeDe(p[1]); String f = p[2]; float v = p[3].toFloat();
    if (i < 0) { Serial.println("  eje: 1, 2 o z"); return; }
    Config &k = CFG[i];
    if (f == "cpu") k.cpu = v; else if (f == "dirhome") k.dirHome = v < 0 ? -1 : 1;
    else if (f == "home") k.homePos = v; else if (f == "alejar") k.alejar = v < 0 ? -1 : 1;
    else if (f == "rango") k.rango = v; else if (f == "minpwm") k.minPWM = v;
    else if (f == "maxpwm") k.maxPWM = v; else if (f == "kp") k.kp = v;
    else if (f == "tol") k.tol = v; else if (f == "velhome") k.velHome = v;
    else { Serial.println("  campo desconocido"); return; }
    Serial.printf("-> %s.%s = %.4f  (si cambiaste home/alejar/dirhome/cpu, vuelve a hacer 'home')\n", k.nombre, f.c_str(), v);
  }
  else Serial.println("?? escribe 'ayuda'");
}

void leerSerie() {
  static String l;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { if (l.length()) comando(l); l = ""; }
    else if (l.length() < 60) l += ch;
  }
}

void leerBoton() {
  static bool antes = HIGH; static unsigned long t = 0;
  bool b = digitalRead(PIN_BOTON);
  if (b != antes && millis() - t > 50) {
    t = millis(); antes = b;
    if (b == LOW) { pararTodo(); if (estado != FALLA) estado = E[0].homed && E[1].homed && E[2].homed ? QUIETO : SIN_HOME; Serial.println("\n-> BOOT: PARADO\n"); }
  }
}

// ----------------------------------------------------------------- setup / loop
void setup() {
  pinMode(PIN_EN, OUTPUT); digitalWrite(PIN_EN, LOW);
  for (int i = 0; i < 3; i++) {
    pwmIniciar(CFG[i].rpwm, CFG[i].canalR, PWM_FREQ, PWM_BITS);
    pwmIniciar(CFG[i].lpwm, CFG[i].canalL, PWM_FREQ, PWM_BITS);
    salida(i, 0);
    pinMode(CFG[i].encA, INPUT); pinMode(CFG[i].encB, INPUT);
    pinMode(CFG[i].fc, FC_ACTIVO_EN_HIGH ? INPUT_PULLDOWN : INPUT_PULLUP);
    memset(&E[i], 0, sizeof(Eje));
    E[i].encSigno = 1;
  }
  pwmIniciar(PIN_SERVO, CANAL_SERVO, 50, 16);
  servo(PINZA_CERRADA);          // pinza cerrada al arrancar (25 = cerrada, 145 = abierta)
  for (int i = 0; i < 3; i++) {
    encEst[i] = (digitalRead(CFG[i].encA) << 1) | digitalRead(CFG[i].encB);
    ultA[i] = digitalRead(CFG[i].encA);
  }
  attachInterrupt(digitalPinToInterrupt(16), enc0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(17), enc0, CHANGE);
  if (SOLO_A[1]) {                                         // codo: canal A por muestreo con temporizador
    filtA[1] = digitalRead(18);
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    timerA = timerBegin(1000000);                          // 1 MHz
    timerAttachInterrupt(timerA, &muestreoA);
    timerAlarm(timerA, 100, true, 0);                      // cada 100 us
#else
    timerA = timerBegin(1, 80, true);
    timerAttachInterrupt(timerA, &muestreoA, true);
    timerAlarmWrite(timerA, 100, true);
    timerAlarmEnable(timerA);
#endif
  } else attachInterrupt(digitalPinToInterrupt(18), enc1a, CHANGE);
  if (SOLO_A[1]) pinMode(19, INPUT_PULLDOWN);              // canal B del codo quemado: pin fijo en 0, sin interrupcion
  else attachInterrupt(digitalPinToInterrupt(19), enc1b, CHANGE);
  attachInterrupt(digitalPinToInterrupt(4),  enc2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(23), enc2, CHANGE);
  pinMode(PIN_BOTON, INPUT_PULLUP);
  Serial.begin(115200);
  delay(200);
  digitalWrite(PIN_EN, HIGH);
  Serial.println("\n=== SCARA - VERIFICACION DE CINEMATICA ===");
  Serial.printf("L1 = %.1f mm, L2 = %.1f mm. Escribe 'ayuda'. Primero: 'home'.\n", L1, L2);
  info();
}

void loop() {
  static unsigned long tCtl = 0, tRep = 0;
  leerSerie();
  leerBoton();

  if (millis() - tCtl >= 5) {
    tCtl = millis();
    leerEncoders();

    if (estado == HOMING && nColaHome > 0) {
      int i = colaHome[0];
      if (E[i].faseHome != 0) {
        if (pasoHome(i) && estado != FALLA) { /* separandose */ }
      } else if (!E[i].activo || llegaron()) {
        for (int k = 1; k < nColaHome; k++) colaHome[k - 1] = colaHome[k];
        nColaHome--;
        if (nColaHome > 0) empezarHome(colaHome[0]);
        else {
          estado = (E[0].homed && E[1].homed && E[2].homed) ? QUIETO : SIN_HOME;
          Serial.println("[home] listo");
          imprimirPos();
        }
      }
    }
    if (estado != FALLA) control();

    if (estado == MOVIENDO && llegaron()) {
      if (iRuta >= 0) {
        if (!esperandoPausa) {
          esperandoPausa = true; tLlegada = millis();
          Serial.printf("   esquina %d de %d: (%.0f, %.0f) -> marcala\n", iRuta + 1, nRuta, ruta[iRuta][0], ruta[iRuta][1]);
          imprimirPos();
        } else if (millis() - tLlegada > PAUSA_RUTA_MS) {
          esperandoPausa = false; iRuta++;
          if (iRuta < nRuta) moverXYZ(ruta[iRuta][0], ruta[iRuta][1], ruta[iRuta][2]);
          else { iRuta = -1; estado = QUIETO; Serial.println("-> cuadrado terminado"); }
        }
      } else {
        estado = QUIETO;                     // los ejes siguen activos sosteniendo la posicion
        if (!modoStream) { Serial.print("   llego. "); imprimirPos(); }
      }
    }
  }

  if (reporte && millis() - tRep >= 100) { tRep = millis(); lineaVisor(); }
}
