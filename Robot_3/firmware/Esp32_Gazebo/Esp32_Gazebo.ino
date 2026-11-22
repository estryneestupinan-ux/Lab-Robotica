#include <WiFi.h>
#include <WiFiUdp.h>

struct Paso { float m1, m2, m3; int servo; int espera; };
void irHome(bool home2); 
void abandonarOrdenPC(const char* porque);
void terminarOrdenPC();

// ===== WiFi / UDP =====
const char* WIFI_SSID = "RedmiUnU";     
const char* WIFI_PASS = "123456789";    
const int PUERTO_UDP  = 4210;

// ===== Coordinador de la celda=====
#define ROBOT 3
const char* PREFIJO = "R3 ";
const bool BUSCAR_PC = true;
IPAddress IP_PC(192, 168, 137, 1);      
const uint16_t PUERTO_PC = 5005;       
const uint16_t PUERTO_COORD = 6000 + ROBOT;  

WiFiUDP udp;               
WiFiUDP udpPC;                         
IPAddress ipPC;
bool pcConocido = false;              
bool coordConectado = false;        
unsigned long tHola = 0, tPosPC = 0;
String ultimaOrdenPC = "";                
String ordenEnCurso = "";               
String ordenEnCola = "";                 

IPAddress ipCliente;
uint16_t puertoCliente = 0;
bool hayCliente = false;


class Salida : public Print {
 public:
  using Print::write;
  size_t write(uint8_t c) override {
    Serial.write(c);
    if (hayCliente) {
      buf[n++] = c;
      if (c == '\n' || n >= sizeof(buf) - 1) enviar();
    }
    return 1;
  }
  void enviar() {
    if (n == 0) return;
    udp.beginPacket(ipCliente, puertoCliente);
    udp.write((const uint8_t*)buf, n);
    udp.endPacket();
    n = 0;
  }
 private:
  char buf[256];
  size_t n = 0;
};
Salida out;

// ===== Finales de carrera=====
int FC[3]      = {13, 4, 16};    
int fcMotor[3] = {0, 0, 1};         
bool fcCero[3] = {false, true, true};
int fcLado[3]  = {+1, -1, -1};     
const int FC_HOME_M2 = 2;            

// ===== Pines =====
int IN1[3]   = {32, 26, 17};
int IN2[3]   = {33, 27, 5};
int EN[3]    = {25, 14, 23};
int ENC_A[3] = {34, 36, 18};
int ENC_B[3] = {35, 39, 19};
#define HALL  15
#define SERVO 22

// ===== Datos =====
const float GRADOS_POR_PULSO = 360.0 / (17 * 119);
float factor[3] = {16.0 / 4250, -2.0 / 3, 3.0 / 7};
float cambio[3] = {0.5, 5, 5};       
const float M1_MIN = -17;
const float M1_MAX = 17;
const float DT = 0.01;             

const int   DIR_HOMING   = -1;    
const float VEL_HOMING   = 15;       
const unsigned long TIMEOUT_CAL = 15000; 
unsigned long tCal;
int homeDespuesDeCal = 0;           

float VEL_MAX[3] = {3.0, 60.0, 60.0};
float velocidad = 70;            

const float F_ACEL = 0.25;          
const float T_MIN  = 0.3;            

const float HOME_M2  = 0;     const float HOME_M3  = 0;
const float HOME2_M2 = 30;    const float HOME2_M3 = 0;
const float PLL_M2   = 15.5;  const float PLL_M3   = -14.5;
const float PS_M2    = 15.5;  const float PS_M3    = 165.5;

const float M1_ARRIBA = 16;            
const float M1_AGARRE = M1_ARRIBA - 8;   
const float M1_SOLTAR = M1_ARRIBA - 8;  
const int SERVO_ABIERTO = 90;            
const int SERVO_CERRADO = 30;            

const float GZ_PUNTOS[3][2] = {            
  {1.6275, 1.8523}, {1.9791, 1.7479}, {-1.3195, 1.9270}
};
const float REAL_PUNTOS[3][2] = {          
  {HOME_M3, HOME_M2}, {PLL_M3, PLL_M2}, {PS_M3, PS_M2}
};
const float GZ_ESCALA[2] = {-1.0500, -1.0};
const float GZ_OFFSET[2] = {1.5997, 1.8523};
const float Z_GZ_ARRIBA = 0.001;
const float Z_ESCALA = 0.020 / 0.08; 
const float GZ_GARRA_CERRADA = 0.24, GZ_GARRA_ABIERTA = 0.45;

const float M2_HOME    = 0;
const float M2_SENTIDO = 1;
bool calibrado[3] = {false, false, false};

float aReal(int i, float interno) {
  if (i == 1) return M2_HOME + M2_SENTIDO * interno;
  return interno;
}
float aInterno(int i, float real) {
  if (i == 1) return (real - M2_HOME) * M2_SENTIDO;
  return real;
}

const float NADA = -999;

const Paso SEC_HOME[] = {
  {M1_ARRIBA, NADA,    NADA,    -1, 0}, 
  {NADA,      HOME_M2, HOME_M3, -1, 0},  
};

const Paso SEC_HOME2[] = {
  {M1_ARRIBA, NADA,     NADA,     -1, 0},
  {NADA,      HOME2_M2, HOME2_M3, -1, 0}, 
};

const Paso SEC_PLL[] = {
  {M1_ARRIBA, NADA,   NADA,   -1, 0},    
  {NADA,      PLL_M2, PLL_M3, -1, 0},   
};

const Paso SEC_PS[] = {
  {M1_ARRIBA, NADA,  NADA,  -1, 0},       
  {NADA,      PS_M2, PS_M3, -1, 0},      
};

const Paso SEC_AGARRE[] = {
  {NADA,      NADA, NADA, SERVO_ABIERTO, 300},  
  {M1_AGARRE, NADA, NADA, -1,            0},   
  {NADA,      NADA, NADA, SERVO_CERRADO, 700}, 
};

const Paso SEC_SOLTAR[] = {
  {M1_SOLTAR, NADA, NADA, -1,            0},   
  {NADA,      NADA, NADA, SERVO_ABIERTO, 700},  
};

const Paso SEC_IR[] = {                          
  {M1_ARRIBA, NADA,   NADA,   SERVO_ABIERTO, 300},
  {NADA,      PLL_M2, PLL_M3, -1,            0},
  {M1_AGARRE, NADA,   NADA,   -1,            0},
  {NADA,      NADA,   NADA,   SERVO_CERRADO, 700},
  {M1_ARRIBA, NADA,   NADA,   -1,            0},
};
const Paso SEC_LLEVAR[] = {                     
  {M1_ARRIBA, NADA,  NADA,  -1,            0},
  {NADA,      PS_M2, PS_M3, -1,            0},
  {M1_SOLTAR, NADA,  NADA,  -1,            0},
  {NADA,      NADA,  NADA,  SERVO_ABIERTO, 700},
  {M1_ARRIBA, NADA,  NADA,  -1,            0},
};
const Paso SEC_VOLVER[] = {                     
  {M1_ARRIBA, NADA,    NADA,    -1,            0},
  {NADA,      HOME_M2, HOME_M3, -1,            0},
  {NADA,      NADA,    NADA,    SERVO_CERRADO, 300},
};
Paso secMueve[3];                              

#define CANT(x) (sizeof(x) / sizeof(Paso))

const float TOL[3] = {0.3, 1.5, 3.0}; 
const float TOL_MAX[3] = {1.0, 5.0, 8.0};
const unsigned long MARGEN_PASO = 5000;     

const Paso* sec = nullptr;
const char* nombreSec = "";
int nPasos = 0, pasoActual = -1;
bool movio[3];
unsigned long tPaso, limitePaso;

// ===== Maquina de estados =====
#define E_DESCONOCIDO 0
#define E_CALIBRANDO  1
#define E_LISTO       2  
#define E_PLL_ARRIBA  3
#define E_PLL_ABAJO   4
#define E_PS_ARRIBA   5
#define E_PS_ABAJO    6
const char* NOMBRE_ESTADO[] = {
  "DESCONOCIDO", "CALIBRANDO", "LISTO", "PLL arriba", "PLL abajo", "PS arriba", "PS abajo"
};

#define C_PLL    0
#define C_AGARRE 1
#define C_PS     2
#define C_SOLTAR 3
const char* NOMBRE_CMD[] = {"PLL", "AGARRE PLL", "PS", "SOLTAR PS"};

int estado = E_DESCONOCIDO;
bool tienePieza = false;
int estadoDestino = E_DESCONOCIDO;
int piezaDestino = -1;                

// ===== PID =====
float num[3] = {-412.7971, 784.29, -372.4525};
float den[3] = {1, -0.6027, -0.3973};
float err[3][3], co[3][3];

// ===== Variables =====
volatile long pulsos[3];
volatile bool ultA[3], ultB[3];
float pos[3], impreso[3];           
float meta[3];                      
float ref[3];                       

// Trayectoria actual
float q0[3], dq[3];                   
float T_mov = 0;                  
unsigned long tInicioMov;
bool moviendo = false;

bool fcAntes[3], hallAntes;
bool hayCambio = true;               

int servoGrados = 90;
int pulsoUS = 1500;
bool servoAlto = false;
unsigned long tServo, tControl;
String comando = "";
bool ipAvisada = false;

unsigned long tPos = 0;
const unsigned long PERIODO_POS = 100;  

void IRAM_ATTR leerEncoder(int i) {
  bool a = digitalRead(ENC_A[i]);
  bool b = digitalRead(ENC_B[i]);
  if (a != ultA[i]) { if (a != b) pulsos[i]++; else pulsos[i]--; }
  if (b != ultB[i]) { if (a == b) pulsos[i]++; else pulsos[i]--; }
  ultA[i] = a;
  ultB[i] = b;
}
void IRAM_ATTR encoder1() { leerEncoder(0); }
void IRAM_ATTR encoder2() { leerEncoder(1); }
void IRAM_ATTR encoder3() { leerEncoder(2); }

void moverMotor(int i, float voltaje) {
  int pwm = fabs(voltaje) * 255 / 12.33;
  if (pwm > 255) pwm = 255;
  digitalWrite(IN1[i], voltaje > 0);
  digitalWrite(IN2[i], voltaje < 0);
  analogWrite(EN[i], pwm);
}

float PID(int i, float error) {
  err[i][2] = err[i][1];
  err[i][1] = err[i][0];
  err[i][0] = error;
  co[i][2] = co[i][1];
  co[i][1] = co[i][0];
  co[i][0] = err[i][0]*num[0] + err[i][1]*num[1] + err[i][2]*num[2]
           - co[i][1]*den[1] - co[i][2]*den[2];
  if (co[i][0] > 12) co[i][0] = 12;
  if (co[i][0] < -12) co[i][0] = -12;
  return co[i][0];
}

void detener(int i) {
  ref[i] = pos[i];
  meta[i] = pos[i];
  q0[i] = pos[i];                    
  dq[i] = 0;
  for (int k = 0; k < 3; k++) { err[i][k] = 0; co[i][k] = 0; }
}

float perfilTrapecio(float tau) {
  const float f = F_ACEL;
  if (tau <= 0) return 0;
  if (tau >= 1) return 1;
  float k = 1.0 / (2.0 * f * (1.0 - f));
  if (tau < f) return k * tau * tau;                    
  if (tau < 1.0 - f) return (tau - f / 2.0) / (1.0 - f); 
  float r = 1.0 - tau;
  return 1.0 - k * r * r;                               
}

void iniciarTrayectoria() {
  float T = 0;
  for (int i = 0; i < 3; i++) {
    q0[i] = ref[i];
    dq[i] = meta[i] - ref[i];
    float v = VEL_MAX[i] * velocidad / 100.0;
    float Ti = fabs(dq[i]) / (v * (1.0 - F_ACEL));
    if (Ti > T) T = Ti;
  }

  if (T < 0.001) {                  
    moviendo = false;
    T_mov = 0;
    return;
  }
  if (T < T_MIN) T = T_MIN;

  T_mov = T;
  tInicioMov = micros();
  moviendo = true;
  out.print("Trayectoria coordinada: T = "); out.print(T_mov, 2); out.println(" s");
}

void actualizarTrayectoria() {
  if (!moviendo) return;
  float tau = (micros() - tInicioMov) / 1e6 / T_mov;
  if (tau >= 1) { tau = 1; moviendo = false; }
  float s = perfilTrapecio(tau);
  for (int i = 0; i < 3; i++) ref[i] = q0[i] + dq[i] * s;
}

void ponerServo(int grados) {
  servoGrados = constrain(grados, 0, 180);
  pulsoUS = map(servoGrados, 0, 180, 500, 2500);
  hayCambio = true;
}

void terminarCalibracion() {
  pulsos[1] = 0;
  pos[1] = 0;
  detener(1);
  calibrado[1] = true;
  tienePieza = false;
  estado = E_LISTO;
  hayCambio = true;
  out.println("Calibrado: M2 en FC3 = 0 (home). Listo para el ciclo.");
  if (homeDespuesDeCal) {             
    bool h2 = (homeDespuesDeCal == 2);
    homeDespuesDeCal = 0;
    irHome(h2);
  }
}

void iniciarCalibracion() {
  abandonarOrdenPC("se pidio calibrar");
  pasoActual = -1;                   
  moviendo = false;
  for (int i = 0; i < 3; i++) detener(i);
  estado = E_CALIBRANDO;
  hayCambio = true;
  tCal = millis();
  out.println("Calibrando: moviendo M2 hacia el FC3...");
  if (digitalRead(FC[FC_HOME_M2]) == LOW) terminarCalibracion(); 
}

void calibrar() {
  if (estado != E_CALIBRANDO) return;

  if (fcAntes[FC_HOME_M2]) {       
    terminarCalibracion();
    return;
  }
  if (millis() - tCal > TIMEOUT_CAL) {
    detener(1);
    estado = E_DESCONOCIDO;
    hayCambio = true;
    homeDespuesDeCal = 0;
    out.println("ERROR: no encontro el FC3. Revisa DIR_HOMING o el final de carrera y usa CAL.");
    return;
  }
  ref[1] += DIR_HOMING * VEL_HOMING * DT;
  meta[1] = ref[1];
}

bool permitido(int c) {
  switch (c) {
    case C_PLL:
    case C_PS:     return calibrado[1] && estado != E_CALIBRANDO;
    case C_AGARRE: return estado == E_PLL_ARRIBA;
    case C_SOLTAR: return estado == E_PS_ARRIBA;
  }
  return false;
}

const char* motivo(int c) {
  if (c == C_PLL || c == C_PS) return "el M2 no esta calibrado: usa CAL o HOME";
  if (c == C_AGARRE) return "AGARRE PLL solo se puede estando en PLL arriba: manda PLL primero";
  return "SOLTAR PS solo se puede estando en PS arriba: manda PS primero";
}

bool ocupado() {
  return pasoActual >= 0 || moviendo || estado == E_CALIBRANDO;
}

void imprimirEstado() {
  out.print("Estado: "); out.print(NOMBRE_ESTADO[estado]);
  out.print(tienePieza ? " (con pieza)" : " (sin pieza)");
  if (pasoActual >= 0 || moviendo) { out.print(" | ejecutando: "); out.print(nombreSec); }
  out.print(" | permitidos: HOME, HOME2, CAL");
  for (int c = 0; c < 4; c++)
    if (permitido(c)) { out.print(", "); out.print(NOMBRE_CMD[c]); }
  out.println();
}

void avisarOcupado() {
  out.print("Ocupado ");
  if (estado == E_CALIBRANDO) out.print("calibrando");
  else { out.print("ejecutando "); out.print(nombreSec); }
  out.println(". Espera, o usa HOME / HOME2 / STOP.");
}

void iniciarPaso() {
  const Paso& p = sec[pasoActual];
  float v[3] = {p.m1, p.m2, p.m3};
  bool algo = false;
  for (int i = 0; i < 3; i++) {
    movio[i] = (v[i] != NADA);
    if (movio[i]) { meta[i] = aInterno(i, v[i]); algo = true; }
  }
  if (algo) iniciarTrayectoria();
  if (p.servo >= 0) ponerServo(p.servo);
  tPaso = millis();
  limitePaso = (algo ? (unsigned long)(T_mov * 1000) : 0) + p.espera + MARGEN_PASO;
  out.print("  Paso "); out.print(pasoActual + 1);
  out.print("/"); out.println(nPasos);
}

void empezar(const Paso* s, int n, const char* nombre, int destino, int pieza) {
  if (!calibrado[1]) out.println("OJO: M2 sin calibrar, usa CAL");
  sec = s;
  nPasos = n;
  nombreSec = nombre;
  estadoDestino = destino;
  piezaDestino = pieza;
  pasoActual = 0;
  out.print("Secuencia "); out.println(nombre);
  iniciarPaso();
}

void abandonarOrdenPC(const char* porque) {
  if (ordenEnCurso.length() == 0) return;
  out.print("Coordinador: la orden '"); out.print(ordenEnCurso);
  out.print("' NO se termino ("); out.print(porque); out.println("). No se manda LISTO.");
  ordenEnCurso = "";
}

void cancelarSecuencia() {
  abandonarOrdenPC("secuencia cancelada");
  if (pasoActual >= 0) {
    estado = E_DESCONOCIDO;
    hayCambio = true;
  }
  pasoActual = -1;
}

bool llego(int i) {
  return !moviendo && fabs(pos[i] - meta[i]) < TOL[i];
}

void revisarSecuencia() {
  if (pasoActual < 0) return;

  bool listo = millis() - tPaso >= (unsigned long)sec[pasoActual].espera;
  for (int i = 0; i < 3; i++)
    if (movio[i] && !llego(i)) listo = false;

  if (!listo) {
    if (millis() - tPaso <= limitePaso) return; 

    bool cerca = !moviendo;
    for (int i = 0; i < 3; i++)
      if (movio[i] && fabs(pos[i] - meta[i]) >= TOL_MAX[i]) cerca = false;

    for (int i = 0; i < 3; i++)
      if (movio[i] && !llego(i)) {      
        out.print(cerca ? "AVISO: M" : "  M"); out.print(i + 1);
        out.print(" quedo en "); out.print(aReal(i, pos[i]), 1);
        out.print(" (meta "); out.print(aReal(i, meta[i]), 1);
        out.println(cerca ? "), se acepta y sigue" : ")");
      }

    if (!cerca) {
      out.println("Secuencia cancelada: un motor quedo muy lejos. Usa HOME.");
      cancelarSecuencia();
      return;
    }
  }

  pasoActual++;
  if (pasoActual >= nPasos) {
    pasoActual = -1;
    estado = estadoDestino;
    if (piezaDestino >= 0) tienePieza = piezaDestino;
    hayCambio = true;
    out.print("Secuencia terminada. ");
    imprimirEstado();
    terminarOrdenPC();            
  } else iniciarPaso();
}

void pedir(int c) {
  if (ocupado()) { avisarOcupado(); return; }
  if (!permitido(c)) {
    out.print("No se puede "); out.print(NOMBRE_CMD[c]);
    out.print(": "); out.println(motivo(c));
    imprimirEstado();
    return;
  }
  switch (c) {
    case C_PLL:    empezar(SEC_PLL,    CANT(SEC_PLL),    "PLL",        E_PLL_ARRIBA, -1); break;
    case C_AGARRE: empezar(SEC_AGARRE, CANT(SEC_AGARRE), "AGARRE PLL", E_PLL_ABAJO,   1); break;
    case C_PS:     empezar(SEC_PS,     CANT(SEC_PS),     "PS",         E_PS_ARRIBA,  -1); break;
    case C_SOLTAR: empezar(SEC_SOLTAR, CANT(SEC_SOLTAR), "SOLTAR PS",  E_PS_ABAJO,    0); break;
  }
}

void irHome(bool home2) {
  if (estado == E_CALIBRANDO) { avisarOcupado(); return; }
  if (pasoActual >= 0) { out.print("Interrumpiendo "); out.println(nombreSec); }
  abandonarOrdenPC("se pidio HOME");
  pasoActual = -1;
  if (!calibrado[1]) {
    out.println("M2 sin calibrar: primero lo calibro contra el FC3 y despues voy a HOME.");
    homeDespuesDeCal = home2 ? 2 : 1;
    iniciarCalibracion();
    return;
  }
  if (home2) empezar(SEC_HOME2, CANT(SEC_HOME2), "HOME2", E_LISTO, -1);
  else       empezar(SEC_HOME,  CANT(SEC_HOME),  "HOME",  E_LISTO, -1);
}

void finalesDeCarrera() {
  for (int k = 0; k < 3; k++) {
    bool presionado = digitalRead(FC[k]) == LOW;
    int i = fcMotor[k];

    if (presionado && !fcAntes[k]) {         
      if (fcCero[k]) {                     
        pulsos[i] = 0;
        pos[i] = 0;
      }
      detener(i);
    }

    if (presionado && (meta[i] - pos[i]) * fcLado[k] > 0) detener(i);

    if (presionado != fcAntes[k]) hayCambio = true;
    fcAntes[k] = presionado;
  }
}

void reporte() {
  out.print("HALL:"); out.print(hallAntes ? "Y" : "N");
  out.print(" FC1:"); out.print(fcAntes[0] ? "Y" : "N");
  out.print(" FC2:"); out.print(fcAntes[1] ? "Y" : "N");
  out.print(" FC3:"); out.print(fcAntes[2] ? "Y" : "N");
  out.print(" | M1:"); out.print(pos[0], 1); out.print("cm");
  out.print(" M2:"); out.print(aReal(1, pos[1]), 1); out.print("°");
  if (!calibrado[1]) out.print("(sin cal)");
  out.print(" M3:"); out.print(pos[2], 1); out.print("°");
  out.print(" | SERVO:"); out.print(servoGrados); out.print("°");
  out.print(" | VEL:"); out.print(velocidad, 0); out.print("%");
  out.print(" | "); out.print(NOMBRE_ESTADO[estado]);
  out.println(tienePieza ? " +pieza" : "");
}

void enviarPosicion() {
  if (!hayCliente || millis() - tPos < PERIODO_POS) return;
  tPos = millis();
  char msg[64];
  snprintf(msg, sizeof(msg), "POS:%.2f,%.2f,%.2f\n", pos[0], aReal(1, pos[1]), pos[2]);
  udp.beginPacket(ipCliente, puertoCliente);
  udp.write((const uint8_t*)msg, strlen(msg));
  udp.endPacket();
}

void servo() {
  unsigned long t = micros() - tServo;
  if (servoAlto && t >= pulsoUS) {
    digitalWrite(SERVO, LOW);
    servoAlto = false;
  } else if (!servoAlto && t >= 20000) {
    tServo = micros();
    digitalWrite(SERVO, HIGH);
    servoAlto = true;
  }
}

void imprimirIP() {
  if (WiFi.status() == WL_CONNECTED) {
    out.print("Conectado a "); out.print(WIFI_SSID);
    out.print(" | IP: "); out.print(WiFi.localIP());
  } else {
    out.print("WiFi sin conectar (buscando "); out.print(WIFI_SSID); out.print(")");
  }
  out.print(" | UDP puerto "); out.println(PUERTO_UDP);
}

void iniciarWiFi() {
  WiFi.persistent(false);            
  WiFi.softAPdisconnect(true);    
  WiFi.mode(WIFI_STA);             
  WiFi.setSleep(false);             
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  udp.begin(PUERTO_UDP);
  udpPC.begin(PUERTO_COORD);          // puerto para el coordinador de la celda
  ipAvisada = false;
  out.print("Conectando a "); out.print(WIFI_SSID); out.println(" en segundo plano...");
}

void procesarComando(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  if (cmd.length() == 0) return;

  if (cmd == "STOP") {
    abandonarOrdenPC("STOP");
    ordenEnCola = "";
    homeDespuesDeCal = 0;
    pasoActual = -1;
    moviendo = false;
    for (int i = 0; i < 3; i++) detener(i);
    estado = E_DESCONOCIDO;
    hayCambio = true;
    out.println("Todo detenido. Usa HOME, HOME2 o CAL para seguir.");
  }
  else if (cmd == "P") {  
    out.print("P1:"); out.print(pulsos[0]);
    out.print(" P2:"); out.print(pulsos[1]);
    out.print(" P3:"); out.println(pulsos[2]);
  }
  else if (cmd == "ESTADO")                          imprimirEstado();
  else if (cmd == "IP")                              imprimirIP();
  else if (cmd == "CAL")                             iniciarCalibracion();
  else if (cmd == "HOME")                            irHome(false);
  else if (cmd == "HOME2" || cmd == "HOME 2")        irHome(true);
  else if (cmd == "PLL")                             pedir(C_PLL);
  else if (cmd == "AGARRE PLL" || cmd == "AGARRE")   pedir(C_AGARRE);
  else if (cmd == "PS")                              pedir(C_PS);
  else if (cmd == "SOLTAR PS" || cmd == "SOLTAR")    pedir(C_SOLTAR);
  else if (cmd.charAt(0) == 'V') {
    velocidad = constrain(cmd.substring(1).toFloat(), 1, 100);
    out.print("Velocidad -> "); out.print(velocidad, 0);
    out.println(" % (se aplica al siguiente movimiento)");
    hayCambio = true;
  }
  else if (cmd.charAt(0) == 'M') {
    int i = cmd.charAt(1) - '1';
    float valor = cmd.substring(2).toFloat();
    if (i < 0 || i > 2) { out.println("Usa M1, M2 o M3"); return; }
    if (ocupado()) { avisarOcupado(); return; }
    if (i == 0) valor = constrain(valor, M1_MIN, M1_MAX);
    estado = E_DESCONOCIDO;       
    hayCambio = true;
    nombreSec = "movimiento manual";
    meta[i] = aInterno(i, valor);
    out.print("Motor "); out.print(i + 1); out.print(" -> ");
    out.print(valor); out.println(i == 0 ? " cm" : " grados");
    iniciarTrayectoria();
  }
  else if (cmd.charAt(0) == 'S') {
    if (ocupado()) { avisarOcupado(); return; }
    ponerServo(cmd.substring(1).toInt());
  }
  else out.println("Comandos: PLL, AGARRE PLL, PS, SOLTAR PS, HOME, HOME2, CAL, STOP, ESTADO, IP, M1 8, M2 120, M3 90, S 90, V 70, P");
}

void leerSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (comando.length() > 0) procesarComando(comando);
      comando = "";
    } else comando += c;
  }
}

void leerUDP() {
  int len = udp.parsePacket();
  if (len <= 0) return;

  char buf[128];
  int n = udp.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = 0;

  ipCliente = udp.remoteIP();
  puertoCliente = udp.remotePort();
  hayCliente = true;
  hayCambio = true;

  String linea = "";
  for (int k = 0; k <= n; k++) {
    char c = buf[k];
    if (c == '\n' || c == '\r' || c == 0) {
      if (linea.length() > 0) procesarComando(linea);
      linea = "";
    } else linea += c;
  }
}

void enviarPC(const String& txt) {
  IPAddress destino = pcConocido ? ipPC
                    : (BUSCAR_PC ? IPAddress(255, 255, 255, 255) : IP_PC);
  udpPC.beginPacket(destino, PUERTO_PC);
  udpPC.print(txt);
  udpPC.endPacket();
}

void terminarOrdenPC() {
  if (ordenEnCurso.length() == 0) return;
  enviarPC(String(PREFIJO) + "LISTO " + ordenEnCurso);
  out.print("Coordinador: LISTO "); out.println(ordenEnCurso);
  ordenEnCurso = "";
}

float aRad(float g) { return g * PI / 180.0; }
float aGrados(float r) { return r * 180.0 / PI; }

// Punto conocido (0 HOME, 1 PLL, 2 PS) a partir de angulos de Gazebo, o -1
int puntoDesdeGazebo(float e1, float e2) {
  for (int p = 0; p < 3; p++)
    if (fabs(e1 - GZ_PUNTOS[p][0]) < 0.02 && fabs(e2 - GZ_PUNTOS[p][1]) < 0.02) return p;
  return -1;
}
// Punto conocido a partir de los angulos reales (grados), o -1
int puntoDesdeReal(float m2, float m3) {
  for (int p = 0; p < 3; p++)
    if (fabs(m3 - REAL_PUNTOS[p][0]) < 2.0 && fabs(m2 - REAL_PUNTOS[p][1]) < 2.0) return p;
  return -1;
}

float servoAGarra(int s) {
  float k = (GZ_GARRA_ABIERTA - GZ_GARRA_CERRADA) / (float)(SERVO_ABIERTO - SERVO_CERRADO);
  return GZ_GARRA_CERRADA + (s - SERVO_CERRADO) * k;
}
int garraAServo(float g) {
  float k = (float)(SERVO_ABIERTO - SERVO_CERRADO) / (GZ_GARRA_ABIERTA - GZ_GARRA_CERRADA);
  int s = SERVO_CERRADO + (int)((g - GZ_GARRA_CERRADA) * k);
  int lo = min(SERVO_ABIERTO, SERVO_CERRADO), hi = max(SERVO_ABIERTO, SERVO_CERRADO);
  return constrain(s, lo, hi);
}

void moverDesdeGazebo(float e1, float e2, float z, float g) {
  int p = puntoDesdeGazebo(e1, e2);
  float m3, m2;
  if (p >= 0) { m3 = REAL_PUNTOS[p][0]; m2 = REAL_PUNTOS[p][1]; }
  else {
    m3 = aGrados((e1 - GZ_OFFSET[0]) / GZ_ESCALA[0]);
    m2 = aGrados((e2 - GZ_OFFSET[1]) / GZ_ESCALA[1]);
  }
  float m1 = M1_ARRIBA - (z - Z_GZ_ARRIBA) / Z_ESCALA * 100.0;
  m1 = constrain(m1, M1_MIN, M1_MAX);
  int s = garraAServo(g);

  bool brazos = fabs(aReal(1, pos[1]) - m2) > 1.0 || fabs(pos[2] - m3) > 1.0;
  int n = 0;
  if (brazos) {
    secMueve[n++] = Paso{M1_ARRIBA, NADA, NADA, -1, 0};
    secMueve[n++] = Paso{NADA, m2, m3, -1, 0};
    secMueve[n++] = Paso{m1, NADA, NADA, s, 300};
  } else {
    secMueve[n++] = Paso{m1, m2, m3, s, 300};
  }

  int destino = E_DESCONOCIDO;
  bool arriba = m1 > M1_ARRIBA - 0.5;
  if (p == 0) destino = E_LISTO;
  if (p == 1) destino = arriba ? E_PLL_ARRIBA : E_PLL_ABAJO;
  if (p == 2) destino = arriba ? E_PS_ARRIBA : E_PS_ABAJO;
  empezar(secMueve, n, "MUEVE (coordinador)", destino, -1);
}

void ejecutarOrdenPC(const String& o) {
  if (!calibrado[1]) {         
    out.println("Coordinador: primero calibro el M2, la orden queda en espera");
    iniciarCalibracion();
    ordenEnCola = o;
    return;
  }
  if (o.startsWith("HAZ ")) {
    String orden = o.substring(4);                   
    String tramo = orden.substring(0, orden.indexOf(' '));
    ordenEnCurso = orden;
    out.print("Coordinador: HAZ "); out.println(orden);
    if (tramo == "ir")          empezar(SEC_IR,     CANT(SEC_IR),     "ir (PLL y agarrar)",  E_PLL_ARRIBA, 1);
    else if (tramo == "llevar") empezar(SEC_LLEVAR, CANT(SEC_LLEVAR), "llevar (PS y soltar)", E_PS_ARRIBA, 0);
    else if (tramo == "volver") empezar(SEC_VOLVER, CANT(SEC_VOLVER), "volver (HOME)",       E_LISTO,     -1);
    else {
      out.print("Coordinador: tramo desconocido '"); out.print(tramo); out.println("', se contesta LISTO");
      terminarOrdenPC();
    }
  }
  else if (o.startsWith("MUEVE ")) {
    String resto = o.substring(6);        
    float v[5];
    int k = 0, desde = 0, largo = resto.length();
    while (k < 5 && desde < largo) {
      int hasta = resto.indexOf(' ', desde);
      if (hasta < 0) hasta = largo;
      v[k++] = resto.substring(desde, hasta).toFloat();
      desde = hasta + 1;
    }
    ordenEnCurso = "MUEVE " + resto.substring(resto.lastIndexOf(' ') + 1);
    if (k < 5) {
      out.println("Coordinador: MUEVE con datos incompletos, se contesta LISTO sin mover");
      terminarOrdenPC();
      return;
    }
    out.print("Coordinador: "); out.println(o);
    moverDesdeGazebo(v[0], v[1], v[2], v[3]);
  }
}

void leerCoordinador() {
  int len = udpPC.parsePacket();
  if (len <= 0) return;
  char buf[160];
  int n = udpPC.read(buf, sizeof(buf) - 1);
  if (n <= 0) return;
  buf[n] = 0;
  String msg = String(buf);
  msg.trim();
  String yo = PREFIJO;
  if (!msg.startsWith(yo)) return;                 

  ipPC = udpPC.remoteIP();                         
  pcConocido = true;
  String resto = msg.substring(yo.length());
  resto.trim();

  if (resto == "BIENVENIDO") {
    if (!coordConectado) {
      out.print("Coordinador: conectado (PC "); out.print(ipPC); out.println(")");
    }
    coordConectado = true;
  }
  else if (resto.startsWith("HAZ ")) {
    String orden = resto.substring(4);               
    orden.trim();
    enviarPC(yo + "OK " + orden);                  
    if (orden == ultimaOrdenPC) return;              
    ultimaOrdenPC = orden;
    if (ocupado()) {
      ordenEnCola = "HAZ " + orden;
      out.print("Coordinador: HAZ "); out.print(orden); out.println(" en espera (robot ocupado)");
    } else ejecutarOrdenPC("HAZ " + orden);
  }
  else if (resto.startsWith("MUEVE ")) {
    String orden = resto.substring(6);               
    orden.trim();
    String num = orden.substring(orden.lastIndexOf(' ') + 1);
    enviarPC(yo + "OK MUEVE " + num);
    if (("MUEVE " + num) == ultimaOrdenPC) return;
    ultimaOrdenPC = "MUEVE " + num;
    if (ocupado()) {
      ordenEnCola = "MUEVE " + orden;
      out.println("Coordinador: MUEVE en espera (robot ocupado)");
    } else ejecutarOrdenPC("MUEVE " + orden);
  }
}

void saludarCoordinador() {
  unsigned long cada = coordConectado ? 5000 : 1000;
  if (millis() - tHola < cada) return;
  tHola = millis();
  enviarPC(String(PREFIJO) + "HOLA");
}

void enviarPosPC() {
  if (!coordConectado || millis() - tPosPC < 100) return;
  tPosPC = millis();
  float m2 = aReal(1, pos[1]), m3 = pos[2];
  float e1, e2;
  int p = puntoDesdeReal(m2, m3);
  if (p >= 0) { e1 = GZ_PUNTOS[p][0]; e2 = GZ_PUNTOS[p][1]; }
  else {
    e1 = GZ_ESCALA[0] * aRad(m3) + GZ_OFFSET[0];
    e2 = GZ_ESCALA[1] * aRad(m2) + GZ_OFFSET[1];
  }
  float z = Z_GZ_ARRIBA + (M1_ARRIBA - pos[0]) / 100.0 * Z_ESCALA;
  z = constrain(z, 0.001, 0.09);
  char msg[80];
  snprintf(msg, sizeof(msg), "%sPOS %.4f %.4f %.4f %.4f", PREFIJO, e1, e2, z, servoAGarra(servoGrados));
  enviarPC(msg);
}

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(IN1[i], OUTPUT);
    pinMode(IN2[i], OUTPUT);
    pinMode(EN[i], OUTPUT);
    moverMotor(i, 0);
  }

  pinMode(34, INPUT); pinMode(35, INPUT);
  pinMode(36, INPUT); pinMode(39, INPUT);
  pinMode(18, INPUT_PULLUP); pinMode(19, INPUT_PULLUP);

  for (int i = 0; i < 3; i++) {
    ultA[i] = digitalRead(ENC_A[i]);
    ultB[i] = digitalRead(ENC_B[i]);
  }
  attachInterrupt(ENC_A[0], encoder1, CHANGE); attachInterrupt(ENC_B[0], encoder1, CHANGE);
  attachInterrupt(ENC_A[1], encoder2, CHANGE); attachInterrupt(ENC_B[1], encoder2, CHANGE);
  attachInterrupt(ENC_A[2], encoder3, CHANGE); attachInterrupt(ENC_B[2], encoder3, CHANGE);

  for (int k = 0; k < 3; k++) {
    pinMode(FC[k], INPUT_PULLUP);
    fcAntes[k] = digitalRead(FC[k]) == LOW;
  }
  pinMode(HALL, INPUT);
  hallAntes = digitalRead(HALL);
  pinMode(SERVO, OUTPUT);

  out.println("Robot listo.");
  out.println("Ciclo: PLL -> AGARRE PLL -> PLL -> PS -> SOLTAR PS -> PS -> PLL ...");

  iniciarCalibracion();

  iniciarWiFi();
}

void loop() {
  servo();
  if (servoAlto) return; 

  leerSerial();
  leerUDP();
  if (WiFi.status() == WL_CONNECTED) {
    leerCoordinador();
    saludarCoordinador();
  } else coordConectado = false;

  // Avisa la IP cuando el hotspot conecte despues del arranque
  if (!ipAvisada && WiFi.status() == WL_CONNECTED) {
    ipAvisada = true;
    imprimirIP();
  }

  if (micros() - tControl < 10000) return;
  tControl = micros();

  for (int i = 0; i < 3; i++)
    pos[i] = pulsos[i] * GRADOS_POR_PULSO * factor[i];

  finalesDeCarrera();

  calibrar();

  actualizarTrayectoria();

  for (int i = 0; i < 3; i++) {
    float error = (ref[i] - pos[i]) / factor[i] * PI / 180.0;
    moverMotor(i, PID(i, error));
  }

  revisarSecuencia();

  bool hall = digitalRead(HALL);
  if (hall != hallAntes) {
    hallAntes = hall;
    hayCambio = true;
  }

  for (int i = 0; i < 3; i++)
    if (fabs(pos[i] - impreso[i]) >= cambio[i]) hayCambio = true;

  if (hayCambio) {
    for (int i = 0; i < 3; i++) impreso[i] = pos[i];
    reporte();
    hayCambio = false;
  }

  if (ordenEnCola.length() && !ocupado()) {
    String o = ordenEnCola;
    ordenEnCola = "";
    ejecutarOrdenPC(o);
  }

  enviarPosicion();
  enviarPosPC();
}