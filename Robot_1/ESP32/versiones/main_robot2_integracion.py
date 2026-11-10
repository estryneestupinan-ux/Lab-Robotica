# ESP32 MicroPython - ROS 2 ROBOT2, integracion por etapas
# ETAPA DIAGNOSTICA: NO ACCIONA MOTORES NI SERVOS
# Puerto USB / REPL: protocolo ASCII HOME_CONFIRM, REF,seq,j1,j2,j3,j4,
# STATUS, HOME_INVALIDAR, STOP, PING
# ORDEN SERIAL: BASE, CODO, HOMBRO, MUNECA_ROS
# El codigo de esta etapa NO configura GPIO de drivers ni salidas PWM.
import sys
import select
import math
from machine import Pin, disable_irq, enable_irq
from time import ticks_ms, ticks_diff, sleep_ms

TS_MS = 10
TIMEOUT_MS = 300
HOME_ROS = (0.0, 0.0, -180.0, 0.0)
HOME_MG = 150.0
# Ventanas de OBSERVACION de trayectoria, no limites fisicos certificados.
VENTANAS = ((-160.0, 40.0), (-5.0, 100.0),
            (-190.0, -75.0), (-140.0, 30.0))

# Configuracion exacta de la imagen recibida.
# nombre, IN1, IN2, PWM, ENC_A, grados/pulso, KP, KI, KD,
# pwm_min, pwm_max, pwm_arranque, arranque_ms,
# ang_min, ang_max, max_cambio, tolerancia, ant_pos, ant_neg,
# alpha, integral_max, tiempo_max_ms, sin_pulsos_ms
CONFIG = (
    ('BASE', 25, 26, 27, 32, 360.0/7585.125, 3.0, 0.1, 0.15,
     90.0, 100.0, 100.0, 100,
     -160.0, 40.0, 200.0, 1.5, 1.0, 3.0,
     0.2, 30.0, 4000, 250),
    ('CODO', 19, 21, 18, 34, 0.0177, 6.0, 0.5, 0.15,
     0.0, 100.0, 100.0, 80,
     -100.0, 200.0, 150.0, 1.0, 0.5, 0.5,
     0.2, 20.0, 10000, 200),
    ('HOMBRO', 2, 4, 15, 23, 0.0165, 6.0, 0.5, 0.15,
     0.0, 100.0, 100.0, 80,
     -190.0, 100.0, 200.0, 1.0, 0.5, 0.5,
     0.2, 20.0, 10000, 200),
)

class EjeDiagnostico:
    def __init__(self, cfg, home):
        (self.nombre, self.in1, self.in2, self.pwm_pin, self.enc_pin,
         self.k, self.kp, self.ki, self.kd,
         self.pwm_min, self.pwm_max, self.pwm_arranque, self.arranque_ms,
         self.lim_min, self.lim_max, self.max_cambio, self.tol,
         self.ant_pos, self.ant_neg, self.alpha, self.integral_max,
         self.tiempo_max_ms, self.sin_pulsos_ms) = cfg
        self.home = home
        self.objetivo = home
        self.pulsos_a = 0
        self.integral = 0.0
        self.u_diagnostica = 0.0
        self.encoder = (Pin(self.enc_pin, Pin.IN) if self.enc_pin == 34
                        else Pin(self.enc_pin, Pin.IN, Pin.PULL_UP))
        self.encoder.irq(trigger=Pin.IRQ_RISING, handler=self._irq)

    def _irq(self, pin):
        # Solo canal A: conteo SIN signo de direccion verificable.
        self.pulsos_a += 1

    def contar(self):
        estado = disable_irq()
        n = self.pulsos_a
        enable_irq(estado)
        return n

    def fijar_home(self):
        estado = disable_irq()
        self.pulsos_a = 0
        enable_irq(estado)
        self.objetivo = self.home
        self.integral = 0.0
        self.u_diagnostica = 0.0

    def pid_sombra(self, dt):
        # NO se inventa la posicion real: se usa HOME como valor ficticio
        # para verificar el calculo, NO para controlar un motor.
        error = self.objetivo - self.home
        self.integral = max(-self.integral_max,
                            min(self.integral_max, self.integral + self.ki*error*dt))
        self.u_diagnostica = self.kp*error + self.integral


ejes = [EjeDiagnostico(cfg, home) for cfg, home in zip(CONFIG, HOME_ROS[:3])]
refs = list(HOME_ROS)
home_confirmado = False
modo = 'SIN_HOME'
ultimo_seq = -1
ultimo_rx = None
aviso_timeout = False
ultimo_tick = ticks_ms()

poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

def invalidar(motivo):
    global home_confirmado, modo, ultimo_rx, aviso_timeout
    home_confirmado = False
    modo = 'SIN_HOME'
    ultimo_rx = None
    aviso_timeout = False
    for eje in ejes:
        eje.objetivo = eje.home
        eje.integral = 0.0
    print('ALERTA,%s,ACTUADORES=OFF' % motivo)

def interpretar(linea):
    global home_confirmado, modo, ultimo_rx, ultimo_seq, aviso_timeout, refs
    if linea == 'PING':
        print('PONG')
        return
    if linea in ('STOP', 'HOME_INVALIDAR'):
        invalidar(linea)
        return
    if linea == 'HOME_CONFIRM':
        # Operador debe haber verificado postura HOME fisica e inmovilidad.
        for eje in ejes:
            eje.fijar_home()
        home_confirmado = True
        modo = 'HOME_CONFIRMADO'
        ultimo_seq = -1
        ultimo_rx = None
        aviso_timeout = False
        print('HOME_CONFIRMADO,BASE=0,CODO=0,HOMBRO=-180,MG=150,ACTUADORES=OFF')
        return
    if linea == 'STATUS':
        print('STATUS,modo=%s,home=%d,seq=%d,encA_base=%d,encA_codo=%d,encA_hombro=%d,ACTUADORES=OFF' %
              (modo, int(home_confirmado), ultimo_seq,
               ejes[0].contar(), ejes[1].contar(), ejes[2].contar()))
        return
    if not linea.startswith('REF,'):
        print('ERR,COMANDO')
        return
    partes = linea.split(',')
    if len(partes) != 6:
        print('ERR,NUM_CAMPOS')
        return
    try:
        seq = int(partes[1])
        q = [float(x) for x in partes[2:]]
    except ValueError:
        print('ERR,FORMATO')
        return
    if not home_confirmado:
        print('ERR,SIN_HOME')
        return
    if seq <= ultimo_seq:
        print('ERR,SECUENCIA')
        return
    if not all(math.isfinite(v) for v in q):
        print('ERR,NO_FINITO')
        return
    for nombre, valor, (lo, hi) in zip(('BASE','CODO','HOMBRO','MUNECA'), q, VENTANAS):
        if not lo <= valor <= hi:
            print('ERR,FUERA_DE_VENTANA,%s,%.3f' % (nombre, valor))
            return
    mg = HOME_MG + q[3]
    if not 0.0 <= mg <= 180.0:
        print('ERR,SERVO_FUERA_RANGO')
        return
    refs = q
    ultimo_seq = seq
    ultimo_rx = ticks_ms()
    aviso_timeout = False
    modo = 'OBSERVANDO'
    for i, eje in enumerate(ejes):
        eje.objetivo = q[i]
    if seq % 20 == 0:
        print('RX,%d,HOME=1,BASE=%.2f,CODO=%.2f,HOMBRO=%.2f,MG=%.2f,GRIPPER=NO_ASIGNADO,ACTUADORES=OFF' %
              (seq, q[0], q[1], q[2], mg))
    print('ACK,%d' % seq)

print('LISTO,ROBOT2_INTEGRACION_DIAGNOSTICO,ACTUADORES=OFF')
print('PINES,BASE=25/26/27/32,CODO=19/21/18/34,HOMBRO=2/4/15/23')
print('AVISO,ENCODER_A_NO_MIDE_SIGNO_REAL')
while True:
    # Limite por ciclo para evitar que la recepcion bloquee al control.
    procesados = 0
    while procesados < 8 and poll.poll(0):
        linea = sys.stdin.readline()
        if not linea:
            break
        interpretar(linea.strip())
        procesados += 1
    ahora = ticks_ms()
    dt_ms = ticks_diff(ahora, ultimo_tick)
    if dt_ms >= TS_MS:
        ultimo_tick = ahora
        dt = dt_ms / 1000.0
        for eje in ejes:
            eje.pid_sombra(dt)
        if home_confirmado and ultimo_rx is not None and ticks_diff(ahora, ultimo_rx) > TIMEOUT_MS:
            invalidar('REFERENCIA_CADUCADA')
    sleep_ms(1)
