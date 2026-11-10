# ESP32 MicroPython - ROS 2 -> 3 PID DC + MG996R + SG90
# Arranque DESARMADO. No energizar el robot antes de contar con corte fisico de potencia.
# REF,seq,base,codo,hombro,muneca: angulos de ROS2 en grados.
# HOME ROS: 0,0,-180,0. SG90: comando independiente GRIP,angulo.
import sys, select, math
from machine import Pin, PWM, disable_irq, enable_irq
from time import ticks_us, ticks_ms, ticks_diff, sleep_ms

TS_US = 10000
TIMEOUT_MS = 250
# Referencias inicialmente limitadas a HOME +/- 2 grados en las 4 juntas.
# Esta ventana NO cubre la trayectoria A-B. No ampliarla sin verificar limites reales.
VENTANA_REF = (2.0, 2.0, 2.0, 2.0)
HOME_ROS = (0.0, 0.0, -180.0, 0.0)
CONFIG = (
    ('BASE', 25, 26, 27, 32, 360.0/7585.125, 3.0, 0.1, 0.15,
     90.0, 100.0, 100.0, 100, -160.0, 40.0, 200.0, 1.5, 1.0, 3.0,
     0.2, 30.0, 4000, 250),
    ('HOMBRO', 2, 4, 15, 23, 0.0165, 6.0, 0.5, 0.15,
     0.0, 100.0, 100.0, 80, -190.0, 100.0, 200.0, 1.0, 0.5, 0.5,
     0.2, 20.0, 10000, 200),
    ('CODO', 19, 21, 18, 34, 0.0177, 6.0, 0.5, 0.15,
     0.0, 100.0, 100.0, 80, -100.0, 200.0, 150.0, 1.0, 0.5, 0.5,
     0.2, 20.0, 10000, 200),
)

def limitar(x, a, b):
    return max(a, min(b, x))

class Eje:
    def __init__(self, cfg):
        (self.nombre, p1, p2, ppwm, penc, self.k, self.kp, self.ki, self.kd,
         self.pwm_min, self.pwm_max, self.pwm_inicio, self.arranque_ms,
         self.lim_min, self.lim_max, self.max_cambio, self.tol,
         self.ant_pos, self.ant_neg, self.alpha, self.int_max,
         self.tiempo_max_ms, self.sin_pulsos_ms) = cfg
        self.in1 = Pin(p1, Pin.OUT, value=0)
        self.in2 = Pin(p2, Pin.OUT, value=0)
        self.pwm = PWM(Pin(ppwm), freq=1000, duty_u16=0)
        self.enc = Pin(penc, Pin.IN) if penc == 34 else Pin(penc, Pin.IN, Pin.PULL_UP)
        self.n = 0
        self.signo = 0
        self.enc.irq(trigger=Pin.IRQ_RISING, handler=self._irq)
        self.ref = 0.0
        self.moviendo = False
        self.integral = 0.0
        self.vel_f = 0.0
        self.n_prev = 0
        self.ultimo_pulso = ticks_ms()
        self.inicio = ticks_ms()
        self.inicial = 0.0
        self.sentido = 0

    def _irq(self, pin):
        if self.signo:
            self.n += self.signo

    def cuentas(self):
        estado = disable_irq()
        n = self.n
        enable_irq(estado)
        return n

    def angulo(self):
        return self.cuentas() * self.k

    def signo_irq(self, valor):
        estado = disable_irq()
        self.signo = valor
        enable_irq(estado)

    def apagar(self):
        self.pwm.duty_u16(0)
        self.in1.value(0)
        self.in2.value(0)
        self.signo_irq(0)
        self.moviendo = False

    def cero(self):
        self.apagar()
        estado = disable_irq()
        self.n = 0
        enable_irq(estado)
        self.integral = 0.0
        self.vel_f = 0.0
        self.ref = 0.0

    def paso(self, ahora, dt):
        theta = self.angulo()
        err = self.ref - theta
        if not (self.lim_min <= theta <= self.lim_max):
            raise ValueError(self.nombre + '_LIMITE')
        if abs(err) > self.max_cambio:
            raise ValueError(self.nombre + '_CAMBIO')
        if abs(err) <= self.tol:
            self.apagar()
            self.integral = 0.0
            return
        sentido = 1 if err > 0 else -1
        ant = self.ant_pos if sentido > 0 else self.ant_neg
        if abs(err) <= ant:
            self.apagar()
            return
        if not self.moviendo:
            self.sentido = sentido
            self.inicio = ahora
            self.inicial = theta
            self.ultimo_pulso = ahora
            self.n_prev = self.cuentas()
            self.integral = 0.0
            self.vel_f = 0.0
            self.moviendo = True
            self.signo_irq(sentido)
        if sentido != self.sentido:
            # No invertir mientras gira; cortar primero. El encoder A no da direccion real.
            self.apagar()
            return
        if ticks_diff(ahora, self.inicio) > self.tiempo_max_ms:
            raise ValueError(self.nombre + '_TIMEOUT_MOV')
        if abs(theta - self.inicial) > self.max_cambio:
            raise ValueError(self.nombre + '_MAX_CAMBIO')
        n = self.cuentas()
        if n != self.n_prev:
            self.ultimo_pulso = ahora
        elif ticks_diff(ahora, self.ultimo_pulso) > self.sin_pulsos_ms:
            raise ValueError(self.nombre + '_SIN_PULSOS')
        vel = (n-self.n_prev)*self.k/dt
        self.vel_f = self.alpha*vel + (1-self.alpha)*self.vel_f
        candidato = limitar(self.integral + self.ki*err*dt,
                            -self.int_max, self.int_max)
        u_c = sentido*(self.kp*err+candidato-self.kd*self.vel_f)
        if 0 < u_c < self.pwm_max:
            self.integral = candidato
        u = sentido*(self.kp*err + self.integral - self.kd*self.vel_f)
        if u <= 0:
            self.apagar()
            return
        duty = limitar(u, self.pwm_min, self.pwm_max)
        if ticks_diff(ahora, self.inicio) < self.arranque_ms:
            duty = self.pwm_inicio
        self.in1.value(1 if sentido > 0 else 0)
        self.in2.value(0 if sentido > 0 else 1)
        self.pwm.duty_u16(int(limitar(duty, 0, 100)*65535/100))
        self.n_prev = n

base, hombro, codo = [Eje(c) for c in CONFIG]
ejes = (base, hombro, codo)
servo_mg = PWM(Pin(12), freq=50, duty_u16=0)
servo_sg = PWM(Pin(13), freq=50, duty_u16=0)
rx = select.poll()
rx.register(sys.stdin, select.POLLIN)
home = False
armado = False
seq_anterior = -1
ultimo_ref = None
ref_ros = HOME_ROS

# Conversor SERVO: MG996R nominal 1000-2000 us, SG90 500-2500 us.
def duty_servo(angulo, pulso_min, pulso_max):
    if not (0 <= angulo <= 180):
        raise ValueError('SERVO_FUERA_DE_RANGO')
    return int((pulso_min+(pulso_max-pulso_min)*angulo/180)*65535/20000)

def cortar_todo():
    for eje in ejes:
        eje.apagar()
    # Deshabilitar PWM de servos. Puede liberar torque: prever soporte del brazo.
    servo_mg.duty_u16(0)
    servo_sg.duty_u16(0)

def detener(razon):
    global home, armado, ultimo_ref, ref_ros
    cortar_todo()
    armado = False
    home = False
    ultimo_ref = None
    ref_ros = HOME_ROS
    print('STOP,%s,ALL_OFF' % razon)

def procesar(linea):
    global home, armado, seq_anterior, ultimo_ref, ref_ros
    if linea == 'PING':
        print('PONG'); return
    if linea in ('STOP', 'HOME_INVALIDAR'):
        detener(linea); return
    if linea == 'STATUS':
        print('STATUS,home=%d,armado=%d,base=%.3f,codo=%.3f,hombro=%.3f,ref=%.3f,seq=%d' %
              (int(home), int(armado), base.angulo(), codo.angulo(),
               -180+hombro.angulo(), ref_ros[0], seq_anterior))
        return
    if linea == 'HOME_CONFIRM':
        if armado:
            print('ERR,ARMADO'); return
        # Exige verificar HOME fisicamente ANTES de este comando.
        for eje in ejes:
            eje.cero()
        home = True
        seq_anterior = -1
        ultimo_ref = None
        ref_ros = HOME_ROS
        print('HOME_CONFIRMADO,BASE=0,ARMADO=0'); return
    if linea == 'ARM_ALL':
        if (not home or ultimo_ref is None or
            ticks_diff(ticks_ms(), ultimo_ref) > TIMEOUT_MS or
            any(abs(ref_ros[i]-HOME_ROS[i]) > .5 for i in range(4)) or
            any(abs(eje.angulo()) > .5 for eje in ejes)):
            print('ERR,NO_SINCRONIZADO'); return
        armado = True
        print('ARMADO,TRES_PID_DOS_SERVOS'); return
    if linea.startswith('GRIP,'):
        if not armado:
            print('ERR,DESARMADO'); return
        try:
            a = float(linea.split(',')[1])
            if not math.isfinite(a): raise ValueError()
            d = duty_servo(a, 500, 2500)
        except (ValueError, IndexError):
            print('ERR,GRIP'); return
        servo_sg.duty_u16(d)
        print('GRIP_OK'); return
    if not linea.startswith('REF,'):
        print('ERR,COMANDO'); return
    p = linea.split(',')
    if len(p) != 6:
        print('ERR,FORMATO'); return
    try:
        seq = int(p[1])
        q = tuple(float(x) for x in p[2:])
        if not all(math.isfinite(x) for x in q): raise ValueError()
    except ValueError:
        print('ERR,FORMATO'); return
    if not home:
        print('ERR,SIN_HOME'); return
    if seq <= seq_anterior:
        print('ERR,REF_INVALIDA'); return
    if any(abs(q[i]-HOME_ROS[i]) > VENTANA_REF[i] for i in range(4)):
        if armado: detener('REF_FUERA_DE_VENTANA')
        else: print('ERR,REF_FUERA_DE_VENTANA')
        return
    # ROS hombro -180 corresponde a cero local del encoder del hombro.
    objetivos = (q[0], q[2]+180, q[1])
    if any(not eje.lim_min <= x <= eje.lim_max for eje, x in zip(ejes, objetivos)):
        if armado: detener('LIMITE_REFERENCIA')
        else: print('ERR,LIMITE_REFERENCIA')
        return
    try:
        d_mg = duty_servo(150+q[3], 1000, 2000)
    except ValueError:
        print('ERR,MUNECA'); return
    seq_anterior = seq
    ultimo_ref = ticks_ms()
    ref_ros = q
    if armado:
        for eje, objetivo in zip(ejes, objetivos):
            eje.ref = objetivo
        servo_mg.duty_u16(d_mg)
    print('ACK,%d' % seq)

cortar_todo()
print('LISTO,ROS2_5_EJES,ARMADO=0,VENTANA=2_GRADOS')
ultimo_ciclo = ticks_us()
try:
    while True:
        while rx.poll(0):
            linea = sys.stdin.readline()
            if not linea:
                break
            procesar(linea.strip())
        ahora_ms = ticks_ms()
        if armado and (ultimo_ref is None or
                       ticks_diff(ahora_ms, ultimo_ref) > TIMEOUT_MS):
            detener('TIMEOUT_COMUNICACION')
        ahora_us = ticks_us()
        if ticks_diff(ahora_us, ultimo_ciclo) >= TS_US:
            dt = max(0.001, ticks_diff(ahora_us, ultimo_ciclo)/1000000.0)
            ultimo_ciclo = ahora_us
            if armado:
                try:
                    for eje in ejes:
                        eje.paso(ahora_ms, dt)
                except ValueError as exc:
                    detener(str(exc))
        sleep_ms(1)
except BaseException as exc:
    detener('ERROR_FATAL')
    print('ERROR_FATAL', repr(exc))
    raise
finally:
    cortar_todo()
