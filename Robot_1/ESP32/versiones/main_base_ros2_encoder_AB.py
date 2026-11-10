# ESP32 MicroPython — base ROS 2, encoder cuadratura A/B
# VERSION DE PUESTA EN MARCHA: POTENCIA MOTOR BLOQUEADA POR SOFTWARE.
# No habilita PWM ni movimiento. No reemplaza un corte de potencia fisico.
import sys, select, math
from machine import Pin, PWM, disable_irq, enable_irq
from time import ticks_ms, ticks_diff, sleep_ms

# Base: pines del codigo verificado; B=33 es propuesta, confirmar cableado.
IN1_PIN, IN2_PIN, PWM_PIN = 25, 26, 27
ENC_A_PIN, ENC_B_PIN = 32, 33
TS_MS = 10
REF_TIMEOUT_MS = 300
LIM_MIN, LIM_MAX = -160.0, 40.0
HOME_REF = (0.0, 0.0, -180.0, 0.0)
HOME_TOL = 1.0
# Calibracion antigua era para un canal A. En cuadratura x4 se requiere
# nueva calibracion; el conteo crudo siempre es valido sin ella.
DEG_POR_CUENTA_AB = None

in1 = Pin(IN1_PIN, Pin.OUT, value=0)
in2 = Pin(IN2_PIN, Pin.OUT, value=0)
pwm = PWM(Pin(PWM_PIN), freq=1000, duty_u16=0)
a = Pin(ENC_A_PIN, Pin.IN, Pin.PULL_UP)
b = Pin(ENC_B_PIN, Pin.IN, Pin.PULL_UP)

cuentas = 0
transiciones_invalidas = 0
estado_ab = (a.value() << 1) | b.value()
# Indice: (estado previo << 2) | estado nuevo. 00->01->11->10->00 positivo.
TABLA = (0, 1, -1, 0,
         -1, 0, 0, 1,
         1, 0, 0, -1,
         0, -1, 1, 0)

def encoder_irq(pin):
    global cuentas, estado_ab, transiciones_invalidas
    nuevo = (a.value() << 1) | b.value()
    previo = estado_ab
    if nuevo == previo:
        return
    paso = TABLA[(previo << 2) | nuevo]
    if paso == 0:
        transiciones_invalidas += 1
    else:
        cuentas += paso
    estado_ab = nuevo

a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_irq)
b.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=encoder_irq)

def off():
    pwm.duty_u16(0)
    in1.value(0)
    in2.value(0)

def leer_cuentas():
    irq = disable_irq()
    valores = (cuentas, transiciones_invalidas, estado_ab)
    enable_irq(irq)
    return valores

home = False
seq = -1
ref = 0.0
ultimo_ref = None
rx = select.poll()
rx.register(sys.stdin, select.POLLIN)

def home_confirmar():
    global home, seq, ref, cuentas, transiciones_invalidas, ultimo_ref
    # SOLO con base fisicamente posicionada en HOME e inmovil.
    off()
    irq = disable_irq()
    cuentas = 0
    transiciones_invalidas = 0
    enable_irq(irq)
    home = True
    seq = -1
    ref = 0.0
    ultimo_ref = None
    print('HOME_CONFIRMADO,BASE=0,ARMADO=0')

def procesa(linea):
    global home, seq, ref, ultimo_ref
    if linea == 'PING':
        print('PONG')
        return
    if linea in ('STOP', 'HOME_INVALIDAR'):
        off()
        home = False
        ultimo_ref = None
        print('STOP,%s,BASE_OFF' % linea)
        return
    if linea == 'HOME_CONFIRM':
        home_confirmar()
        return
    if linea == 'STATUS':
        n, malas, ab = leer_cuentas()
        print('STATUS,home=%d,armado=0,base_cuentas=%d,ref=%.3f,seq=%d,invalidas=%d,ab=%d' %
              (int(home), n, ref, seq, malas, ab))
        return
    if linea in ('ARM_BASE', 'ARM_ALL'):
        print('ERR,POTENCIA_BLOQUEADA_PUESTA_EN_MARCHA')
        return
    if not linea.startswith('REF,'):
        print('ERR,COMANDO')
        return
    p = linea.split(',')
    if len(p) != 6:
        print('ERR,FORMATO')
        return
    try:
        nuevo_seq = int(p[1])
        q = [float(x) for x in p[2:]]
    except ValueError:
        print('ERR,FORMATO')
        return
    if not all(math.isfinite(x) for x in q):
        print('ERR,NO_FINITO')
        return
    if not home:
        print('ERR,SIN_HOME')
        return
    if nuevo_seq <= seq:
        print('ERR,SECUENCIA')
        return
    if not LIM_MIN <= q[0] <= LIM_MAX:
        print('ERR,LIM_BASE')
        return
    # Se reciben q2-q4 para compatibilidad con puente existente,
    # sin activar ningun otro eje.
    seq = nuevo_seq
    ref = q[0]
    ultimo_ref = ticks_ms()
    print('ACK,%d' % seq)

# No PWM en ningun caso, incluso si se manda ARM_BASE/ARM_ALL.
off()
print('LISTO,BASE_ROS2_ENCODER_AB,SIN_PWM,ARMADO=0')
print('ENCODER,GPIO_A=%d,GPIO_B=%d' % (ENC_A_PIN, ENC_B_PIN))
try:
    while True:
        while rx.poll(0):
            linea = sys.stdin.readline()
            if linea:
                procesa(linea.strip())
            else:
                break
        if ultimo_ref is not None and ticks_diff(ticks_ms(), ultimo_ref) > REF_TIMEOUT_MS:
            ultimo_ref = None
            # No invalidar HOME para permitir lectura de conteos; sin salidas siempre.
        off()
        sleep_ms(TS_MS)
except BaseException:
    off()
    raise
finally:
    off()
