# ESP32 MicroPython: receptor SOLO BASE, salidas deshabilitadas.
# Formato: BASE_REF,seq,angulo (grados). NO aplica PWM.
import sys, select, math
from machine import Pin, PWM
from time import ticks_ms, ticks_diff, sleep_ms

# La salida de la base se mantiene siempre en cero.
in1 = Pin(25, Pin.OUT, value=0)
in2 = Pin(26, Pin.OUT, value=0)
pwm = PWM(Pin(27), freq=1000, duty_u16=0)

def apagar():
    pwm.duty_u16(0)
    in1.value(0)
    in2.value(0)

lim_min, lim_max = -160.0, 40.0
ultimo_seq = -1
ultima_ref = 0.0
ultimo_ms = None
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

def procesar(linea):
    global ultimo_seq, ultima_ref, ultimo_ms
    if linea == 'PING':
        print('PONG'); return
    if linea in ('STOP', 'HOME_INVALIDAR'):
        apagar(); ultimo_seq = -1; ultima_ref = 0.0; ultimo_ms = None
        print('STOP,BASE_OFF'); return
    if linea == 'STATUS':
        edad = -1 if ultimo_ms is None else ticks_diff(ticks_ms(), ultimo_ms)
        print('STATUS,armado=0,base_pwm=0,ref=%.3f,seq=%d,edad_ms=%d' %
              (ultima_ref, ultimo_seq, edad)); return
    if not linea.startswith('BASE_REF,'):
        print('ERR,COMANDO'); return
    p = linea.split(',')
    if len(p) != 3:
        print('ERR,FORMATO'); return
    try:
        seq = int(p[1]); ang = float(p[2])
    except ValueError:
        print('ERR,FORMATO'); return
    if not math.isfinite(ang) or seq <= ultimo_seq:
        print('ERR,SECUENCIA_O_VALOR'); return
    if not lim_min <= ang <= lim_max:
        print('ERR,LIMITE_BASE'); return
    ultimo_seq = seq
    ultima_ref = ang
    ultimo_ms = ticks_ms()
    print('ACK,%d' % seq)

apagar()
print('LISTO,SOLO_BASE,RECEPCION_SIN_PWM')
try:
    while True:
        if poll.poll(0):
            s = sys.stdin.readline()
            if s:
                procesar(s.strip())
        # Apagado redundante incluso si se recibe referencia valida.
        apagar()
        sleep_ms(5)
finally:
    apagar()
