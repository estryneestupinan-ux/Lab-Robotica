# MicroPython ESP32 - Integracion ROS2, primera etapa BASE
# Arranque SIEMPRE desarmado. Prueba restringida a HOME +/- 2 grados.
# Codo/hombro/servos no se configuran ni se accionan.
import sys, select, math
from machine import Pin, PWM, disable_irq, enable_irq
from time import ticks_ms, ticks_diff, sleep_ms

# GPIO confirmados por usuario: BASE IN1=25 IN2=26 PWM=27 ENC_A=32
IN1, IN2, PWM_PIN, ENCODER_PIN = 25, 26, 27, 32
K = 360.0 / 7585.125
KP, KI, KD = 3.0, 0.1, 0.15
TS_MS = 10
TIMEOUT_MS = 250
MAX_ANG = 3.0              # barrera relativa a HOME
MAX_REF = 2.0              # orden admisible en esta prueba
MAX_MOVIMIENTO_MS = 700    # corta incluso si no llega a la referencia
SIN_PULSOS_MS = 200
# Conserva los valores PID originales, pero la prueba usa techo de PWM inferior.
# Si el motor no arranca, NO elevar sin revisar la respuesta y el montaje.
PWM_MAX_PCT = 30.0

in1 = Pin(IN1, Pin.OUT, value=0)
in2 = Pin(IN2, Pin.OUT, value=0)
pwm = PWM(Pin(PWM_PIN), freq=1000, duty_u16=0)
enc = Pin(ENCODER_PIN, Pin.IN, Pin.PULL_UP)
cuentas = 0
signo = 0
home = False
armado = False
ref = 0.0
seq_anterior = -1
ultimo_ref_ms = None
t_inicio_mov = None
t_ultimo_pulso = None
cuentas_anteriores = 0
integral = 0.0
vel_filtrada = 0.0
ultimo_control = ticks_ms()
rx = select.poll()
rx.register(sys.stdin, select.POLLIN)

def irq_encoder(pin):
    global cuentas
    if signo:
        cuentas += signo
enc.irq(trigger=Pin.IRQ_RISING, handler=irq_encoder)

def contar():
    s = disable_irq()
    x = cuentas
    enable_irq(s)
    return x

def angulo():
    return contar() * K

def cortar():
    global signo
    pwm.duty_u16(0)
    in1.value(0)
    in2.value(0)
    s = disable_irq()
    signo = 0
    enable_irq(s)

def desarmar(razon):
    global armado, home, ref, integral, vel_filtrada
    cortar()
    armado = False
    # Los pulsos de canal A no permiten reconstruir desplazamientos externos.
    # Tras STOP/fallo se vuelve a exigir confirmacion fisica de HOME.
    home = False
    ref = 0.0
    integral = 0.0
    vel_filtrada = 0.0
    print('STOP,%s,BASE_OFF' % razon)

def establecer_home():
    global home, armado, cuentas, signo, ref, integral, vel_filtrada, seq_anterior
    cortar()
    s = disable_irq()
    cuentas = 0
    signo = 0
    enable_irq(s)
    home = True
    armado = False
    ref = 0.0
    integral = 0.0
    vel_filtrada = 0.0
    seq_anterior = -1
    print('HOME_CONFIRMADO,BASE=0,ARMADO=0')

def leer(linea):
    global armado, ref, seq_anterior, ultimo_ref_ms, t_inicio_mov, t_ultimo_pulso
    global cuentas_anteriores, integral, vel_filtrada
    if linea == 'PING':
        print('PONG')
        return
    if linea in ('STOP','HOME_INVALIDAR'):
        desarmar(linea)
        return
    if linea == 'HOME_CONFIRM':
        establecer_home()
        return
    if linea == 'STATUS':
        print('STATUS,home=%d,armado=%d,base=%.3f,ref=%.3f,seq=%d' %
              (int(home), int(armado), angulo(), ref, seq_anterior))
        return
    if linea == 'ARM_BASE':
        if not home or abs(angulo()) > 0.5 or abs(ref) > 0.5 or ultimo_ref_ms is None or ticks_diff(ticks_ms(), ultimo_ref_ms) > TIMEOUT_MS:
            print('ERR,NO_SINCRONIZADO')
            return
        armado = True
        integral = 0.0
        vel_filtrada = 0.0
        cuentas_anteriores = contar()
        t_inicio_mov = None
        t_ultimo_pulso = ticks_ms()
        print('ARMADO,BASE_SOLO')
        return
    if not linea.startswith('REF,'):
        print('ERR,COMANDO')
        return
    p = linea.split(',')
    if len(p) != 6:
        print('ERR,FORMATO')
        return
    try:
        seq = int(p[1]); q = [float(x) for x in p[2:]]
    except ValueError:
        print('ERR,FORMATO')
        return
    if not all(math.isfinite(x) for x in q) or seq <= seq_anterior:
        print('ERR,REF_INVALIDA')
        return
    if not home:
        print('ERR,SIN_HOME')
        return
    # Impide perseguir punto A: exigir resto de juntas en HOME y base muy cercana.
    if abs(q[0]) > MAX_REF or abs(q[1]) > 0.5 or abs(q[2]+180) > 0.5 or abs(q[3]) > 0.5:
        if armado:
            desarmar('REF_FUERA_DE_VENTANA')
        else:
            print('ERR,REF_FUERA_DE_VENTANA')
        return
    seq_anterior = seq
    ultimo_ref_ms = ticks_ms()
    ref = q[0]
    print('ACK,%d' % seq)

def control(ahora, dt):
    global t_inicio_mov, t_ultimo_pulso, cuentas_anteriores, integral, vel_filtrada, signo
    if not armado:
        cortar()
        return
    if ultimo_ref_ms is None or ticks_diff(ahora, ultimo_ref_ms) > TIMEOUT_MS:
        desarmar('TIMEOUT_COMUNICACION'); return
    theta = angulo()
    if abs(theta) > MAX_ANG:
        desarmar('LIMITE_DESPLAZAMIENTO'); return
    error = ref - theta
    if abs(error) <= 0.6:
        cortar()
        t_inicio_mov = None
        integral = 0.0
        return
    direccion = 1 if error > 0 else -1
    if t_inicio_mov is None:
        t_inicio_mov = ahora
        t_ultimo_pulso = ahora
        cuentas_anteriores = contar()
    if ticks_diff(ahora, t_inicio_mov) > MAX_MOVIMIENTO_MS:
        desarmar('TIEMPO_MAXIMO_MOVIMIENTO'); return
    n = contar()
    if n != cuentas_anteriores:
        t_ultimo_pulso = ahora
    elif ticks_diff(ahora, t_ultimo_pulso) > SIN_PULSOS_MS:
        desarmar('ENCODER_SIN_PULSOS'); return
    velocidad = (n - cuentas_anteriores) * K / dt
    vel_filtrada = 0.2 * velocidad + 0.8 * vel_filtrada
    integral = max(-30.0, min(30.0, integral + KI*error*dt))
    u = KP*error + integral - KD*vel_filtrada
    # Nunca invierte automaticamente el motor ante cambio abrupto de referencia.
    if u*direccion <= 0:
        cortar()
        return
    if direccion > 0:
        in1.value(1); in2.value(0)
    else:
        in1.value(0); in2.value(1)
    s = disable_irq()
    signo = direccion
    enable_irq(s)
    duty = min(PWM_MAX_PCT, abs(u))
    pwm.duty_u16(int(duty*65535/100))
    cuentas_anteriores = n

cortar()
print('LISTO,BASE_ROS2_PRUEBA_RESTRINGIDA,ARMADO=0')
try:
    while True:
        while rx.poll(0):
            linea = sys.stdin.readline()
            if not linea:
                break
            leer(linea.strip())
        ahora = ticks_ms()
        if ticks_diff(ahora, ultimo_control) >= TS_MS:
            dt = max(0.001, ticks_diff(ahora, ultimo_control)/1000.0)
            ultimo_control = ahora
            control(ahora, dt)
        sleep_ms(1)
except BaseException as exc:
    cortar()
    print('ERROR_FATAL,BASE_OFF', repr(exc))
    raise
finally:
    cortar()
