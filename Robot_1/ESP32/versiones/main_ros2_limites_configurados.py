# MicroPython ESP32 - referencias ROS 2 con CONFIG del usuario.
# NO ACCIONA MOTORES NI SERVOS. Es el paso de validacion de trayectoria.
# Mantener alimentacion de potencia desconectada.
import sys, select, math
from time import ticks_ms, ticks_diff, sleep_ms

# Nombre, IN1, IN2, PWM, ENC_A, grados/pulso, Kp, Ki, Kd, min, max
# Parametros tomados del ultimo control de 5 actuadores suministrado.
CONFIG = (
    ('BASE', 25, 26, 27, 32, 360.0/7585.125, 3.0, 0.1, 0.15, -160.0, 40.0),
    ('HOMBRO', 2, 4, 15, 23, 0.0165, 6.0, 0.5, 0.15, -190.0, 100.0),
    ('CODO', 19, 21, 18, 34, 0.0177, 6.0, 0.5, 0.15, -100.0, 200.0),
)
MG996R_GPIO = 12
SG90_GPIO = 13
MG996R_PULSOS_US = (1000, 2000)
SG90_PULSOS_US = (500, 2500)
HOME_ROS = (0.0, 0.0, -180.0, 0.0)
TIMEOUT_MS = 300

home = False
seq_anterior = -1
ultimo_ref = None
referencia = HOME_ROS
recibidas = 0
rechazadas = 0
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

def rangos(q):
    # REF: base, codo, hombro ROS, muneca ROS.
    # Hombro: -180 ROS equivale a 0 local.
    base, codo, hombro_ros, muneca_ros = q
    hombro_local = hombro_ros + 180.0
    # Servo MG996R: consigna fisica nominal 150 + joint_4.
    muneca_servo = 150.0 + muneca_ros
    return (CONFIG[0][-2] <= base <= CONFIG[0][-1] and
            CONFIG[2][-2] <= codo <= CONFIG[2][-1] and
            CONFIG[1][-2] <= hombro_local <= CONFIG[1][-1] and
            0.0 <= muneca_servo <= 180.0)

def estado():
    edad = -1 if ultimo_ref is None else ticks_diff(ticks_ms(), ultimo_ref)
    print('STATUS,modo=SIMULACION_CONFIG,home=%d,armado=0,seq=%d,recibidas=%d,rechazadas=%d,edad_ms=%d,base=%.3f,codo=%.3f,hombro=%.3f,muneca=%.3f' %
          (int(home), seq_anterior, recibidas, rechazadas, edad,
           referencia[0], referencia[1], referencia[2], referencia[3]))

def procesar(linea):
    global home, seq_anterior, ultimo_ref, referencia, recibidas, rechazadas
    if linea == 'PING':
        print('PONG'); return
    if linea == 'STATUS':
        estado(); return
    if linea in ('STOP', 'HOME_INVALIDAR'):
        home = False
        seq_anterior = -1
        ultimo_ref = None
        referencia = HOME_ROS
        print('STOP,SIMULACION,SIN_ACTUADORES'); return
    if linea == 'HOME_CONFIRM':
        home = True
        seq_anterior = -1
        ultimo_ref = None
        referencia = HOME_ROS
        print('HOME_CONFIRMADO,SIMULACION,ARMADO=0'); return
    if linea in ('ARM_BASE', 'ARM_ALL'):
        print('ERR,MODO_SIMULACION_NO_ARMABLE'); return
    if linea.startswith('REF,'):
        partes = linea.split(',')
        try:
            if len(partes) != 6:
                raise ValueError('formato')
            seq = int(partes[1])
            q = tuple(float(x) for x in partes[2:])
            if not all(math.isfinite(x) for x in q):
                raise ValueError('no_finito')
        except ValueError:
            rechazadas += 1
            print('ERR,REF_FORMATO'); return
        if not home:
            rechazadas += 1
            print('ERR,SIN_HOME'); return
        if seq <= seq_anterior:
            rechazadas += 1
            print('ERR,SEQ_ANTERIOR'); return
        if not rangos(q):
            rechazadas += 1
            print('ERR,REF_FUERA_DE_LIMITES'); return
        referencia = q
        seq_anterior = seq
        ultimo_ref = ticks_ms()
        recibidas += 1
        print('ACK,%d' % seq)
        return
    print('ERR,COMANDO')

print('LISTO,ROS2_5_EJES,LIMITES_CONFIGURADOS,SIN_ACTUADORES')
try:
    while True:
        while poll.poll(0):
            linea = sys.stdin.readline()
            if not linea:
                break
            procesar(linea.strip())
        sleep_ms(1)
except BaseException as exc:
    print('ERROR_FATAL,SIN_ACTUADORES', repr(exc))
    raise
