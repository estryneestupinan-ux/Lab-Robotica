# ESP32 MicroPython: puente de trayectoria ROS2, MODO SIN MOVIMIENTO FISICO.
# Compatible con ros2_esp32_puente_ack.py (REF,seq,base,codo,hombro,muneca).
# NO crea salidas PWM ni conecta drivers ni servos. Mantener potencia desconectada.
import sys, select, math
from time import ticks_ms, ticks_diff, sleep_ms

HOME_ROS = (0.0, 0.0, -180.0, 0.0)
# Rangos de protocolo para validar datos; NO representan limites mecanicos.
RANGO_PROTOCOLO = ((-360.0, 360.0), (-360.0, 360.0),
                   (-360.0, 360.0), (-360.0, 360.0))
TIMEOUT_MS = 300
home = False
seq_anterior = -1
ultimo_ref = None
referencia = HOME_ROS
recibidas = 0
rechazadas = 0
poll = select.poll()
poll.register(sys.stdin, select.POLLIN)

def enviar_estado():
    edad = -1 if ultimo_ref is None else ticks_diff(ticks_ms(), ultimo_ref)
    print('STATUS,modo=SIMULACION,home=%d,armado=0,seq=%d,recibidas=%d,rechazadas=%d,edad_ms=%d,base=%.3f,codo=%.3f,hombro=%.3f,muneca=%.3f' %
          (int(home), seq_anterior, recibidas, rechazadas, edad,
           referencia[0], referencia[1], referencia[2], referencia[3]))

def procesar(linea):
    global home, seq_anterior, ultimo_ref, referencia, recibidas, rechazadas
    if linea == 'PING':
        print('PONG'); return
    if linea == 'STATUS':
        enviar_estado(); return
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
                raise ValueError('cantidad')
            seq = int(partes[1])
            q = tuple(float(x) for x in partes[2:])
            if not all(math.isfinite(x) for x in q):
                raise ValueError('no_finito')
            if not all(lo <= x <= hi for x, (lo, hi) in zip(q, RANGO_PROTOCOLO)):
                raise ValueError('fuera_rango_protocolo')
        except ValueError:
            rechazadas += 1
            print('ERR,REF_FORMATO_O_RANGO'); return
        if not home:
            rechazadas += 1
            print('ERR,SIN_HOME'); return
        if seq <= seq_anterior:
            rechazadas += 1
            print('ERR,SEQ_ANTERIOR'); return
        referencia = q
        seq_anterior = seq
        ultimo_ref = ticks_ms()
        recibidas += 1
        print('ACK,%d' % seq)
        return
    print('ERR,COMANDO')

print('LISTO,ROS2_5_EJES,MODO_SIMULACION,SIN_ACTUADORES')
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
