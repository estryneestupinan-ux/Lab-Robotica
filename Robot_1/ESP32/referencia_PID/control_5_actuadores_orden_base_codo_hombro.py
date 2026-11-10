# MicroPython ESP32 — 3 PID de posicion + MG996R + SG90
# Inicio coordinado, final independiente. Encoder solo canal A.
from machine import Pin, PWM, disable_irq, enable_irq
from time import ticks_us, ticks_diff, sleep_ms

TS_US = 10000
MAX_CONTROL_TOTAL_MS = 11500
ASENTAMIENTO_MS = 500

# IMPORTANTE: son los valores de los programas individuales entregados.
# No son limites mecanicos/electricos garantizados.
CONFIG = (
    # nombre, IN1, IN2, PWM, ENC_A, grados/pulso, KP, KI, KD,
    # pwm_min, pwm_max, pwm_arranque, arranque_ms,
    # ang_min, ang_max, max_cambio, tolerancia, ant_pos, ant_neg,
    # alpha, integral_max, tiempo_max_ms, sin_pulsos_ms
    ('BASE', 26, 25, 27, 32, 360.0/7585.125, 3.0, 0.1, 0.15,
     90.0, 100.0, 100.0, 100,
     -40.0, 355.0, 200.0, 1.5, 1.0, 3.0,
     0.2, 30.0, 4000, 250),
    ('HOMBRO', 2, 4, 15, 23, 0.0165, 6.0, 0.5, 0.15,
     0.0, 100.0, 100.0, 80,
     -100.0, 200.0, 150.0, 1.0, 0.5, 0.5,
     0.2, 20.0, 10000, 200),
    ('CODO', 21, 19, 18, 34, 0.0177, 6.0, 0.5, 0.15,
     0.0, 100.0, 100.0, 80,
     -100.0, 200.0, 150.0, 1.0, 0.5, 0.5,
     0.2, 20.0, 10000, 200),
)

def limitar(x, lo, hi):
    return max(lo, min(hi, x))

class Eje:
    def __init__(self, cfg):
        (self.nombre, p1, p2, ppwm, penc, self.k,
         self.kp, self.ki, self.kd,
         self.pwm_min, self.pwm_max, self.pwm_inicio, self.arranque_ms,
         self.lim_min, self.lim_max, self.max_cambio, self.tol,
         self.ant_pos, self.ant_neg, self.alpha, self.int_max,
         self.timeout_ms, self.sin_pulsos_ms) = cfg
        self.in1 = Pin(p1, Pin.OUT, value=0)
        self.in2 = Pin(p2, Pin.OUT, value=0)
        self.pwm = PWM(Pin(ppwm), freq=1000, duty_u16=0)
        self.enc = Pin(penc, Pin.IN) if penc == 34 else Pin(penc, Pin.IN, Pin.PULL_UP)
        self.n = 0
        self.signo = 0
        self.estado = 'LISTO'
        self.motivo = ''
        self.referencia = 0.0
        self.sentido = 0
        self.enc.irq(trigger=Pin.IRQ_RISING, handler=self._irq)

    def _irq(self, pin):
        if self.signo:
            self.n += self.signo

    def lectura(self):
        s = disable_irq()
        n = self.n
        enable_irq(s)
        return n

    def angulo(self):
        return self.lectura() * self.k

    def _signo(self, valor):
        s = disable_irq()
        self.signo = valor
        enable_irq(s)

    def apagar(self):
        self.pwm.duty_u16(0)
        self.in1.value(0)
        self.in2.value(0)

    def cero(self):
        self.apagar()
        s = disable_irq()
        self.signo = 0
        self.n = 0
        enable_irq(s)
        self.estado = 'LISTO'
        self.motivo = ''

    def preparar(self, objetivo, ahora):
        actual = self.angulo()
        error = objetivo - actual
        if not (self.lim_min <= objetivo <= self.lim_max):
            raise ValueError(self.nombre + ': objetivo fuera de limites configurados')
        if not (self.lim_min <= actual <= self.lim_max):
            raise ValueError(self.nombre + ': posicion estimada fuera de limites')
        if abs(error) > self.max_cambio:
            raise ValueError(self.nombre + ': cambio supera MAX_CAMBIO')
        if not 0 <= self.pwm_min <= self.pwm_max <= 100:
            raise ValueError(self.nombre + ': PWM invalido')
        self.referencia = objetivo
        self.inicial = actual
        self.sentido = 1 if error > 0 else -1
        self.ant = self.ant_pos if self.sentido == 1 else self.ant_neg
        self.i = 0.0
        self.omega_f = 0.0
        self.n_anterior = self.lectura()
        self.n_ultimo = self.n_anterior
        self.t_inicio = ahora
        self.t_anterior = ahora
        self.t_ultimo_pulso = ahora
        self.t_corte = ahora
        self.motivo = ''
        self._signo(0)
        self.apagar()
        if abs(error) <= self.tol:
            self.estado = 'HECHO'
            self.motivo = 'Ya en tolerancia'
        elif abs(error) <= self.ant:
            self.estado = 'HECHO'
            self.motivo = 'Dentro de zona anticipacion (sin accion)'
        else:
            self.estado = 'PREPARADO'

    def arrancar(self, ahora):
        if self.estado != 'PREPARADO':
            return
        # Armar direccion y signo antes de aplicar PWM.
        if self.sentido == 1:
            self.in1.value(1)
            self.in2.value(0)
        else:
            self.in1.value(0)
            self.in2.value(1)
        self._signo(self.sentido)
        self.pwm.duty_u16(int(self.pwm_inicio * 65535 / 100))
        self.estado = 'MOVIENDO'
        self.t_inicio = ahora
        self.t_anterior = ahora
        self.t_ultimo_pulso = ahora

    def cortar(self, motivo, ahora, error=False):
        self.apagar()
        self.motivo = motivo
        self.t_corte = ahora
        self.estado = 'FALLO' if error else 'ASENTANDO'
        # Se conserva signo durante movimiento residual en ASENTANDO.

    def actualizar(self, ahora):
        if self.estado == 'ASENTANDO':
            if ticks_diff(ahora, self.t_corte) >= ASENTAMIENTO_MS*1000:
                self._signo(0)
                self.estado = 'HECHO'
            return
        if self.estado != 'MOVIENDO':
            return
        dt_us = ticks_diff(ahora, self.t_anterior)
        if dt_us <= 0:
            return
        dt = dt_us / 1000000.0
        n = self.lectura()
        theta = n * self.k
        error = self.referencia - theta
        error_dir = self.sentido * error
        elapsed_ms = ticks_diff(ahora, self.t_inicio) / 1000.0

        # Mismas reglas fundamentales de los controladores originales.
        if ((self.sentido == 1 and theta >= self.lim_max)
                or (self.sentido == -1 and theta <= self.lim_min)):
            self.cortar('Limite angular estimado', ahora, True)
            return
        if error_dir <= self.ant:
            self.cortar('Parada anticipada', ahora)
            return
        if abs(theta - self.inicial) >= self.max_cambio:
            self.cortar('Cambio maximo', ahora, True)
            return
        if elapsed_ms >= self.timeout_ms:
            self.cortar('Tiempo maximo', ahora, True)
            return
        if n != self.n_ultimo:
            self.n_ultimo = n
            self.t_ultimo_pulso = ahora
        if ticks_diff(ahora, self.t_ultimo_pulso) >= self.sin_pulsos_ms * 1000:
            self.cortar('Sin pulsos', ahora, True)
            return

        omega = (n - self.n_anterior) * self.k / dt
        self.omega_f = self.alpha*omega + (1-self.alpha)*self.omega_f
        p = self.kp * error
        d = -self.kd * self.omega_f
        i_candidate = limitar(self.i + self.ki*error*dt, -self.int_max, self.int_max)
        u_candidate = self.sentido * (p + i_candidate + d)
        u_old = self.sentido * (p + self.i + d)
        # antiwindup: solamente permitir integracion si no agrava saturacion
        if (0 < u_candidate < self.pwm_max or
            (u_old >= self.pwm_max and self.sentido*error < 0) or
            (u_old <= 0 and self.sentido*error > 0)):
            self.i = i_candidate
        u_dir = self.sentido*(p + self.i + d)
        if u_dir <= 0:
            self.cortar('PID solicita detener', ahora)
            return
        pwm = limitar(u_dir, self.pwm_min, self.pwm_max)
        if elapsed_ms < self.arranque_ms:
            pwm = self.pwm_inicio
        self.pwm.duty_u16(int(limitar(pwm, 0, 100) * 65535 / 100))
        self.n_anterior = n
        self.t_anterior = ahora

# Crear los ejes con parametros EXACTOS de los archivos nuevos.
base, hombro, codo = [Eje(c) for c in CONFIG]
ejes = (base, hombro, codo)

# MG996R muñeca (archivo aportado), GPIO12: 1000 a 2000 us
servo_mg = PWM(Pin(12), freq=50, duty_u16=0)
# SG90 gripper (archivo anterior), GPIO13: 500 a 2500 us
servo_sg = PWM(Pin(13), freq=50, duty_u16=0)

def duty_servo(ang, pulso_min, pulso_max):
    if not 0 <= ang <= 180:
        raise ValueError('Angulo servo fuera de 0..180')
    us = pulso_min + (pulso_max-pulso_min)*ang/180.0
    return int(us*65535/20000)

referenciado = False

def detener_todos():
    for eje in ejes:
        eje.apagar()
    # Servo PWM: no se corta automaticamente para evitar perder su consigna.

def establecer_ceros():
    global referenciado
    # SOLO cuando los tres ejes esten fisicamente en ceros conocidos, inmoviles.
    for eje in ejes:
        eje.cero()
    referenciado = True
    print('Tres articulaciones referenciadas en cero')

def posiciones():
    q = tuple(eje.angulo() for eje in ejes)
    print('BASE %.2f  HOMBRO %.2f  CODO %.2f' % q)
    return q

def mover_5(q_base, q_codo, q_hombro, ang_mg996r, ang_sg90):
    """Lanzar las cinco consignas casi al mismo instante, terminacion independiente."""
    if not referenciado:
        print('Primero establecer_ceros()')
        return False
    # Validar todo antes de iniciar NINGUN actuador.
    # Internamente ejes = (base, hombro, codo): adaptar orden de argumentos.
    objetivos = (float(q_base), float(q_hombro), float(q_codo))
    ang_mg996r = float(ang_mg996r)
    ang_sg90 = float(ang_sg90)
    d_mg = duty_servo(ang_mg996r, 1000, 2000)
    d_sg = duty_servo(ang_sg90, 500, 2500)
    inicio = ticks_us()
    for eje, ref in zip(ejes, objetivos):
        eje.preparar(ref, inicio)

    # Despacho consecutivo SIN ESPERAS BLOQUEANTES.
    # No es inicio fisico exacto al mismo microsegundo.
    try:
        instante = ticks_us()
        for eje in ejes:
            eje.arrancar(instante)
        servo_mg.duty_u16(d_mg)
        servo_sg.duty_u16(d_sg)

        ultimo_ciclo = instante
        while True:
            ahora = ticks_us()
            if ticks_diff(ahora, ultimo_ciclo) < TS_US:
                sleep_ms(1)
                continue
            ultimo_ciclo = ahora
            for eje in ejes:
                eje.actualizar(ahora)
            if any(e.estado == 'FALLO' for e in ejes):
                for e in ejes:
                    if e.estado == 'MOVIENDO':
                        e.cortar('Parada por fallo en otro eje', ahora, True)
                break
            if all(e.estado == 'HECHO' for e in ejes):
                break
            if ticks_diff(ahora, instante) >= MAX_CONTROL_TOTAL_MS * 1000:
                for e in ejes:
                    if e.estado == 'MOVIENDO':
                        e.cortar('Timeout total', ahora, True)
                break
    except KeyboardInterrupt:
        print('Interrumpido por usuario')
        return False
    finally:
        # Aun ante errores se corta TODO motor DC.
        detener_todos()
        # Contar residual solo con el ultimo signo ordenado.
        sleep_ms(ASENTAMIENTO_MS)
        for eje in ejes:
            eje._signo(0)

    print('\nRESULTADOS')
    exito = True
    for eje in ejes:
        actual = eje.angulo()
        err = eje.referencia - actual
        ok = abs(err) <= eje.tol and eje.estado != 'FALLO'
        if not ok:
            exito = False
        print('%s: ref=%.2f real_est=%.2f error=%.2f estado=%s motivo=%s' %
              (eje.nombre, eje.referencia, actual, err, eje.estado, eje.motivo))
    print('MG996R ordenado:', ang_mg996r, 'SG90 ordenado:', ang_sg90)
    print('Posicion de servos no verificada por sensores externos')
    return exito

# Inicializar sin ordenar movimiento de forma automatica
# Las ordenes de los servos solo se activan cuando se invoque mover_5().
detener_todos()
print('Robot: 3 PID + 2 servos. Inicio coordinado.')
print('Comandos: establecer_ceros(), posiciones(), mover_5(base,codo,hombro,muneca,gripper)')
