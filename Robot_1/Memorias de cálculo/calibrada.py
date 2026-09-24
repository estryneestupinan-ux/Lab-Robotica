from machine import Pin, PWM
import time

# ============================================
# PINES
# ============================================

PIN_PWM = 25
PIN_IN1 = 26
PIN_IN2 = 27

PIN_ENC_A = 32

# ============================================
# MOTOR
# ============================================

in1 = Pin(PIN_IN1, Pin.OUT)
in2 = Pin(PIN_IN2, Pin.OUT)

pwm = PWM(
    Pin(PIN_PWM),
    freq=10000,
    duty_u16=0
)

# ============================================
# ENCODER
# ============================================

enc_a = Pin(
    PIN_ENC_A,
    Pin.IN,
    Pin.PULL_UP
)

contador = 0


def encoder_isr(pin):

    global contador

    contador += 1


enc_a.irq(
    trigger=Pin.IRQ_RISING,
    handler=encoder_isr
)


# ============================================
# PWM
# ============================================

def set_pwm(percent):

    duty = int(
        percent *
        65535 /
        100
    )

    pwm.duty_u16(duty)


# ============================================
# MOTOR
# ============================================

def motor_forward(percent):

    in1.value(1)
    in2.value(0)

    set_pwm(percent)


def motor_stop():

    set_pwm(0)

    in1.value(0)
    in2.value(0)


# ============================================
# PRUEBA
# ============================================

contador = 0

print()
print("CALIBRACION DEL ENCODER")
print()

print("El motor girara al 45 %.")
print("Cuenta EXACTAMENTE 10 vueltas")
print("del eje de salida.")
print()

input("Presiona ENTER para comenzar...")

contador = 0

motor_forward(100)

try:

    while True:

        print("Pulsos =", contador)

        time.sleep(0.2)

except KeyboardInterrupt:

    motor_stop()

    print()
    print("Motor detenido.")
    print("Pulsos totales =", contador)