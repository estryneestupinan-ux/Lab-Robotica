from machine import Pin, PWM
import time
import micropython

# ============================================================
# JGA25-370 - IDENTIFICACION EXPERIMENTAL
# ESP32 + MicroPython
# ============================================================

micropython.alloc_emergency_exception_buf(100)


# ============================================================
# 1. PINES
# ============================================================

# Puente H
PIN_PWM = 25
PIN_IN1 = 26
PIN_IN2 = 27

# Encoder
PIN_ENC_A = 32       # Amarillo
PIN_ENC_B = 33       # Verde


# ============================================================
# 2. PARAMETROS
# ============================================================

V_SUPPLY = 12.0

PWM_FREQ = 10000


# ============================================================
# 3. ENCODER
# ============================================================

PULSES_PER_OUTPUT_REV = 1088.1

# ============================================================
# 4. MUESTREO
# ============================================================

# 20 ms
TS_MS = 20

# 50 Hz aproximadamente


# ============================================================
# 5. PINES DEL PUENTE H
# ============================================================

in1 = Pin(PIN_IN1, Pin.OUT)
in2 = Pin(PIN_IN2, Pin.OUT)

pwm = PWM(
    Pin(PIN_PWM),
    freq=PWM_FREQ,
    duty_u16=0
)


# ============================================================
# 6. ENCODER
# ============================================================

enc_a = Pin(
    PIN_ENC_A,
    Pin.IN,
    Pin.PULL_UP
)

enc_b = Pin(
    PIN_ENC_B,
    Pin.IN,
    Pin.PULL_UP
)


encoder_count = 0


# ============================================================
# 7. INTERRUPCION
# ============================================================

def encoder_isr(pin):

    global encoder_count

    # IMPORTANTE:
    # Para identificación el motor gira siempre
    # en el mismo sentido.
    #
    # Por tanto simplemente contamos pulsos.

    encoder_count += 1


enc_a.irq(
    trigger=Pin.IRQ_RISING,
    handler=encoder_isr
)


# ============================================================
# 8. PWM
# ============================================================

def set_pwm(percent):

    if percent < 0:
        percent = 0

    if percent > 100:
        percent = 100

    duty = int(
        percent *
        65535 /
        100
    )

    pwm.duty_u16(duty)


# ============================================================
# 9. MOTOR
# ============================================================

def motor_forward(percent):

    in1.value(1)
    in2.value(0)

    set_pwm(percent)


def motor_stop():

    set_pwm(0)

    in1.value(0)
    in2.value(0)


# ============================================================
# 10. SECUENCIA DE IDENTIFICACION
# ============================================================

# Evitamos la zona inferior al 40 %

sequence = [

    (0,   2.0),

    (45,  3.0),
    (60,  3.0),
    (80,  3.0),
    (100, 3.0),

    (70,  3.0),
    (90,  3.0),
    (55,  3.0),
    (85,  3.0),
    (65,  3.0),

    (0,   3.0)
]

# ============================================================
# 11. EXPERIMENTO
# ============================================================

def run_experiment():

    global encoder_count

    encoder_count = 0

    previous_count = 0

    experiment_start = time.ticks_ms()


    print()
    print(
        "time_s,pwm_pct,voltage_equiv_V,"
        "rpm,pulses_interval,count"
    )


    try:

        for pwm_percent, duration_s in sequence:


            # =================================================
            # APLICAR ESCALON
            # =================================================

            if pwm_percent == 0:

                motor_stop()

            else:

                motor_forward(
                    pwm_percent
                )


            step_start = time.ticks_ms()


            # =================================================
            # MUESTREO
            # =================================================

            next_sample = time.ticks_add(
                time.ticks_ms(),
                TS_MS
            )


            while time.ticks_diff(
                time.ticks_ms(),
                step_start
            ) < int(duration_s * 1000):


                # Esperar hasta el siguiente instante
                # de muestreo

                while time.ticks_diff(
                    next_sample,
                    time.ticks_ms()
                ) > 0:

                    time.sleep_ms(1)


                current_time = time.ticks_ms()


                # =============================================
                # ENCODER
                # =============================================

                current_count = encoder_count

                delta_pulses = (
                    current_count -
                    previous_count
                )

                previous_count = (
                    current_count
                )


                # =============================================
                # RPM
                # =============================================

                dt_s = TS_MS / 1000.0

                revolutions = (
                    delta_pulses /
                    PULSES_PER_OUTPUT_REV
                )

                rpm = (
                    revolutions *
                    60 /
                    dt_s
                )


                # =============================================
                # TIEMPO
                # =============================================

                elapsed_ms = time.ticks_diff(
                    current_time,
                    experiment_start
                )

                elapsed_s = (
                    elapsed_ms /
                    1000
                )


                # =============================================
                # VOLTAJE EQUIVALENTE
                # =============================================

                voltage_equiv = (
                    V_SUPPLY *
                    pwm_percent /
                    100
                )


                # =============================================
                # DATOS
                # =============================================

                print(
                    "{:.3f},{:.1f},{:.3f},{:.3f},{},{}".format(
                        elapsed_s,
                        pwm_percent,
                        voltage_equiv,
                        rpm,
                        delta_pulses,
                        current_count
                    )
                )


                # Próximo instante exacto

                next_sample = time.ticks_add(
                    next_sample,
                    TS_MS
                )


    except KeyboardInterrupt:

        print()
        print("# EXPERIMENTO INTERRUMPIDO")


    finally:

        motor_stop()

        print("#END")
        print("Motor detenido.")


# ============================================================
# 12. INICIO
# ============================================================

motor_stop()


print()
print("======================================")
print(" JGA25-370 - SYSTEM IDENTIFICATION")
print(" ESP32 + MicroPython")
print("======================================")

print()

print(
    "Pulsos salida:",
    PULSES_PER_OUTPUT_REV
)

print()

print(
    "Periodo:",
    TS_MS,
    "ms"
)

print(
    "Frecuencia:",
    1000 / TS_MS,
    "Hz"
)

print()

print("Escribe S para comenzar")


while True:

    command = input("> ")

    if command.upper() == "S":

        print()
        print(
            "Experimento inicia en 2 segundos..."
        )

        time.sleep(2)

        run_experiment()

        break

    else:

        print("Escribe S")