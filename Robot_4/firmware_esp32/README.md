# Firmware del ESP32: control de bajo nivel del SCARA R4

Proyecto de PlatformIO (framework Arduino) que cierra el lazo de posición de los tres motores, mueve el servo de la pinza y se comunica con el computador por USB serial. Versión: **2026-11-18f**.

```
firmware_esp32/
├── platformio.ini      ← entornos featheresp32 (con el que se grabó el robot) y esp32dev
└── src/main.cpp        ← todo el firmware, comentado por secciones
```

## Compilar y grabar

```bash
cd firmware_esp32
pio run -t upload                 # compila y graba (env featheresp32)
pio device monitor                # monitor a 115200 baudios
```

También funciona desde VS Code con la extensión de PlatformIO. El código compila con el Arduino core 2.x y 3.x (detecta la versión con `ESP_ARDUINO_VERSION_MAJOR`).

> ⚠️ **GPIO 12.** El canal A del encoder de la base está en el GPIO 12, que el ESP32 lee al arrancar para fijar el voltaje de la flash. Si la base queda detenida con ese canal en alto, el ESP32 entra en un ciclo de reinicios (`invalid header`). Mientras no se corrija, desconectar el conector U6 al encender o grabar. Solución definitiva: mover el canal al GPIO 18 o fijar el voltaje de la flash con `espefuse.py --port /dev/esp32 set_flash_voltage 3.3V` (solo módulos con flash de 3.3 V, como los WROOM-32; es **irreversible**).

## Organización del código

| Sección de `main.cpp` | Función |
|---|---|
| Configuración general | Periodo Ts = 20 ms, PWM de 20 kHz y 8 bits, límite PWM 120, watchdog de 500 ms, interruptores de las protecciones |
| Servo de la pinza | GPIO 23, canal LEDC 6 a 50 Hz y 16 bits, pulso de 500 a 2500 µs, ángulos abierta (105°) y cerrada (60°) |
| Configuración por motor (`CFG`) | Pines, sentido, cuentas por unidad, ganancias P y LGR, PWM mínimo y tolerancia de cada motor |
| Interrupciones | `isrEncoder`: suma o resta una cuenta en cada flanco de subida del canal A según el nivel del canal B, y descarta picos de ruido. `isrTimer`: levanta la bandera de control cada 20 ms |
| Control | `calcularPwm`: ley P con zona muerta o LGR. `pasoControl`: watchdog, error de cada junta, PWM y telemetría |
| Comandos | `procesarLinea`: interpreta el protocolo serial |
| `setup()` / `loop()` | Inicializa pines, PWM, interrupciones, servo y timer. El `loop` atiende la bandera del timer y lee el puerto serial |

El diagrama de flujo está en [Documentos/Diagramas](../Documentos/Diagramas/README.md#diagrama-de-flujo-del-firmware-del-esp32).

## Motores y conversión

| Motor | Junta | Driver | PWM | IN1, IN2 | Encoder A, B | Constante |
|---|---|---|---|---|---|---|
| M1 | Eje Z (N20) | U7, canal A | GPIO 16 | 33, 32 | 13, 27 | 7·1000/8 = 875 cuentas/mm |
| M2 | Codo (JGB37 + correa 20:60) | U7, canal B | GPIO 17 | 25, 26 | 22, 21 | 11·810·3/2π = 4254.2 cuentas/rad |
| M3 | Base (JGB37) | U1, canal A | GPIO 15 | 4, 2 | 12, 14 | 11·810/2π = 1418.1 cuentas/rad |

Convención: PWM positivo hace **subir** las cuentas (`invertir = true` en M1 y M2 lo garantiza).

## Control

**Proporcional con compensación de zona muerta** (por defecto, `MODO:P`):

```
u = sign(e) · min(u_max, u_min + Kp·|e|)   si |e| > e_tol
u = 0                                      si |e| ≤ e_tol
```

| Junta | Kp | u_min | e_tol | u_max |
|---|---|---|---|---|
| Base | 400 PWM/rad | 35 | 0.005 rad | 120 |
| Codo | 1200 PWM/rad | 35 | 0.005 rad | 120 |
| Eje Z | 80 PWM/mm | 35 | 0.1 mm | 255 |

**LGR** (`MODO:LGR`): `u[k] = u[k−1] + 6.9327·(e[k] − 0.0435·e[k−1])`, con saturación y anti-windup (guarda el u ya saturado). Diseñado sobre G(s) = 0.046814/(1 + 0.002554 s); implementado pero no validado en el robot real.

**Seguridad**: el watchdog apaga los motores si pasan 500 ms sin `SET` y vuelven a funcionar con el siguiente comando. `PROTECCION_ATASCO` (PWM aplicado sin que avancen las cuentas en 1 s) y `PROTECCION_ERROR` (error mayor a 0.8 rad o 10 mm) están en `false` en esta versión; se reactivan poniéndolas en `true`.

## Protocolo serial (115200 baudios, una línea por mensaje)

| Comando | Función |
|---|---|
| `SET:c1,c2,c3` | Consignas en cuentas de M1 (Z), M2 (codo) y M3 (base); activa el modo CONTROL |
| `STOP` | Apaga los motores (el conteo sigue) |
| `ZERO` | La posición actual pasa a ser la cuenta 0 (pose cero) |
| `PRUEBA[:m[,v]]` | Prueba automática: cada motor a +PWM y −PWM durante 0.8 s, con veredicto |
| `PWM:m,v` | Lazo abierto: motor m con PWM v (pruebas) |
| `MODO:P` · `MODO:LGR` | Cambia el controlador |
| `WD:ms` | Tiempo del watchdog (0 = desactivado) |
| `TEL:1` · `TEL:0` | Enciende o apaga la telemetría |
| `INFO` | Versión y configuración |
| `SERVO:θ` · `SERVO:OFF` · `SERVO_US:us` | Servo a θ grados, sin pulsos o con un pulso dado (calibración) |
| `PINZA:ABRIR` · `PINZA:CERRAR` · `PRUEBA_SERVO[:a,b]` | Atajos de la pinza y prueba del servo |

Respuestas: `POS:t_ms,c1,c2,c3,pwm1,pwm2,pwm3` cada 20 ms (con `TEL:1`), `MSG:…` y `ERR:…`.

## Verificación rápida

En el monitor serial, un comando a la vez:

1. `INFO` → `MSG:firmware 2026-11-18f …` y `proteccion_atasco=OFF proteccion_error=OFF`.
2. `PRUEBA` → tres `OK` (Z ≈ 950 cuentas, codo ≈ 250, base ≈ 250 con la fuente de 12 V).
3. Robot a mano en la pose cero (brazo extendido, Z arriba) → `ZERO`.
