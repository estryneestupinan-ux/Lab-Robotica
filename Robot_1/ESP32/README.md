# ESP32

Firmware en **MicroPython** del microcontrolador que gobierna los motores y servos del Robot 1. Recibe por el puerto serie las referencias articulares que publica ROS 2, ejecuta el control PID de cada motor y responde con su estado.

> **Lee primero.** Este directorio contiene la historia del desarrollo, no un único firmware definitivo. Las versiones usan protocolos distintos y varias **no accionan ningún motor**. No mezclar firmware y puente sin comprobar que hablan el mismo protocolo (ver [`Ejecucion_Comandos`](../Ejecucion_Comandos/README.md)).

## Estructura

```text
ESP32/
├── versiones/       Etapas de integración con ROS 2, de diagnóstico a control
└── referencia_PID/  Control de los 5 actuadores sin ROS 2 (referencia histórica)
```

## Versiones

Para cargar una versión en la placa, copiarla al ESP32 con el nombre `main.py` (por ejemplo con Thonny). Cerrar Thonny antes de conectar la placa a WSL.

| Archivo | Acciona hardware | Protocolo | Descripción |
|---|---|---|---|
| `main_ros2_trayectoria_sin_potencia.py` | No | `REF,seq,base,codo,hombro,muneca` | Recibe y valida referencias sin crear salidas PWM. Compatible con `ros2_esp32_puente_ack.py` |
| `main_ros2_limites_configurados.py` | No | `REF,...` | Valida cada referencia contra los límites de `CONFIG` y de los servos. Paso previo a mover el robot |
| `main_robot2_integracion.py` | No | `REF,...` | Etapa de diagnóstico para las tres juntas con motor y la muñeca. No configura GPIO de drivers |
| `main_receptor_solo_base.py` | No | `BASE_REF,seq,angulo` | Receptor de la base con salidas en cero. Compatible con `puente_ros2_solo_base.py` |
| `main_base_ros2_encoder_AB.py` | No (PWM bloqueado por software) | `REF,...` | Base con encoder en cuadratura A/B. Requiere `HOME_CONFIRM` |
| `main_base_ros2_guardado.py` | **Sí, solo la base** | `REF,...` tras `ARM_BASE` | Arranca desarmado. Limitado a HOME ±2°, PWM máximo del 30 % y movimientos de máximo 700 ms |
| `main_ros2_5actuadores.py` | **Sí, 3 motores y servos** | `REF,...` tras `ARM_ALL` | Control completo. Referencias limitadas a HOME ±2° en las cuatro juntas. Arranca desarmado |

Las dos últimas pueden mover el robot. La documentación del proyecto indica que `main_ros2_5actuadores.py` y `main_robot2_integracion.py` se conservan como **referencia histórica** y que hubo fallos de parada e inversión en codo y hombro durante los ensayos.

## Protocolo serie

Texto ASCII, una orden por línea, a 115200 baudios. Los ángulos están en grados y expresados como en ROS 2.

| Orden | Respuesta | Función |
|---|---|---|
| `PING` | `PONG` | Comprobar la comunicación |
| `STATUS` | `STATUS,...` | Estado: HOME, armado, secuencia, recibidas, rechazadas y referencia |
| `HOME_CONFIRM` | `HOME_CONFIRMADO,...` | Declara que el robot está físicamente en HOME y fija el cero lógico |
| `ARM_BASE` / `ARM_ALL` | — | Habilita el movimiento de la base o de todos los ejes, según la versión |
| `REF,seq,base,codo,hombro,muneca` | `ACK` o `ERR,...` | Referencia articular. `seq` debe ser creciente |
| `GRIP,angulo` | `GRIP_OK` | Ángulo del servo de la pinza (solo `main_ros2_5actuadores.py`) |
| `STOP` / `HOME_INVALIDAR` | `STOP,...` | Apaga las salidas e invalida el HOME |

Si no llegan referencias nuevas durante unos 250 a 300 ms, el firmware descarta la referencia (*timeout*). Una orden desconocida devuelve `ERR,COMANDO`.

## Hardware

HOME en coordenadas de ROS 2: `(0, 0, −180, 0)`.

| Eje | Articulación ROS 2 | IN1 / IN2 / PWM | Encoder | Grados por pulso | Kp / Ki / Kd |
|---|---|---|---|---|---|
| BASE | `joint_1` | 25 / 26 / 27 | GPIO 32 (B = 33, propuesto) | 360 / 7585,125 | 3 / 0,1 / 0,15 |
| CODO | `joint_2` | 19 / 21 / 18 | GPIO 34 | 0,0177 | 6 / 0,5 / 0,15 |
| HOMBRO | `joint_3` | 2 / 4 / 15 | GPIO 23 | 0,0165 | 6 / 0,5 / 0,15 |

| Servo | Función | GPIO | Pulso (µs) |
|---|---|---|---|
| MG996R | Muñeca (`joint_4`), consigna física nominal `150° + joint_4` | 12 | 1000 a 2000 |
| SG90 | Pinza | 13 | 500 a 2500 |

Los drivers de motor son L298N y el periodo del lazo de control es de 10 ms. Los nombres CODO y HOMBRO siguen la convención del firmware. Los encoders cuentan solo el canal A, salvo la prueba en cuadratura de la base, cuya calibración en grados por cuenta **todavía no está definida**.

## `referencia_PID/`

`control_5_actuadores_orden_base_codo_hombro.py` controla base, hombro y codo con PID más los servos MG996R y SG90, sin ROS 2: arranca de forma coordinada y termina cada eje de manera independiente. Es la referencia de la que se tomaron los parámetros de las versiones de `versiones/`.

Difiere de ellas en el sentido de giro de base y codo (`IN1` e `IN2` intercambiados) y en los límites angulares. No usarlo junto con los puentes ROS 2 sin revisarlo.

## Advertencias

- Los límites angulares de `CONFIG` provienen de programas individuales; **no son límites mecánicos certificados**.
- Mantener la potencia de los motores desconectada en las pruebas de comunicación.
- Para pruebas mecánicas se necesitan supervisión, límites fiables y un corte físico de potencia.
- Un HOME confirmado no mueve el robot: solo registra que ya está en esa posición.
