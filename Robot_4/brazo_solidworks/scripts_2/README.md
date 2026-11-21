# Nodos de Python del SCARA R4

Todos los nodos publican en los controladores de Gazebo. Con `puente_serial.py` corriendo, el robot real copia lo que hace Gazebo, así que **cualquier planeador mueve también el hardware** sin cambios de código.

| Script | Función | Publica | Escucha |
|---|---|---|---|
| `joint3_position_hold.py` | Lazo P de posición del eje Z en Gazebo, con saturación (0.1 m/s) y rampa de aceleración | `/joint3_velocity_controller/commands` | `/joint_states`, `/joint3_target` |
| `joint_angle_gui.py` | Sliders por junta, para pruebas y calibración | `/joint1…`, `/joint2_position_controller/commands`, `/joint3_target` | — |
| `ik_gui_teleop.py` | Control por coordenadas (X, Y, Z) con cinemática inversa (Guía 3). Mueve el Z con su propio lazo | `/joint1…`, `/joint2…`, `/joint3_velocity_controller/commands` | `/joint_states` |
| `trayectoria_lineal.py` | Recta cartesiana P₀ → P₁ en 5 s a 50 Hz; guarda `~/ros2_ws/trayectoria_datos.csv` | `/joint1…`, `/joint2…`, `/joint3_velocity_controller/commands` | `/joint_states` |
| `secuencia_gui.py` | Tarea completa con pinza (Guía 4), sincronizada con el robot real | `/joint1…`, `/joint2…`, `/joint3_target`, `/pinza` | `/joint_states`, `/real/joint_states` |
| `puente_serial.py` | Puente Gazebo → ESP32: el robot real copia a Gazebo | `/real/joint_states` | `/joint_states`, `/pinza` |
| `graficar_comparacion.py` | Gráfica Gazebo vs. robot real y errores de seguimiento a partir del CSV del puente | — | — |
| `99-esp32.rules` | Regla udev que crea `/dev/esp32` | — | — |

**No combinar** en la misma sesión: `joint_angle_gui.py` con `secuencia_gui.py` (publican en los mismos tópicos), ni `joint3_position_hold.py` con `ik_gui_teleop.py` o `trayectoria_lineal.py` (los tres mandan velocidad al eje Z).

## `secuencia_gui.py`

Tramos (Home → tapas → bajar y cerrar → subir → botella → bajar y abrir → subir → Home), cada uno con un perfil *smoothstep* q(t) = q₀ + Δq·(3τ² − 2τ³). Parámetros al inicio del archivo:

| Parámetro | Valor | Significado |
|---|---|---|
| `VEL_MAX` | (0.35 rad/s, 0.11 rad/s, 0.0028 m/s) | Velocidad pico por junta; si un tramo la superaría, su duración se alarga (T = máx(T_nom, 1.5·\|Δq\|/v)) |
| `Z_ABAJO`, `Z_BOTELLA` | 0.030 m | Bajada sobre la tapa y sobre la botella |
| `PINZA_ABIERTA`, `PINZA_CERRADA` | 105°, 60° | Ángulos calibrados del servo |
| `SOLTAR_PINZA` | `True` | Deja el servo sin pulsos tras abrir o cerrar (los encoders fallan con el servo activo) |
| `TOLERANCIA` | 0.02 rad, 0.02 rad, 3 mm | Cuándo se considera que el robot real llegó |
| `T_MAX_ESPERA` | 60 s | Si el robot real no llega en ese tiempo, la secuencia se detiene |
| `J1_MIN`, `J1_MAX` | −3.0, 3.2 rad | Rango de la base: se elige el equivalente ±2π más cercano para no dar vueltas completas |

## `puente_serial.py`

Cada 20 ms toma la posición de Gazebo, avanza la consigna hacia ella sin pasar `VEL_MAX` por junta (0.40 rad/s, 0.12 rad/s, 3 mm/s), la recorta a `LIMITES` y la envía como cuentas: `SET:c_Z,c_codo,c_base`.

| Parámetro | Valor | Significado |
|---|---|---|
| `PUERTO` | `/dev/esp32` | Ver `99-esp32.rules` |
| `SIGNO` | −1 (base), −1 (codo), +1 (Z) | En el robot real, las cuentas positivas giran la base y el codo en sentido horario, al revés que en Gazebo |
| `CUENTAS` | 1418.1, 4254.2 cuentas/rad; 875 cuentas/mm | Constantes de conversión de cada junta |
| `MOTOR_DE` | base → M3, codo → M2, Z → M1 | Conector de la PCB de cada junta |

Seguridad: la primera consigna es la posición medida (sin saltos al arrancar) y, si el ESP32 se reinicia, deja de mandar consignas porque las cuentas volvieron a cero. El CSV `~/ros2_ws/robot_real_<fecha>.csv` guarda, por muestra, el tiempo del ESP32, la posición de Gazebo, la consigna, la posición real y el ángulo de la pinza.

## `graficar_comparacion.py`

```bash
python3 graficar_comparacion.py              # toma el CSV más reciente de ~/ros2_ws
python3 graficar_comparacion.py archivo.csv
```

Dibuja posición y velocidad de cada junta (Gazebo y real), marca los movimientos de la pinza, imprime el error máximo, RMS y final, y guarda la figura como PNG junto al CSV.
