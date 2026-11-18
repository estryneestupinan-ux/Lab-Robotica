# GUI

Interfaz gráfica de monitoreo del Robot 1 en ROS 2.

## Contenido

| Archivo | Descripción |
|---|---|
| `monitor_ros2_gui.py` | Monitor de **solo lectura** hecho con Tkinter |
| `GUI_ORIGINAL_FALTANTE.txt` | Nota sobre la GUI original que no está en el repositorio |

## `monitor_ros2_gui.py`

Se suscribe al tópico `/arm_controller/controller_state` y muestra, para `joint_1` a `joint_4`, la referencia, la posición actual en Gazebo y el error, todo en grados. Se actualiza cada 100 ms.

| Columna | Origen |
|---|---|
| Referencia (°) | Referencia del controlador de trayectoria |
| Actual Gazebo (°) | *Feedback* del controlador |
| Error (°) | Referencia menos posición actual |

**No envía órdenes** ni al controlador ni al ESP32, y no activa motores ni servos. Sirve para observar una trayectoria mientras se ejecuta en simulación.

### Requisitos

- ROS 2 con el paquete `control_msgs` y `rclpy`.
- Python 3 con Tkinter (`sudo apt install python3-tk` en Ubuntu).
- El controlador `arm_controller` activo (ver [`Ejecucion_Comandos`](../Ejecucion_Comandos/README.md)).

### Ejecución

```bash
source /opt/ros/lyrical/setup.bash
source ~/robot2_ws/install/setup.bash
python3 GUI/monitor_ros2_gui.py
```

Mientras no lleguen mensajes, la ventana muestra `Esperando /arm_controller/controller_state ...`.

## GUI original no incluida

El equipo usa en el robot físico el script `puente_trayectoria_con_servos.py`, que combina el puente ROS 2 ↔ ESP32, el control cinemático, una GUI y el manejo de servos. **Ese archivo no está en el repositorio**: solo existe en el WSL del equipo. `monitor_ros2_gui.py` es un programa distinto y no lo reemplaza.

Para recuperarlo, copiarlo desde el WSL donde se desarrolló:

```bash
cp ~/robot2_ws/scripts/puente_trayectoria_con_servos.py <destino>/robot2_ws/scripts/
```

Revisar su comportamiento y su compatibilidad con el firmware antes de usarlo, y subirlo al repositorio para no perderlo.
