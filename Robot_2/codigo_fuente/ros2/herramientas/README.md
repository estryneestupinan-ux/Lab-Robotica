# Herramientas

## `espejo_gazebo_real.py`

Hace que el robot físico copie lo que hace Gazebo.

- Lee `/joint_states` de Gazebo y envía al ESP32 la pose como objetivo de posición (`qq`) 20 veces por segundo.
- Copia la pinza leyendo `Revoluci_n_12`.
- Publica la posición real de los encoders en `/robot_real/joint_states`.

Protecciones:

- No mueve nada hasta que el ESP32 reporte home en los tres ejes (`homed = 111`).
- El objetivo avanza como máximo 0.20 rad/s y 0.15 m/s, así que si el robot arranca lejos de Gazebo se acerca despacio.
- Recorta q1, q2 y d3 a los límites del robot real.
- Si el ESP32 entra en FALLA, deja de enviar y avisa.
- Al salir con `Ctrl + C` envía `s` (parada).

Al abrir el puerto el ESP32 se reinicia: el nodo espera 3 s, envía `rep 1`, `reset` y `home`, y empieza a seguir a Gazebo cuando el home termina.

```bash
source ~/ros2_ws/install/setup.bash
python3 espejo_gazebo_real.py --ros-args -p puerto:=/dev/ttyUSB0
```

No lo ejecutes junto con `puente_robot_real.py`: los dos usarían el mismo puerto. Los parámetros están en el [manual de usuario](../../../documentacion/manual_usuario/README.md#parámetros-del-espejo).
