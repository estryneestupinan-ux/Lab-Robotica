# Código fuente del robot SCARA

| Carpeta | Contenido |
|---|---|
| [`ros2/Robrot_description/`](ros2/Robrot_description) | **Paquete actual.** Modelo URDF/Xacro, launch de RViz y Gazebo, controladores, control cinemático, trayectorias y puente con el robot real |
| [`ros2/herramientas/`](ros2/herramientas) | `espejo_gazebo_real.py`: envía la pose de Gazebo al robot físico |
| [`ros2/Scara_Final_description/`](ros2/Scara_Final_description) | Versión anterior del modelo, con su nodo de cinemática inversa. Se conserva como referencia y ya no se usa |
| [`microcontrolador/scara_verificacion/`](microcontrolador/scara_verificacion) | Firmware del ESP32, visor web y guía de calibración |
