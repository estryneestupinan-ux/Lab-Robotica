# Documentación técnica del robot SCARA

Esta carpeta documenta el diseño, la simulación, el control y la validación del manipulador SCARA RRP en su versión actual: el paquete `Robrot_description` en ROS 2 y Gazebo, y el robot físico controlado por el ESP32.

## Organización

| Carpeta | Contenido |
|---|---|
| [`arquitectura/`](arquitectura/README.md) | Componentes de software y hardware, nodos, tópicos, orden de las juntas y protocolo serie |
| [`diagramas_secuencia/`](diagramas_secuencia/README.md) | Arranque del sistema, ciclo de cada trayectoria con la pinza y enlace con el robot físico |
| [`memorias_calculo/`](memorias_calculo/README.md) | Dimensiones, cinemática, Jacobiano, conversión modelo–firmware, correcciones al modelo y control |
| [`manual_usuario/`](manual_usuario/README.md) | Instalación, operación en Gazebo y en el robot físico, y solución de problemas |
| [`Planos/`](Planos/README.md) | Dimensiones, recorridos, conexiones eléctricas y planos pendientes |
