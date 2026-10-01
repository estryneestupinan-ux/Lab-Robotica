# 🤖 Proyecto: Control y Simulación de Robot SCARA R4 en ROS 2

## 📖 Descripción del Proyecto
Este repositorio contiene el desarrollo, simulación y control dinámico de un brazo robótico tipo SCARA R4 (3 Grados de Libertad: 2 rotacionales y 1 prismático). El proyecto abarca desde la exportación del modelo CAD hasta la simulación física en Gazebo y la planificación de trayectorias cartesianas.

## 🎯 Objetivos de la Tarea Actual
1. **Simulación Física:** Integrar el modelo URDF en Gazebo utilizando el motor de físicas `bullet-featherstone` y `ros2_control`.
2. **Cinemática Inversa:** Calcular la posición de las articulaciones para seguir una trayectoria en línea recta cartesiana desde $P_0 = (200, 50, 140)$ hasta $P_1 = (100, 200, 80)$.
3. **Análisis Dinámico:** Extraer y registrar los datos de posición, velocidad y esfuerzo (torque/fuerza) de las articulaciones durante la trayectoria con un tiempo de muestreo de $0.02\text{ s}$ ($50\text{ Hz}$).

## 🛠️ Tecnologías y Herramientas Utilizadas
* **ROS 2:** Lyrical / Humble
* **Simulador:** Gazebo Sim (Ignition)
* **Control:** ros2_control (position_controllers, velocity_controllers)
* **Lenguajes:** Python (Scripts de trayectoria), C++ / XML (URDF y Launch)

## 📁 Estructura del Repositorio
* `/brazo_solidworks`: Paquete principal de ROS 2.
* `/docs`: Archivos de datos (.csv), informe técnico y diagramas.

## 🚀 Cómo ejecutar la simulación
(Aquí agregaremos las instrucciones de compilación y los comandos `ros2 launch` y `ros2 run` más adelante).
