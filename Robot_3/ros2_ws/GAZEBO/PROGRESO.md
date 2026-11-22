# Progreso del proyecto SCARA en Gazebo

Entorno: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic
Carpeta de trabajo: ~/ros2_ws/src/urdfscarafinal

## Paso 1: robot en Gazebo — COMPLETO
- URDF corregido: urdf/scara_gazebo.urdf.xacro
  (nombres sin espacios, robot fijo al suelo con J1 en el origen, TCP,
  ros2_control, colores, dedos sin colisión y sin autocolisión)
- Controladores activos: joint_state_broadcaster, arm_controller, gripper_controller
- Probado: giro de eslabón 1 y 2, bajada de la cremallera, apertura y cierre de ambos dedos

## Paso 2: tareas con planeación de trayectorias — PENDIENTE
- Celda de trabajo (mesas, cubos, placa)
- Nodo que ejecuta las tareas (perfil cúbico / quíntico / trapezoidal)
- Gráficas planeada vs ejecutada

## Comandos útiles
Abrir Gazebo:            ./iniciar_gazebo.sh   (o el alias: scara)
Ver controladores:       ros2 control list_controllers
Ver articulaciones:      ros2 topic echo /joint_states --once --field position
Mover brazo:
  ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [eslabon1_joint, eslabon2_joint, prismatico_joint], points: [{positions: [0.8, 1.2, 0.04], time_from_start: {sec: 3}}]}"
Mover garra (0 = cerrada, 0.2 = sujeta cubo, 0.7 = abierta):
  ros2 topic pub --once /gripper_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: [garra1_joint, garra2_joint], points: [{positions: [0.7, 0.7], time_from_start: {sec: 2}}]}"
