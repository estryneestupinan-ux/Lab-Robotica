# SCARA urdfscarafinal: ROS 2 Jazzy + Gazebo Harmonic (Ubuntu 24.04)

## Paso 1: cargar el robot en Gazebo

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui \
  python3-colcon-common-extensions python3-matplotlib

mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
unzip ~/Descargas/urdfscarafinal_ros2.zip
cd ~/ros2_ws && colcon build --symlink-install
source install/setup.bash
ros2 launch urdfscarafinal gazebo.launch.py
```

Ver el robot en RViz2 sin Gazebo: `ros2 launch urdfscarafinal display.launch.py`

## Archivos
* `urdf/scara_gazebo.urdf.xacro`: URDF para Gazebo (nombres sin espacios, robot fijo al suelo
  con J1 en el origen, TCP, ros2_control, colores). El original `urdf/urdfscarafinal.urdf` se conserva.
* `config/controladores.yaml`: joint_state_broadcaster, arm_controller y gripper_controller.
* `scripts/planificador_trayectorias.py`: planeación de trayectorias (cúbica, quíntica, trapezoidal).
* `docs/`: gráficas de la planeación.
