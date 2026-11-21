# Código fuente ROS 2

## Entorno probado

- Ubuntu con Python 3.14
- ROS 2 Lyrical, Gazebo (gz sim), `ros2_control` y `gz_ros2_control`

## Paquete `Robrot_description` (actual)

```text
Robrot_description/
├── urdf/                  Robrot.urdf.xacro (argumento sim:=true para Gazebo) y ensambles
├── meshes/                Mallas visuales (.dae) y de colisión (.stl)
├── launch/
│   ├── display.launch.py  RViz con hardware simulado
│   └── gazebo.launch.py   Gazebo con gz_ros2_control
├── config/
│   ├── gz_controllers.yaml     Controladores para Gazebo
│   └── ros2_controllers.yaml   Controladores para RViz
├── scripts/
│   ├── control_cinematico.py   Control cinemático con saturación y eventos de pinza
│   ├── graficar.py             Gráficas de los registros en ~/scara_logs
│   └── puente_robot_real.py    Robot real sin Gazebo
├── trayectorias/          T1.txt … T19.txt (t, q1, q2, d3) y originales/
├── rviz/display.rviz
└── LEEME_CAMBIOS.md       Correcciones, puente con el robot real y calibración
```

## Compilar y ejecutar

```bash
mkdir -p ~/ros2_ws/src
cp -r Robrot_description ~/ros2_ws/src/
cd ~/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --packages-select Robrot_description
source install/setup.bash

ros2 launch Robrot_description gazebo.launch.py
ros2 run Robrot_description control_cinematico.py --ros-args -p use_sim_time:=true -p trayectoria:=T4
```

## Herramientas

`herramientas/espejo_gazebo_real.py` no es parte del paquete: se ejecuta con `python3` después de cargar el entorno de ROS. Lee `/joint_states` de Gazebo y le envía la pose al ESP32 por USB.

```bash
source ~/ros2_ws/install/setup.bash
python3 herramientas/espejo_gazebo_real.py --ros-args -p puerto:=/dev/ttyUSB0
```

El procedimiento completo está en el [manual de usuario](../../documentacion/manual_usuario/README.md).

## Paquete `Scara_Final_description` (anterior)

Primera versión del modelo, con `scara_gazebo.launch.py` y un nodo de cinemática inversa por terminal. Se conserva como referencia; la implementación actual usa `Robrot_description`.
