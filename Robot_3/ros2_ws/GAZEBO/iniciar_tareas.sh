#!/bin/bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch urdfscarafinal simulacion_tareas.launch.py perfil:=${1:-quintico}
