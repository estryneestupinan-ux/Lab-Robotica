#!/bin/bash
# Abre el SCARA en Gazebo con un solo comando.
# Uso:  ~/ros2_ws/src/urdfscarafinal/iniciar_gazebo.sh
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch urdfscarafinal gazebo.launch.py
