"""Ver el robot en RViz2 y moverlo con deslizadores (sin Gazebo).
Uso:  ros2 launch urdfscarafinal display.launch.py"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('urdfscarafinal')
    desc = ParameterValue(Command(['xacro ', os.path.join(pkg, 'urdf', 'scara_gazebo.urdf.xacro')]), value_type=str)
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': desc}]),
        Node(package='joint_state_publisher_gui', executable='joint_state_publisher_gui'),
        Node(package='rviz2', executable='rviz2'),
    ])
