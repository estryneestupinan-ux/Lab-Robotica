"""PASO 1: cargar el SCARA en Gazebo Harmonic (ROS 2 Jazzy) con sus controladores.
Uso:  ros2 launch urdfscarafinal gazebo.launch.py"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('urdfscarafinal')
    xacro_file = os.path.join(pkg, 'urdf', 'scara_gazebo.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf', 'on_exit_shutdown': 'true'}.items())

    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'urdfscarafinal'])

    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    controllers = Node(package='controller_manager', executable='spawner', output='screen',
                       arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller'])

    return LaunchDescription([
        gazebo, rsp, clock_bridge, spawn,
        # los controladores se cargan cuando el robot ya esta en Gazebo
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers])),
    ])
