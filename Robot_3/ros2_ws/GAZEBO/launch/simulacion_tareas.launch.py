"""Celda de trabajo + tarea HOME -> PLL -> PS -> HOME.
Uso: ros2 launch urdfscarafinal simulacion_tareas.launch.py perfil:=quintico"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory('urdfscarafinal')
    xacro_file = os.path.join(pkg, 'urdf', 'scara_gazebo.urdf.xacro')
    mundo = os.path.join(pkg, 'worlds', 'celda_tareas.sdf')
    robot_description = ParameterValue(Command(['xacro ', xacro_file, ' celda:=true']), value_type=str)
    perfil = LaunchConfiguration('perfil')
    ejecutar = LaunchConfiguration('ejecutar')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 3 ' + mundo, 'on_exit_shutdown': 'true'}.items())
    rsp = Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
               parameters=[{'robot_description': robot_description, 'use_sim_time': True}])
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-topic', 'robot_description', '-name', 'urdfscarafinal'])
    puente = Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
                  arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                             '/cubo/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                             '/cubo/detach@std_msgs/msg/Empty]gz.msgs.Empty',
                             '/choques/eslabon1@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                             '/choques/eslabon2@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                             '/choques/prismatico@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'])
    controladores = Node(package='controller_manager', executable='spawner', output='screen',
                         arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller'])
    tareas = Node(package='urdfscarafinal', executable='ejecutar_tareas.py', output='screen',
                  condition=IfCondition(ejecutar),
                  parameters=[{'use_sim_time': True, 'perfil': perfil, 'espera': 3.0}])

    return LaunchDescription([
        DeclareLaunchArgument('perfil', default_value='quintico'),
        DeclareLaunchArgument('ejecutar', default_value='true'),
        gazebo, rsp, puente, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controladores])),
        RegisterEventHandler(OnProcessExit(target_action=controladores, on_exit=[tareas])),
    ])
