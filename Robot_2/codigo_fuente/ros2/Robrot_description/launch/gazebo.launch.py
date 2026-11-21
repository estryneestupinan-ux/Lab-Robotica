"""
Robrot SCARA RRP en Gazebo (gz sim) con gz_ros2_control.

Uso:
    ros2 launch Robrot_description gazebo.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, IncludeLaunchDescription,
                            RegisterEventHandler, TimerAction)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('Robrot_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'Robrot.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'gz_controllers.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' sim:=true']), value_type=str)

    # Para que Gazebo encuentre las mallas package://Robrot_description/...
    resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg_dir))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r -v 3 empty.sdf', 'on_exit_shutdown': 'true'}.items(),
    )

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen')

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'scara', '-z', '0.0'])

    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])

    jsb = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager',
                   '--param-file', controllers_file, '--controller-manager-timeout', '60'])

    vel = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['velocity_controller', '--controller-manager', '/controller_manager',
                   '--param-file', controllers_file, '--controller-manager-timeout', '60'])

    return LaunchDescription([
        resource_path,
        gazebo,
        rsp,
        clock_bridge,
        spawn,
        # Los controladores se cargan cuando el robot ya existe en Gazebo
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[vel])),
    ])
