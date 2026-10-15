import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = get_package_share_directory('Scara_Final_description')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'Scara_Final.urdf')
    controllers_file = os.path.join(pkg_share, 'config', 'controllers.yaml')

    # Lectura del URDF y reemplazo dinámico de la ruta
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_desc = robot_desc.replace(
        '$(find Scara_Final_description)/config/controllers.yaml',
        controllers_file
    )

    robot_description = {'robot_description': robot_desc}

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'scara'],
        output='screen'
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    scara_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['scara_controller'],
        output='screen'
    )

    return LaunchDescription([
        gz_sim,
        clock_bridge,
        robot_state_publisher,
        spawn_entity,
        TimerAction(period=3.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=5.0, actions=[scara_controller_spawner]),
    ])