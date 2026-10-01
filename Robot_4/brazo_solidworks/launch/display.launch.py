import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf = os.path.join(
        get_package_share_directory('brazo_solidworks'),
        'urdf',
        'ENSAMBLAJE_CON_SUBSISTEMA.SLDASM.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    rviz_config = os.path.join(
        get_package_share_directory('brazo_solidworks'),
        'urdf',
        'my_config.rviz')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config]
        ),
    ])
