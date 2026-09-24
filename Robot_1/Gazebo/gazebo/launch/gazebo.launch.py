import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = get_package_share_directory('rviz_gazebo')

    xacro_file = os.path.join(
        pkg_share,
        'urdf',
        'rviz.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false'
    )

    resource_path = os.path.dirname(pkg_share)

    set_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf'
        }.items()
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ]
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name',
            'rviz_robot',
            '-topic',
            'robot_description'
        ],
        output='screen'
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        output='screen'
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager'
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[
            {
                'use_sim_time': True
            }
        ],
        condition=IfCondition(
            LaunchConfiguration('use_rviz')
        )
    )

    return LaunchDescription([
        use_rviz,
        set_resource_path,
        gazebo,
        robot_state_publisher,
        clock_bridge,
        spawn_robot,

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_robot,
                on_exit=[
                    joint_state_broadcaster
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_state_broadcaster,
                on_exit=[
                    joint_trajectory_controller
                ]
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=joint_trajectory_controller,
                on_exit=[
                    rviz
                ]
            )
        ),
    ])

