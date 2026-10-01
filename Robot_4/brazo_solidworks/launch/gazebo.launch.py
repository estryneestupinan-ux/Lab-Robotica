from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("brazo_solidworks")
    urdf_path = os.path.join(pkg_share, "urdf", "ENSAMBLAJE_CON_SUBSISTEMA.SLDASM.urdf")
    controllers_yaml = os.path.join(pkg_share, "config", "controllers.yaml")
    resource_path = os.path.dirname(pkg_share)

    with open(urdf_path, "r") as infp:
        robot_desc = infp.read()

    set_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=resource_path
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": "empty.sdf -r --physics-engine gz-physics-bullet-featherstone-plugin"}.items()
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}]
    )

    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "brazo_r4", "-z", "0.1"],
        output="screen"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    joint1_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint1_position_controller", "--param-file", controllers_yaml],
        output="screen"
    )

    joint2_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint2_position_controller", "--param-file", controllers_yaml],
        output="screen"
    )

    joint3_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint3_velocity_controller", "--param-file", controllers_yaml],
        output="screen"
    )

    delay_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner]
        )
    )

    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint1_spawner, joint2_spawner, joint3_spawner]
        )
    )

    return LaunchDescription([
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        delay_broadcaster,
        delay_controllers
    ])
