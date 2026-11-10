from pathlib import Path
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():

    ws = Path.home() / "robot2_ws"
    urdf = ws / "robot2_giro_135_limite_prueba.urdf"

    if not urdf.exists():
        raise FileNotFoundError(f"No existe el URDF: {urdf}")

    robot_description = urdf.read_text()

    gazebo_env = os.environ.copy()
    resource_path = str(ws / "install" / "robot2" / "share")
    previous_path = gazebo_env.get("GZ_SIM_RESOURCE_PATH", "")

    gazebo_env["GZ_SIM_RESOURCE_PATH"] = (
        resource_path + (":" + previous_path if previous_path else "")
    )

    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-v", "4", "empty.sdf"],
        output="screen",
        additional_env={
            "GZ_SIM_RESOURCE_PATH": gazebo_env["GZ_SIM_RESOURCE_PATH"]
        },
    )

    publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description}
        ],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/empty/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"
        ],
        remappings=[
            ("/world/empty/clock", "/clock")
        ],
        output="screen",
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-world", "empty",
            "-name", "robot2_giro_135_limite",
            "-file", str(urdf),
            "-z", "0.05",
        ],
        output="screen",
    )

    broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    home = ExecuteProcess(
        cmd=[
            "bash",
            str(ws / "scripts" / "ir_home.sh"),
        ],
        output="screen",
    )

    return LaunchDescription([
        gazebo,
        publisher,
        clock_bridge,

        TimerAction(
            period=5.0,
            actions=[spawn],
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn,
                on_exit=[broadcaster],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=broadcaster,
                on_exit=[controller],
            )
        ),

        RegisterEventHandler(
            OnProcessExit(
                target_action=controller,
                on_exit=[home],
            )
        ),
    ])
