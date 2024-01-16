from launch.launch_description import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    RegisterEventHandler,
    DeclareLaunchArgument,
    EmitEvent,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch.conditions import IfCondition

from launch.substitutions import (
    ThisLaunchFileDir,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bag_arg = DeclareLaunchArgument("bag", default_value="")
    bag = LaunchConfiguration("bag")

    config = PathJoinSubstitution([FindPackageShare("lab05_pkg"), "config", "ekf.yaml"])

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="filter",
        output="screen",
        parameters=[config],
        remappings=[("cmd_vel", "/diff_drive_controller/cmd_vel_unstamped")],
    )

    play_bag = ExecuteProcess(
        cmd=["ros2", "bag", "play", bag],
        output="screen",
        condition=IfCondition(PythonExpression(["not '", bag, "' == ", "''"])),
    )

    shutdown_on_bag_finish = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=play_bag, on_exit=[EmitEvent(event=Shutdown())])
    )

    start_filter_after_rosbag = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=play_bag, on_start=[TimerAction(period=2.0, actions=[ekf])]
        )
    )

    return LaunchDescription([bag_arg, play_bag, start_filter_after_rosbag, shutdown_on_bag_finish])
