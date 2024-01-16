import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lab02_pkg'),
        'config',
        'parameters.yaml'
    )
    #config = '/workspaces/vscode_ros2_workspace/src/lab02_pkg/config/parameters.yaml'
    print(f"Config file path: {config}")


    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node'
        ),
        Node(
            package='turtlesim',
            executable='turtle_teleop_key',
            name='turtle_teleop_key',
            output='screen',
            emulate_tty=True,
            prefix=["xterm -e"],
            parameters=[config],
        ),
        Node(
            package='lab02_pkg',
            executable='service',
            name='service',
        
        ),
        Node(
            package='lab02_pkg',
            executable='ActionServer',
            name='ActionServer',
            namespace='lab02',
            output='screen',
            parameters=[config]
        ),
        Node(
            package='lab02_pkg',
            executable='goal_generator',
            name='goal_generator',
            namespace='lab02',
            output='screen',
            parameters=[config]


        )
    ])
