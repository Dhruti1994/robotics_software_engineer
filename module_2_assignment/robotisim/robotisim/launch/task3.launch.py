from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    # Launch the turtlesim node
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim_node',
        output='screen',
    )

    # Spawn turtles diagonally using the `spawn` service
    spawn_turtles = [
        TimerAction(
            period=i * 1.0,  # Add a delay between each spawn
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'service', 'call', '/spawn',
                        'turtlesim/srv/Spawn',
                        f'{{x: {x}, y: {y}, theta: 0.0, name: "turtle{i + 1}"}}',
                    ],
                    output='screen',
                )
            ],
        )
        for i, (x, y) in enumerate([
            (1.0, 10.0),
            (3.0, 8.0),
            (5.5, 5.5),
            (8.0, 3.0),
            (10.0, 1.0),
        ])
    ]

    # Publish velocity commands to move the middle turtles (turtle2, turtle3, turtle4) back and forth
    drive_turtles = [
        TimerAction(
            period=5.0,  # Ensure turtles are spawned first
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'topic', 'pub', f'/turtle{i}/cmd_vel',
                        'geometry_msgs/msg/Twist',
                        '{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}',
                        '--rate', '1',
                    ],
                    output='screen',
                )
            ],
        )
        for i in [2, 3, 4]  # Middle turtles
    ]

    # Combine everything into a single launch description
    return LaunchDescription([turtlesim_node] + spawn_turtles + drive_turtles)

