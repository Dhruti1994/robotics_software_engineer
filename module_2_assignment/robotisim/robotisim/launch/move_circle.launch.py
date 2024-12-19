from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen'
        ),
        Node(
            package='robotisim',
            executable='move_circle',
            name='move_circle',
            output='screen'
        )
    ])

