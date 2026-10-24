from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clean_bot',
            executable='diff_drive_node',
            parameters=[{'controller_frequency': 20.0}],
        ),
    ])
