"""demo_robot launch file (the planted problem is the extra parameter)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level', default_value='info'),
        Node(
            package='demo_robot',
            executable='diff_drive_node',
            name='diff_drive_node',
            parameters=[{
                'wheel_diameter_mm': 101.6,
                'controller_frequency': 50.0,
                'publish_tf': False,
            }],
            output='screen',
        ),
    ])
