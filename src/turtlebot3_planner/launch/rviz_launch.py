from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir
import os

def generate_launch_description():
    # Use PathJoinSubstitution to join paths in a launch-friendly way
    rviz_config_file = PathJoinSubstitution([ThisLaunchFileDir(), '..', 'rviz', 'default.rviz'])

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
