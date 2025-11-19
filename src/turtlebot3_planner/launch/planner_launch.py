from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir

def generate_launch_description():
    # Use PathJoinSubstitution to join paths in a launch-friendly way
    rviz_config_file = PathJoinSubstitution([ThisLaunchFileDir(), '..', 'rviz', 'default.rviz'])
    return LaunchDescription([
        Node(
            package='turtlebot3_planner',
            executable='planner_node',
            name='planner_node',
            # Pass parameters for number of waypoints
            parameters=[{
                'num_discrete_waypoints': 50,  
                'num_smooth_waypoints': 500 
            }]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            name='static_tf_pub'
        )
    ])
