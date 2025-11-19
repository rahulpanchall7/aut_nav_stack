import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory
import yaml
import os
from PIL import Image
import numpy as np

from turtlebot3_planner.path_generator import generate_collision_aware_path

def load_map_from_yaml(yaml_file):
    """
    Load a 2D occupancy grid from a ROS2 map.yaml + map.pgm.

    Args:
        yaml_file (str): Path to the map.yaml file.

    Returns:
        occupancy_grid (np.ndarray): 2D array, 0=free, 1=occupied
        resolution (float): meters per cell
        origin (tuple): (x, y) origin of the map in world coordinates
    """
    with open(yaml_file, 'r') as f:
        map_data = yaml.safe_load(f)

    resolution = map_data['resolution']
    origin = tuple(map_data['origin'][:2])  # x, y

    # Get full path to PGM file
    pgm_file = os.path.join(os.path.dirname(yaml_file), map_data['image'])

    # Load PGM image and convert to numpy array
    img = Image.open(pgm_file)
    img = np.array(img)

    # Convert to occupancy grid: 0=free, 1=occupied
    occupancy_grid = np.zeros_like(img, dtype=np.int8)
    occupancy_grid[img < 127] = 1  # treat dark pixels as obstacles

    return occupancy_grid, resolution, origin


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Publisher
        self.path_pub = self.create_publisher(Path, '/discrete_path', 10)

        # Store start/goal poses
        self.start_pose = None
        self.goal_pose = None

        # Load static map
        map_yaml_path = os.path.join(get_package_share_directory('turtlebot3_planner'), 'maps', 'map.yaml')
        self.occupancy_grid, self.resolution, self.origin = load_map_from_yaml(map_yaml_path)
        self.get_logger().info(f"Loaded map: resolution={self.resolution}, origin={self.origin}")

    def initial_pose_callback(self, msg):
        # PoseWithCovarianceStamped has msg.pose.pose
        self.start_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f"Received initial pose: x={self.start_pose[0]:.2f}, y={self.start_pose[1]:.2f}")
        self.try_generate_path()

    def goal_pose_callback(self, msg):
        self.goal_pose = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"Received goal pose: x={self.goal_pose[0]:.2f}, y={self.goal_pose[1]:.2f}")
        self.try_generate_path()

    def try_generate_path(self):
        if self.start_pose and self.goal_pose:
            self.get_logger().info("Both start and goal poses are set. Generating path...")
            waypoints = generate_collision_aware_path(
                self.start_pose,
                self.goal_pose,
                self.occupancy_grid,
                self.resolution,
                self.origin
            )
            if not waypoints:
                self.get_logger().warn('No path found!')
                return

            path_msg = Path()
            path_msg.header.frame_id = "map"
            path_msg.header.stamp = self.get_clock().now().to_msg()

            for idx, (x, y) in enumerate(waypoints):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)
                self.get_logger().debug(f"Waypoint {idx}: x={x:.2f}, y={y:.2f}")

            self.path_pub.publish(path_msg)
            self.get_logger().info(f'Published collision-aware path with {len(waypoints)} waypoints')

        else:
            if not self.start_pose:
                self.get_logger().info("Waiting for initial pose...")
            if not self.goal_pose:
                self.get_logger().info("Waiting for goal pose...")

def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    node.get_logger().info("Planner node started. Waiting for initial and goal poses...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
