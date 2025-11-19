import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from ament_index_python.packages import get_package_share_directory
import yaml
from PIL import Image
import numpy as np

from turtlebot3_planner.path_generator import generate_collision_aware_path
from turtlebot3_planner.path_smoother import smooth_path
from turtlebot3_planner.path_smoother import compute_orientations

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
        # num of waypoint arguments
        self.declare_parameter('num_discrete_waypoints', 50)  # default 50
        self.declare_parameter('num_smooth_waypoints', 200)   # default 200

        # Subscribers
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        # Subscribe to current robot pose for dynamic re-planning
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.current_pose_callback, 10)

        # Initialize last plan timestamp for rate-limiting
        self.last_plan_time = self.get_clock().now()


        # Publisher
        self.discrete_path_pub = self.create_publisher(Path, '/discrete_path', 10)
        self.smooth_path_pub = self.create_publisher(Path, '/smooth_path', 10)

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

    def current_pose_callback(self, msg):
        current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self.start_pose:
            dx = current_pose[0] - self.start_pose[0]
            dy = current_pose[1] - self.start_pose[1]
            distance_moved = (dx**2 + dy**2)**0.5

            # Re-plan if robot moved more than 10 cm and 1 second has passed
            now = self.get_clock().now()
            if distance_moved > 0.1 and (now - self.last_plan_time).nanoseconds > 1e9:
                self.start_pose = current_pose
                self.last_plan_time = now
                self.get_logger().info(f"Robot moved {distance_moved:.2f} m. Replanning path...")
                self.try_generate_path()


    def try_generate_path(self):

        # Get parameters
        num_discrete = self.get_parameter('num_discrete_waypoints').get_parameter_value().integer_value
        num_smooth = self.get_parameter('num_smooth_waypoints').get_parameter_value().integer_value

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

            # Raw waypoints (collision-aware)
            discrete_waypoints = waypoints  # original A* path

            # Resample discrete path if user requested fewer waypoints
            if num_discrete < len(discrete_waypoints):
                indices = np.linspace(0, len(discrete_waypoints)-1, num_discrete, dtype=int)
                discrete_waypoints = [discrete_waypoints[i] for i in indices]

            # Smoothed path
            smooth_waypoints = smooth_path(discrete_waypoints, num_points=num_smooth)

            # publish discrete path
            discrete_path_msg = Path()
            discrete_path_msg.header.frame_id = "map"
            discrete_path_msg.header.stamp = self.get_clock().now().to_msg()

            # Compute orientations along the discrete path
            discrete_waypoints_with_theta = compute_orientations(discrete_waypoints)

            for idx, (x, y, theta) in enumerate(discrete_waypoints_with_theta):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                # Convert theta to quaternion
                pose.pose.orientation.z = np.sin(theta/2)
                pose.pose.orientation.w = np.cos(theta/2)
                discrete_path_msg.poses.append(pose)
                self.get_logger().debug(f"Waypoint {idx}: x={x:.2f}, y={y:.2f}, theta={theta:.2f} rad")


            self.discrete_path_pub.publish(discrete_path_msg)

            # publish smooth path
            smooth_path_msg = Path()
            smooth_path_msg.header.frame_id = "map"
            smooth_path_msg.header.stamp = self.get_clock().now().to_msg()

            # Compute orientations along the smoothed path
            smooth_waypoints_with_theta = compute_orientations(smooth_waypoints)

            for idx, (x, y, theta) in enumerate(smooth_waypoints_with_theta):
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = x
                pose.pose.position.y = y
                # Convert theta to quaternion
                pose.pose.orientation.z = np.sin(theta/2)
                pose.pose.orientation.w = np.cos(theta/2)
                smooth_path_msg.poses.append(pose)
                self.get_logger().debug(f"Waypoint {idx}: x={x:.2f}, y={y:.2f}, theta={theta:.2f} rad")



            self.smooth_path_pub.publish(smooth_path_msg)

            self.get_logger().info(f"Published raw path ({len(discrete_waypoints)} waypoints) and smoothed path ({len(smooth_waypoints)} waypoints)")

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
