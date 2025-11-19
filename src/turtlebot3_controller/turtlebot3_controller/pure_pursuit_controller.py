import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan  # For dynamic obstacle detection
from turtlebot3_msgs.msg import Trajectory, TrajectoryPoint  # adjust import as per your package
import math
import numpy as np
import matplotlib.pyplot as plt  # Added for plotting

class PurePursuitController(Node):
    def __init__(self):
        super().__init__('pure_pursuit_controller')

        # Subscribers
        self.create_subscription(Trajectory, '/trajectory', self.trajectory_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # New subscriber for dynamic obstacles
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Trajectory
        self.trajectory = []
        self.current_index = 0

        # Pure Pursuit parameters
        self.lookahead_distance = 0.3  # meters
        self.max_linear_vel = 0.22     # TurtleBot3 max
        self.max_angular_vel = 2.84    # TurtleBot3 max
        self.dt = 0.1                  # control loop dt

        # Obstacle avoidance parameters
        self.min_obstacle_distance = 0.25  # meters, stop if closer
        self.scan_ranges = []  # store latest lidar scan

        # --- Added for trajectory logging ---
        self.actual_x = []
        self.actual_y = []
        self.time_stamps = []
        # ------------------------------------

        # Control loop timer
        self.timer = self.create_timer(self.dt, self.control_loop)

    def odom_callback(self, msg):
        # Update robot pose
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        # Convert quaternion to yaw
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def trajectory_callback(self, msg):
        # Store trajectory points: (x, y, t)
        self.trajectory = [(p.x, p.y, p.t) for p in msg.points]
        self.current_index = 0
        self.get_logger().info(f"Received trajectory with {len(self.trajectory)} points")

    def scan_callback(self, msg):
        # Store latest LIDAR scan ranges
        self.scan_ranges = np.array(msg.ranges)
        # Replace inf values with max range for safety
        self.scan_ranges[self.scan_ranges > msg.range_max] = msg.range_max

    def control_loop(self):
        if not self.trajectory or self.current_index >= len(self.trajectory):
            # Stop robot if trajectory finished
            self.cmd_pub.publish(Twist())
            return

        # --- Record actual positions for plotting ---
        self.actual_x.append(self.x)
        self.actual_y.append(self.y)
        self.time_stamps.append(self.get_clock().now().nanoseconds * 1e-9)  # convert to seconds
        # -------------------------------------------

        # --- Pure Pursuit: Find lookahead goal point ---
        goal_point = None
        for i in range(self.current_index, len(self.trajectory)):
            dx = self.trajectory[i][0] - self.x
            dy = self.trajectory[i][1] - self.y
            dist = math.hypot(dx, dy)
            if dist >= self.lookahead_distance:
                goal_point = self.trajectory[i]
                self.current_index = i
                break

        if goal_point is None:
            goal_point = self.trajectory[-1]

        x_g, y_g, t_g = goal_point

        # Transform goal point to robot frame
        dx = x_g - self.x
        dy = y_g - self.y
        x_r = math.cos(-self.theta) * dx - math.sin(-self.theta) * dy
        y_r = math.sin(-self.theta) * dx + math.cos(-self.theta) * dy

        # Pure Pursuit curvature
        if x_r == 0:
            curvature = 0.0
        else:
            curvature = 2 * y_r / (self.lookahead_distance ** 2)

        # --- Compute linear velocity based on trajectory timestamps ---
        if self.current_index > 0:
            x_prev, y_prev, t_prev = self.trajectory[self.current_index - 1]
            segment_length = math.hypot(x_g - x_prev, y_g - y_prev)
            dt_segment = t_g - t_prev
            if dt_segment > 0:
                v = min(segment_length / dt_segment, self.max_linear_vel)
            else:
                v = self.max_linear_vel
        else:
            v = self.max_linear_vel

        # --- Dynamic Obstacle Avoidance Layer ---
        if len(self.scan_ranges) > 0:
            # Consider only front +/- 30 degrees
            front_angle = 30  # degrees
            n = len(self.scan_ranges)
            indices = np.arange(n//2 - n*front_angle//360, n//2 + n*front_angle//360, dtype=int)
            front_distances = self.scan_ranges[indices]
            min_dist = np.min(front_distances)

            if min_dist < self.min_obstacle_distance:
                # Obstacle too close → stop
                self.get_logger().warn(f"Obstacle detected at {min_dist:.2f} m! Stopping.")
                v = 0.0

        # Compute angular velocity
        omega = curvature * v
        omega = max(min(omega, self.max_angular_vel), -self.max_angular_vel)

        # Publish cmd_vel
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.cmd_pub.publish(cmd)

        # Stop if we are very close to the last point
        x_last, y_last, _ = self.trajectory[-1]
        if math.hypot(x_last - self.x, y_last - self.y) < 0.05:
            self.cmd_pub.publish(Twist())
            self.get_logger().info("Reached final waypoint")

            # --- Plot planned vs actual path ---
            planned_x = [p[0] for p in self.trajectory]
            planned_y = [p[1] for p in self.trajectory]

            plt.figure()
            plt.plot(planned_x, planned_y, 'r--', label='Planned Path')
            plt.plot(self.actual_x, self.actual_y, 'b-', label='Actual Path')
            plt.scatter(planned_x[-1], planned_y[-1], c='g', marker='*', label='Goal')
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title('Planned vs Actual Path')
            plt.legend()
            plt.show()
            # -----------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PurePursuitController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
