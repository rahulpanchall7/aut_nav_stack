import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from turtlebot3_msgs.msg import Trajectory, TrajectoryPoint

class TrajectoryGeneratorNode(Node):
    def __init__(self):
        super().__init__('trajectory_generator_node')

        # Subscriber to the smoothed path
        self.create_subscription(Path, '/smooth_path', self.smooth_path_callback, 10)

        # Publisher for the time-parameterized trajectory
        self.trajectory_pub = self.create_publisher(Trajectory, '/trajectory', 10)

        # Desired velocity (m/s)
        self.max_velocity = 0.2
        # Acceleration (m/s^2) for trapezoidal profile
        self.acceleration = 0.1
        # Sampling interval (seconds)
        self.dt = 0.1

    def smooth_path_callback(self, msg: Path):
        # Extract waypoints from Path message
        waypoints = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if not waypoints:
            self.get_logger().warn("Received empty smooth path!")
            return

        # Generate trajectory with regular time intervals and trapezoidal velocity
        trajectory = self.generate_trajectory(waypoints, self.max_velocity, self.acceleration, self.dt)

        traj_msg = Trajectory()
        for x, y, t in trajectory:
            point = TrajectoryPoint()
            point.x = x
            point.y = y
            point.t = t
            traj_msg.points.append(point)

        self.trajectory_pub.publish(traj_msg)
        self.get_logger().info(f"Published trajectory with {len(trajectory)} points")

    def generate_trajectory(self, waypoints, v_max, a, dt):
        """
        Generate time-parameterized trajectory with trapezoidal velocity profile.

        Args:
            waypoints (list of tuples): [(x0, y0), ...]
            v_max (float): maximum velocity (m/s)
            a (float): acceleration/deceleration (m/s^2)
            dt (float): sampling interval (s)

        Returns:
            trajectory (list of tuples): [(x, y, t), ...] at regular time intervals
        """
        waypoints = np.array(waypoints)
        # Compute cumulative distances along the path
        distances = np.zeros(len(waypoints))
        distances[1:] = np.cumsum(np.linalg.norm(np.diff(waypoints, axis=0), axis=1))
        total_length = distances[-1]

        # Compute trapezoidal profile parameters
        t_accel = v_max / a
        d_accel = 0.5 * a * t_accel**2
        if 2 * d_accel > total_length:
            # Short path, triangle profile
            t_accel = np.sqrt(total_length / a)
            t_flat = 0
            v_peak = a * t_accel
        else:
            t_flat = (total_length - 2*d_accel) / v_max
            v_peak = v_max

        # Generate time array
        total_time = 2*t_accel + t_flat
        t_values = np.arange(0, total_time + dt, dt)
        s_values = np.zeros_like(t_values)

        # Compute distance along path for each time
        for i, t in enumerate(t_values):
            if t < t_accel:  # acceleration phase
                s_values[i] = 0.5 * a * t**2
            elif t < t_accel + t_flat:  # constant velocity
                s_values[i] = d_accel + v_peak * (t - t_accel)
            else:  # deceleration phase
                t_dec = t - t_accel - t_flat
                s_values[i] = d_accel + v_peak * t_flat + v_peak*t_dec - 0.5*a*t_dec**2

        # Interpolate x, y along the path for each s_value
        x_values = np.interp(s_values, distances, waypoints[:,0])
        y_values = np.interp(s_values, distances, waypoints[:,1])

        trajectory = list(zip(x_values, y_values, t_values))
        return trajectory


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryGeneratorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
