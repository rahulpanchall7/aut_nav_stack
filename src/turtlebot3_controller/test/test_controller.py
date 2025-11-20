import rclpy
from rclpy.node import Node
import pytest
import time
import numpy as np
from geometry_msgs.msg import Twist
from turtlebot3_msgs.msg import Trajectory, TrajectoryPoint
from turtlebot3_controller.pure_pursuit_controller import PurePursuitController


# ========================= Fixture ========================= #
@pytest.fixture
def rclpy_test_node():
    rclpy.init()
    node = PurePursuitController()
    # Disable obstacle stopping and reduce lookahead distance for tests
    node.min_obstacle_distance = 0.0
    node.scan_ranges = np.array([])
    node.lookahead_distance = 0.01
    yield node
    node.destroy_node()
    rclpy.shutdown()


# ========================= Test 1: Trajectory Publisher ========================= #
def test_publish_trajectory(rclpy_test_node):
    node = rclpy_test_node

    # Create simple straight trajectory
    traj = Trajectory()
    for i in range(5):
        p = TrajectoryPoint()
        p.x = i * 0.1
        p.y = 0.0
        p.t = i * 0.5
        traj.points.append(p)

    node.trajectory_callback(traj)

    # Spin node and simulate odometry along the path
    start = time.time()
    while node.state != "GOAL_REACHED" and (time.time() - start) < 2.0:
        if node.trajectory and node.current_index < len(node.trajectory):
            # Move robot slightly past the current trajectory point
            target = node.trajectory[node.current_index]
            node.x = target[0] + 0.01
            node.y = target[1] + 0.01
        rclpy.spin_once(node, timeout_sec=0.05)

    assert node.state == "GOAL_REACHED"
    assert node.trajectory == []


# ========================= Test 2: Obstacle Avoidance ========================= #
def test_obstacle_avoidance_cmd():
    rclpy.init()
    node = PurePursuitController()
    node.min_obstacle_distance = 0.2  # enable obstacle stopping

    # Simple trajectory
    traj = Trajectory()
    for i in range(3):
        p = TrajectoryPoint()
        p.x = i * 0.1
        p.y = 0.0
        p.t = i * 0.5
        traj.points.append(p)
    node.trajectory_callback(traj)

    # Inject obstacle very close
    node.scan_ranges = np.array([0.1] * 360)
    node.control_loop()

    # Ensure linear velocity is zero due to obstacle
    last_cmd = None
    if hasattr(node.cmd_pub, '_msg_queue') and node.cmd_pub._msg_queue:
        last_cmd = node.cmd_pub._msg_queue[-1]
    assert last_cmd is None or last_cmd.linear.x == 0.0

    node.destroy_node()
    rclpy.shutdown()


# ========================= Test 3: Pure Pursuit Following ========================= #
def test_trajectory_following(rclpy_test_node):
    node = rclpy_test_node

    # Straight trajectory
    traj = Trajectory()
    for i in range(5):
        p = TrajectoryPoint()
        p.x = i * 0.1
        p.y = 0.0
        p.t = i * 0.5
        traj.points.append(p)

    node.trajectory_callback(traj)

    # Simulate robot starting at origin
    node.x = 0.0
    node.y = 0.0
    node.theta = 0.0

    # Spin node and simulate robot following trajectory
    start = time.time()
    while node.state != "GOAL_REACHED" and (time.time() - start) < 2.0:
        if node.trajectory and node.current_index < len(node.trajectory):
            target = node.trajectory[node.current_index]
            node.x = target[0]
            node.y = target[1]
        rclpy.spin_once(node, timeout_sec=0.05)

    assert node.state == "GOAL_REACHED"
    assert len(node.actual_x) > 0
    assert len(node.actual_y) > 0
