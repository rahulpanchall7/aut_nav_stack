# TurtleBot3 Autonomous Navigation Stack

This project implements a complete navigation stack using TurtleBot3 (Waffle) in ROS2. It includes:

- **Path Planning**: A* based path planner with obstacle inflation.
- **Path Smoothing**: Cubic spline interpolation of discrete waypoints.
- **Trajectory Generation**: Time-parameterized trajectory with trapezoidal velocity profile.
- **Pure Pursuit Controller**: Provides velocity commands, and follows the trajectory.
- **Dynamic Replanning**: Robot can replan path if it moves from previous position.
- **Obstacle Inflation**: Static obstacles are inflated for safe navigation.
- **RViz Visualization**: View scans, trajectories, and maps.

---

## **1. Prerequisites**

- Ubuntu 22.04
- ROS 2 Humble Hawksbill
- TurtleBot3:
    - Required for the robot model, cmd_vel, sensors, and robot dimensions.
    ```bash
        sudo apt install ros-humble-turtlebot3*
    ```
    - Set the model in your .bashrc
    ```bash
        export TURTLEBOT3_MODEL=waffle
    ```
- Python 3 packages:
    ```bash
        pip install numpy pillow scipy
    ```