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
- TurtleBot3
    - Required for the robot model, cmd_vel, sensors, and robot dimensions.
    ```bash
    sudo apt install ros-humble-turtlebot3*
    ```
    - Set the model in your `.bashrc`
    ```bash
    export TURTLEBOT3_MODEL=waffle
    ```
- SLAM Toolbox
    - For generating `/map` if you need mapping.
    ```bash
    sudo apt install ros-humble-slam-toolbox
    ```
- Nav2 / AMCL
    - For localization (`/amcl_pose topic`).
    ```bash
    sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup
    ```
- rosdep (automatic)
    - To make sure all ROS2 dependencies in `package.xml` are installed:
    ```bash
    cd ~/ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```
- Python Dependencies
    - Python libraries used in the nodes:    
    ```bash
    pip install numpy pillow scipy
    ```
- Extras
    - If any node imports fail, install the missing package as prompted.
    - Build and source ROS and workspace everytime when you open a new terminal


## **2. Clone Repository**
- git clone
    ```bash
    git clone https://github.com/rahulpanchall7/rahul_panchal_10x_task.git
    ```
- rosdep (automatic)
    ```bash
    cd ~/rahul_panchal_10x_task
    rosdep update
    rosdep install --from-paths src --ignore-src -y
    ```

## **3. Build & Source Workspace**
-   ```bash
    colcon build 
    source install/setup.bash
    ```

## **4. Running the Full Simulation (Step-by-Step Guide)**
This section explains exactly how to start the simulation, launch mapping/localization, run the planner, and run the pure pursuit controller.

**4.1. Launch Gazebo Simulation (TurtleBot3 Waffle)**
- Start the turtlebot3 world and spawn the robot
    ```bash
     ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
     ```
**4.2. Generating a Map (Using SLAM Toolbox) (optional if map files not generated, but I have provided for use)**
- Launch SLAM Toolbox (Synchronous Mode), `online_sync_launch.py` ensures stable real-time SLAM while teleoperating the robot (new terminal):
    ```bash
    ros2 launch slam_toolbox online_sync_launch.py
    ```
- Teleoperate the TurtleBot3, to run keyboard teleoperation, drive the robot around the environment so SLAM can properly detect walls and build a consistent map (new terminal):
    ```bash
    ros2 run turtlebot3_teleop teleop_keyboard
    ```
    `Tip:` Move slowly and avoid sudden rotations for the cleanest map.
- Save the Map, once the environment is fully mapped, save it using Nav2’s map saver tool (new terminal):
    ```bash
    ros2 launch slam_toolbox online_sync_launch.py
    ```
    This will produce:
      - `map.yaml`
      - `map.pgm` or `map.png`
    save them inside the `/maps` directory inside the `turtlebot3_planner` package. For quick implementation I have already provided these for the `turtlebot3_world` world.

**4.3. Repeat 4.1**

**4.4 Launching the Planner Node + Static Transform**
- Once the map has been generated and saved, the next step is to start the planner node and ensure the robot has the correct coordinate frame relationships to operate in the map (new terminal):
    ```bash
    ros2 launch turtlebot3_planner planner_launch.py
    ```
- This starts up the node and it would wait for the `/initialpose` and `/goal_pose` (to be given through Rviz later)

**4.5 Launching AMCL for Localisation (Dynamic Replanning Support)**
- To enable real-time localization and dynamic replanning, we launch the AMCL (Adaptive Monte-Carlo Localization) node using a dedicated launch file (new terminal):
    ```bash
    ros2 launch turtlebot3_planner amcl_launch.py
    ```
- This node too waits for the `/initialpose`

**4.6 Running the Trajectory Generator and Pure Pursuit Controller**
- Trajectory Generation Node, converts the smoothed global path into a time-parameterized trajectory (new terminal):
    ```bash
    ros2 run turtlebot3_planner trajectory_generator_node
    ```
- Pure Pursuit Controller Node, performs the actual driving of the TurtleBot3 including dynamic obstacle avoidance (new terminal):
    ```bash
    ros2 run turtlebot3_controller pure_pursuit_controller
    ```
- Dependent on the Planner Node

**4.7 Launch Rviz**
- (new terminal)
    ```bash
    ros2 launch turtlebot3_planner rviz_launch.py
    ```
- Set the initial and goal pose using the `2D Pose Estimate` and `2D Nav Goal` tools and the robot performs autonomous motion along the given trajectory


























