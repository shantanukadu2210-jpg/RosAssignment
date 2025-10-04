# Level 0: ROS 2 Turtlesim Autonomous Navigation Assignment

## Overview

This repository contains the ROS 2 package (`turtle_autonomy`) that extends the `turtlesim` simulator to implement **custom odometry**, **TF broadcasting**, and **autonomous navigation** based on the **unicycle motion model**.

The core navigation logic uses a **Dynamic Window Approach (DWA)-style** prediction, where a discrete set of future poses are evaluated, and the action (yaw rate) that minimizes the distance to the goal is selected.

## Deliverables

| Section | Node/File | Description |
| :--- | :--- | :--- |
| **1. Odometry** | `odometry_tf_node.py` | Converts `/turtle1/pose` (turtlesim/Pose) to `/turtle1/odom` (nav_msgs/Odometry). |
| **2. TF** | `odometry_tf_node.py` | Broadcasts **dynamic TF** (`odom` $\rightarrow$ `base_link`) and **static TF** (`map` $\rightarrow$ `odom` at (5.5, 5.5)). |
| **3. Autonomy** | `autonomy_node.py` | Subscribes to `/turtle1/pose` and a goal on `/goal_pose` (from RViz2). Implements the unicycle motion model for path planning. |
| **Launch File** | `assignment.launch.py` | Starts the `turtlesim`, custom nodes, and `rviz2` simultaneously. Configures **v** and **Δt** as launch parameters. |

## Instructions to Build and Run

### Prerequisites
* ROS 2 Foxy (or newer)
* Python 3 with `scipy` installed (`pip install scipy`)

### 1. Build the Package
1.  Clone the repository into your ROS 2 workspace (`ros2_ws/src`).
    ```bash
    cd ros2_ws
    # Build your package
    source /opt/ros/foxy/setup.bash
    colcon build --packages-select turtle_autonomy
    ```

### 2. Launch the System
1.  Source your local workspace.
    ```bash
    source install/setup.bash
    ```
2.  Run the launch file. The `v` (linear velocity) and `delta_t` (prediction time step) parameters can be adjusted here.
    ```bash
    ros2 launch turtle_autonomy assignment.launch.py v:=1.0 delta_t:=0.5
    ```

### 3. Test Navigation in RViz2
1.  Once launched, `turtlesim` and `rviz2` windows will open.
2.  In **RViz2**, ensure the **'Global Status'** $\rightarrow$ **'Fixed Frame'** is set to **`map`**.
3.  Use the **'2D Nav Goal'** tool (or an equivalent **PoseStamped** publisher) in RViz2 and click on the grid to set a target position for the turtle. The navigation node will then command the turtle's movement.

## Approach Summary

### Odometry & TF
* **Odometry**: Turtle's `x` and `y` position are offset by $-5.5$ to align the `odom` frame's origin with the turtle's starting point $(5.5, 5.5)$.
* **Static TF**: A static transform is broadcasted from `map` $\rightarrow$ `odom` at $(5.5, 5.5, 0.0)$, matching the `turtlesim` coordinate system to the `map` frame.

### Autonomous Planning (DWA-style)
* The `autonomy_node` uses the **Unicycle Motion Model** to predict the turtle's position at time $t + \Delta t$.
* It evaluates 11 discrete yaw rates ($\omega$) in the range $[-3.0, 3.0]$ rad/s.
* The control command published to `/turtle1/cmd_vel` is the one whose predicted end-state is the minimum Euclidean distance from the desired goal pose.
