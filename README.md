# Path Smoothing and Trajectory Control in 2D Space

> A ROS 2 Humble-based simulation project for generating smooth, obstacle-aware trajectories and tracking them using a differential drive robot (TurtleBot3) in Gazebo.

---

## 🚀 Overview

This project demonstrates:
- Path smoothing using **B-spline** and **Cubic Spline** techniques
- Obstacle-aware trajectory filtering
- Time-parameterized trajectory generation
- Trajectory tracking using a **Pure Pursuit Controller**
- Real-time RViz visualization for path, robot position, and obstacles

All computations run in **real-time simulation** using `turtlebot3_gazebo` in an empty world.

---

## 🧠 Key Features

- 🔄 Smooth path generation with selectable spline type (`bspline` or `cubic`)
- 🧱 Dynamic obstacle avoidance (basic filtering)
- 🐢 Integration with `turtlebot3_gazebo`
- 🧭 Real-time marker updates in RViz (path, robot pose, obstacles)
- 🔧 Param-driven launch configuration (`spline_type`, `start`)

---

## 🛠️ Dependencies

Make sure you have the following installed:

- ROS 2 Humble
- `turtlebot3_gazebo` package
- `rclpy`, `nav_msgs`, `geometry_msgs`, `visualization_msgs`, etc.
- Python ≥ 3.8

---

## 📦 Folder Structure

path_smoothing_control/
├── launch/
│ └── simulation.launch.py
├── path_smoothing_control/
│ ├── main_node.py # ROS 2 Node: Path smoothing + control
│ ├── controller.py # Pure pursuit tracking controller
│ ├── visualizer.py # RViz marker generators
│ ├── obstacle_utils.py # Path filtering based on obstacles
│ ├── smooth_bsplines.py # B-spline smoothing
│ ├── smooth_cubic.py # Cubic spline smoothing
│ ├── trajectory_gen.py # Time-parameterized trajectory
│ └── compare_splines.py # Script to visualize both spline types
├── package.xml
├── setup.py
└── setup.cfg


---

## 🔧 Build Instructions

> Clone inside your ROS 2 workspace `src/` folder:


cd ~/turtlebot3_ws/src
git clone https://github.com/abdu7rahman/Path-Smoothing-and-Trajectory-Control-in-2D-Space.git
cd ..
colcon build --packages-select path_smoothing_control
source install/setup.bash

🚀 How to Run

    🐢 Launch TurtleBot3 in Gazebo:

ros2 launch turtlebot3_gazebo empty_world.launch.py

    🧠 Launch the simulation node:

ros2 launch path_smoothing_control simulation.launch.py spline_type:=bspline

    ✅ Start the controller:

ros2 param set /trajectory_follower start true

🧪 Test Both Spline Types

ros2 launch path_smoothing_control simulation.launch.py spline_type:=cubic
ros2 launch path_smoothing_control simulation.launch.py spline_type:=bspline

💡 Future Enhancements

    Smarter obstacle-aware path generation (A*, RRT*, or optimization-based)

    Live obstacle detection using sensor input

    GUI slider for real-time spline tuning

    Comparison plots (position error, velocity profiles)
