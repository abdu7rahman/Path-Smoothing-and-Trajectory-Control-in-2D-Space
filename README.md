
# 📌 Path Smoothing and Trajectory Control in 2D Space

> A full-featured ROS 2 Humble simulation project to generate and follow smooth, obstacle-aware paths using a differential-drive robot in Gazebo.

---

## 🎯 Project Objective

This project simulates a **2D navigation pipeline** where:
- A predefined set of waypoints are **smoothed into a continuous trajectory**
- Basic obstacles are placed in the world
- The robot intelligently adjusts the path to avoid them
- A **pure pursuit controller** allows the robot to follow the generated trajectory in real-time

Ideal for robotics learners & researchers interested in **trajectory generation**, **spline interpolation**, **real-time robot control**, and **ROS 2 visualization**.

---

## ✨ Features

- ✅ **Path Smoothing Algorithms**
  - B-spline
  - Cubic spline
- 🚧 **Obstacle-Aware Path Filtering**
- 🛣️ **Time-Parameterized Trajectories**
- 🤖 **Pure Pursuit Controller** for tracking
- 🧠 **RViz Visualization** of:
  - Smoothed path
  - Obstacle markers
  - Live robot position
- 🔧 ROS 2 launch parameters (`spline_type`, `start`)
- 🧪 Spline comparison utility
- 🧰 Clean modular code

---

## 🧱 Architecture

```
TurtleBot3 (Gazebo) → Odometry → TrajectoryFollower → CmdVel → Robot
                               ↘ Markers for path, robot, obstacles → RViz
```

---

## 🧰 Dependencies

Ensure you have the following:

- ROS 2 Humble
- `turtlebot3_gazebo`
- Python 3.8+
- Common ROS 2 Python message types: `geometry_msgs`, `nav_msgs`, `visualization_msgs`

Install dependencies:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo python3-colcon-common-extensions
```

---

## 📁 Folder Structure

```
path_smoothing_control/
├── launch/
│   └── simulation.launch.py
├── path_smoothing_control/
│   ├── main_node.py
│   ├── controller.py
│   ├── visualizer.py
│   ├── obstacle_utils.py
│   ├── smooth_bsplines.py
│   ├── smooth_cubic.py
│   ├── trajectory_gen.py
│   ├── compare_splines.py
├── package.xml
├── setup.py
├── setup.cfg
```

---

## 🛠️ Build Instructions

### 1. Clone and build the package

```bash
cd ~/turtlebot3_ws/src
git clone https://github.com/abdu7rahman/Path-Smoothing-and-Trajectory-Control-in-2D-Space.git
cd ..
colcon build --packages-select path_smoothing_control
source install/setup.bash
```

---

## 🚀 Running the Simulation

### Step 1: Launch TurtleBot3 in Gazebo

```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

### Step 2: Launch the path smoothing + controller node

Choose either B-spline or Cubic:

```bash
# B-spline
ros2 launch path_smoothing_control simulation.launch.py spline_type:=bspline

# Cubic spline
ros2 launch path_smoothing_control simulation.launch.py spline_type:=cubic
```

### Step 3: Start the robot movement

```bash
ros2 param set /trajectory_follower start true
```

Now, the robot will start following the trajectory using the pure pursuit controller.

---

## 🧪 Optional: Compare Both Splines (Offline)

```bash
ros2 run path_smoothing_control compare_splines
```

---

## 🧠 RViz Markers

- Blue Line → Smoothed path
- Red Cylinders → Obstacles
- Green Sphere → Live robot position

Ensure RViz is:
- Set to `odom` as Fixed Frame
- Subscribed to `/visualization_marker` and `/visualization_marker_array`

---

## ⚙️ Future Enhancements (Maybe)

- Smarter obstacle-aware path generation (A*, RRT*, Bezier-Elastic Band)
- Live obstacle sensing from LiDAR or depth camera
- Adaptive lookahead distance
- PID or MPC-based trajectory following
- Real-time marker configuration using GUI sliders

---

## 👨‍💻 Author

Developed with 💻 and 🧠 by **Mohammed Abdul Rahman**

GitHub: [abdu7rahman](https://github.com/abdu7rahman)

---

## 📜 License

MIT License — Feel free to fork, modify, or reuse with credits.

---

## ⭐ Star the Repo

If you found this helpful, don’t forget to star it and share it with fellow robotics nerds 🤖✨
