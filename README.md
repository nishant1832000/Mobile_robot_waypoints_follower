# Mobile Robot Waypoints Follower

This package provides a complete pipeline for **trajectory generation and tracking** for a TurtleBot3 robot in simulation.  
It includes waypoint visualization, spline-based path smoothing, and trapezoidal velocity profiling for motion control.

---

##  Overview

This project demonstrates:
1. Launching a TurtleBot3 in Gazebo’s empty world.  
2. Visualizing waypoints and a smooth cubic spline path in RViz.  
3. Generating a **C² continuous natural cubic spline** through given waypoints.  
4. Generating a **velocity trapezoid** along this path and executing motion using **PD control**.

---

##  Prerequisites

Make sure you have the following installed:
- **ROS 2 (Humble / Jazzy)**
- **Gazebo** (Ignition / Fortress / Garden)
- **TurtleBot3 Simulation Packages**

Clone and build required repositories:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git -b main
git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git -b main
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

##  Launch Sequence

### **Step 1 — Launch the TurtleBot3 Simulation**
Run Gazebo with an empty world:
```bash
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

---

### **Step 2 — Open RViz**
In a new terminal:
```bash
rviz2
```
- Add a **MarkerArray** display for topic `/waypoints_markers`
- Add a **Path** display for topic `/spline_path`

This allows you to visualize both the waypoints and the generated spline path.

---

### **Step 3 — Run the Smoother Node**
The smoother node generates a **C² continuous natural cubic spline** passing through the given waypoints:

```bash
ros2 run ten_x smoother
```

**Publishes:**
- `/spline_path` → `nav_msgs/Path`
- `/waypoints_markers` → `visualization_msgs/MarkerArray`

---

### **Step 4 — Run the Trajectory Node**
Using the spline path from the smoother node, this node:
- Fits a **velocity trapezoid** over the path  
- Generates a **smooth trajectory**  
- Moves the TurtleBot using **PD control**

```bash
ros2 run ten_x trapezoid
```

---

##  Repository Sources

The following dependencies are included or required:

```yaml
repositories:
  turtlebot3_simulation:
    type: git
    url: https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
    version: main
  turtlebot3_msgs:
    type: git
    url: https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
    version: main
```

---
## Future work

For avoiding obstacle we can use additional waypoints such that new path changes and dont go through the obstacle.
And for adding additional waypoints we can use algorithm like RRT or RRT*.



---

##  Visualization

| Component | Topic | Type |
|------------|--------|------|
| Waypoints  | `/waypoints_markers` | `visualization_msgs/MarkerArray` |
| Spline Path | `/spline_path` | `nav_msgs/Path` |
| Velocity Commands | `/cmd_vel` | `geometry_msgs/TwistStamped` |

---
 

##  License
This project is released under the [MIT License](LICENSE).

---

## Author
**Nishant**  
Email - nishant1832000@gmail.com 
