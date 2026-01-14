# nav2-manual-navigation

This repository demonstrates a manual ROS 2 Navigation (Nav2) workflow using modular plugins instead of nav2_bringup. It includes map loading, AMCL localization, planning, and navigation for the Testbed-T1.0.0 robot in Gazebo, showcasing clean launch design and plugin-based navigation architecture.

The implementation includes **map loading**, **localization**, and **navigation**, each launched independently.
<img width="1845" height="1047" alt="Screenshot from 2026-01-13 23-43-55" src="https://github.com/user-attachments/assets/51e9269a-b1d8-4b2c-af8b-04cca16cba80" />

---

## 🎯 Objective

- Manually configure and run ROS 2 Nav2 plugins
- Understand the modular architecture of Nav2
- Set up map loading, AMCL localization, and navigation without `nav2_bringup`
- Validate navigation in Gazebo and RViz

---

## 🗂️ Package Structure

```
├── testbed_description/
│   ├── launch/                     # Launch the full base simulation
│   ├── meshes/
│   ├── rviz/                       # RVIZ configuration files
│   └── urdf/                       # URDF files for Testbed-T1.0.0
├── testbed_gazebo/
│   ├── worlds/                     # Simulation world files
│   ├── launch/                     # Launch files for Gazebo
│   └── models/                     # Misc. Gazebo model files
├── testbed_bringup/ 
│   ├── launch/                     # Launch file for bringing up the robot
│   └── maps/                       # Predefined map of the test environment
├── testbed_navigation/
|   ├── launch/
|   │ ├── map_loader.launch.py      # Launch map_server
|   │ ├── localization.launch.py    # Launch Localization
|   │ ├── navigation.launch.py      # Launch Navigation
|   │ └── complete.launch.py        # Launch the all files
|   ├── config/
|   │ ├── amcl_params.yaml          # AMCL Parameters
|   │ └── nav2_params.yaml          # Navigation Parameters
|   ├── rviz/
|   │ └── nav2_default_view.rviz    # Rviz File
|   ├── CMakeLists.txt
|   └── package.xml
├── frames_2026-01-13_17.30.31.pdf  # Complete Transformation of Robot 
└── README.md                       # Documentation File
```

---

## 🧠 Approach

### 1. Simulation Bringup

The simulation environment is launched using the provided bringup package:

```bash
ros2 launch testbed_bringup testbed_full_bringup.launch.py
```

This launches:

- Gazebo simulation  
- Robot description  
- Sensor data  
- RViz visualization  

---

### 2. Map Loading

- The predefined map from `testbed_bringup/maps/testbed_world.yaml` is used.
- A dedicated launch file `map_loader.launch.py` is created to start the `map_server` plugin.
- Map availability is verified in RViz using the `/map` topic.

---

### 3. Localization (AMCL)

- AMCL parameters are defined in `amcl_params.yaml`.
- Localization is launched independently using `localization.launch.py`.
- Localization accuracy is verified using:
  - Particle cloud visualization
  - Laser scan matching
  - `/amcl_pose` topic in RViz

---

### 4. Navigation

- Navigation is configured using individual Nav2 plugins:
  - Planner Server
  - Controller Server
  - Behavior Server
  - Behavior Tree Navigator
  - Smoother Server
  - Waypoint Server
  - Velocity Smoother
- Parameters are defined in `nav2_params.yaml`.
- The navigation stack is launched using `navigation.launch.py`.
- Navigation goals are sent from RViz to validate path planning and execution.

---

## ⚙️ Challenges Faced

### Map Not Loading Issue

**Problem:**   
The map file existed in the `testbed_bringup/maps` directory but failed to load during runtime.

**Root Cause:**   
The `maps` folder was not installed via `CMakeLists.txt`, so it was not available in the package share directory.

**Solution:**   
Added the maps directory to `CMakeLists.txt`:

### Other Challenges

No additional challenges were encountered.
All Nav2 plugins functioned correctly once parameters and launch order were properly configured.

---

## Installation

1. Create your workspace:
    ```bash
    mkdir -p ~/ros_ws/src
    ```
2. Clone this repository:
   ```bash
   cd ~/ros_ws/src
   git clone https://github.com/yashbhaskar/nav2-manual-navigation.git
   ```
2. Build the workspace:
   ```bash
   cd ~/ros_ws/
   colcon build
   source install/setup.bash
   ```

---

## 🚀 How to Run

### 1st Terminal : Launch Robot Simulation
```bash
ros2 launch testbed_bringup testbed_full_bringup.launch.py
```

### 2nd Terminal : Start Map Server
```bash
ros2 launch testbed_navigation map_loader.launch.py
```

### 3rd Terminal : Start Localization
```bash
ros2 launch testbed_navigation localization.launch.py
```

- Now rviz is open and robot is shown in white colour now use 2D Pose Estimate to set the robot’s initial pose on the map and start AMCL localization.
- Then use Nav2 Goal / 2D Goal Pose to send a navigation goal. The robot will autonomously plan a path and move toward the target, avoiding obstacles and reaching the goal successfully.
- You may notice slight odometry drift or small pose adjustments, which is normal during AMCL convergence as the particle filter refines the robot’s exact position.

### 4th Terminal : Start Navigation
```bash
ros2 launch testbed_navigation navigation.launch.py
```

### OR

### Start Map server, Localization, Navigation Combine
```bash
ros2 launch testbed_navigation complete.launch.py
```

<img width="1845" height="1047" alt="Screenshot from 2026-01-13 23-44-39" src="https://github.com/user-attachments/assets/520d95d8-3eda-462b-8376-0ef2f8146cf9" />
<img width="1845" height="1047" alt="Screenshot from 2026-01-13 23-47-01" src="https://github.com/user-attachments/assets/17b1a0ca-e487-4658-96c6-1bae2ed06640" />

Robot performing localization and navigation video : https://drive.google.com/file/d/1gLpBWZCot82HEz6f8Cu_AdGIS3YZtwPk/view?usp=sharing
---
