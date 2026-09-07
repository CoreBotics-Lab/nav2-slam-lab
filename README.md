# 🤖 Gizmo Nav2 & SLAM Robotics Lab (`nav2-slam-lab`)

A comprehensive robotics engineering repository dedicated to **2D SLAM**, **Autonomous Navigation (Nav2)**, and **Python Mission Control** on the **Gizmo 2WD Differential Drive Robot** using **ROS 2 Lyrical**.

---

## 📌 Architectural Overview

The Gizmo software stack is structured into modular layers: simulation physics, sensor bridging, state estimation (EKF), mapping (SLAM Toolbox), autonomous navigation (Nav2 with process composition), and programmatic missions (Simple Commander API).

```
                                  [Python Missions / RViz2]
                                             │
                         ┌───────────────────┴───────────────────┐
                         ▼                                       ▼
               [slam_toolbox]                           [nav2_container]
             (Online Graph SLAM)               ┌─────────────────────────────────┐
                         │                     │ • controller_server (DWB)       │
                         │ (map ➔ odom TF)     │ • planner_server (NavFn)        │
                         ▼                     │ • behavior_server (Recoveries)  │
                    [ekf_node]                 │ • bt_navigator (Behavior Tree)  │
               (robot_localization)            │ • waypoint_follower             │
                         │                     └────────────────┬────────────────┘
                         │ (odom ➔ base TF)                     │ (/cmd_vel)
                         ▼                                      ▼
             [robot_state_publisher]                  [ros_gz_bridge]
                         │ (base ➔ sensor TF)                   │
                         └───────────────────┬──────────────────┘
                                             ▼
                                     [Gazebo Simulation]
                                      (Gizmo Robot Model)
```

---

## 📦 Packages in `src/gizmo`

| Package | Build Type | Primary Responsibility |
| :--- | :--- | :--- |
| **`gizmo_description`** | `ament_cmake` | URDF/Xacro kinematic model, visual/collision meshes, physical inertia, and Gazebo sensor attachments. |
| **`gizmo_gazebo`** | `ament_cmake` | Simulation worlds (`simpleWorld.sdf`, `simpleBiggerWorld.sdf`), Gazebo-to-ROS bridges, EKF configuration, and world spawners. |
| **`gizmo_teleop`** | `ament_cmake` | Teleoperation node converting user input to `/cmd_vel` velocity commands with hardware velocity clamping. |
| **`gizmo_navigation`** | `ament_cmake` | Core Nav2 parameters (`nav2.yaml`), SLAM Toolbox configuration (`slam_toolbox.yaml`), pre-built maps, and modular subsystem launch files. |
| **`gizmo_bringup`** | `ament_cmake` | Top-level master bringup launch files orchestrating simulation, state estimation, mapping, navigation, and visualization into single commands. |
| **`gizmo_scripts`** | `ament_python`| High-level Python mission scripts using `nav2_simple_commander` (`BasicNavigator`), YAML waypoint parsing, and automated patrol logic. |

---

## 🚀 Complete Launch Files Deep-Dive: Why We Need Them & How They Work

The repository provides **10 modular launch files** organized across 3 architectural layers:

```
[Layer 3: Master Bringup Orchestration]
  ├── slamNavigation_simpleBiggerWorld.launch.py (Full simulation + delayed SLAM + Nav2)
  └── slamNavigation_bringup.launch.py           (Unified SLAM or Localization + Nav2 + RViz)

[Layer 2: Core Subsystems]
  ├── navigation.launch.py   (Nav2 5-server stack: Composed or Standalone)
  ├── slam.launch.py         (SLAM Toolbox async mapping + lifecycle manager)
  └── localization.launch.py (Map Server + AMCL particle filter localization)

[Layer 1: Simulation & Robot Model]
  ├── gazebo_simpleBiggerWorld.launch.py (Expanded arena world spawner)
  ├── gazebo_simpleWorld.launch.py       (Starter arena world spawner)
  ├── gazebo.launch.py                   (Base simulator + ros_gz_bridge + EKF)
  ├── display.launch.py                  (URDF kinematic inspection in RViz)
  └── teleop.launch.py                   (Manual joystick velocity controller)
```

---

### Layer 3: Master Bringup Orchestration

#### 1. `slamNavigation_simpleBiggerWorld.launch.py` (`gizmo_bringup`)
* **Why We Need It:** Starting a full robotics simulation involves multiple asynchronous subsystems: Gazebo physics, simulation clock broadcasting, joint transforms, sensor bridges, EKF odometry, SLAM, and Nav2. If all nodes launch at the exact same millisecond, Nav2 and SLAM attempt to look up transforms before the Gazebo clock and EKF have established a valid transform cache. This launch file coordinates the entire system into a single, reliable command.
* **How It Works:**
  1. Boots the Gazebo simulation immediately via `gazebo_simpleBiggerWorld.launch.py`.
  2. Uses a ROS 2 **`TimerAction(period=5.0)`** to introduce a 5-second stabilization window. This allows Gazebo physics, `/clock`, and EKF odometry (`odom ➔ base_footprint`) to stabilize.
  3. Automatically triggers `slamNavigation_bringup.launch.py` with `slam:=true`, activating SLAM Toolbox and the composed Nav2 stack without transform errors.
* **Command:**
  ```bash
  ros2 launch gizmo_bringup slamNavigation_simpleBiggerWorld.launch.py
  # Headless mode (no Gazebo GUI):
  ros2 launch gizmo_bringup slamNavigation_simpleBiggerWorld.launch.py headless:=true
  ```

---

#### 2. `slamNavigation_bringup.launch.py` (`gizmo_bringup`)
* **Why We Need It:** Serves as the flexible bridge between environment tracking (mapping vs. localization) and autonomous navigation. It allows operators to switch seamlessly between creating a new map (`slam:=true`) and navigating an existing map (`slam:=false`) using identical Nav2 navigation pipelines.
* **How It Works:**
  * Evaluates the boolean `slam` argument via `GroupAction` conditional logic:
    * If `slam == true`: includes `slam.launch.py` to start SLAM Toolbox.
    * If `slam == false`: includes `localization.launch.py` to start Map Server and AMCL.
  * Always includes `navigation.launch.py`, passing `use_composition` and parameter files.
  * Optionally launches RViz2 with the unified `nav2.rviz` display configuration.
* **Command:**
  ```bash
  # Online SLAM Mapping Mode:
  ros2 launch gizmo_bringup slamNavigation_bringup.launch.py slam:=true

  # Static Map Localization Mode:
  ros2 launch gizmo_bringup slamNavigation_bringup.launch.py slam:=false map:=/path/to/map.yaml
  ```

---

### Layer 2: Core Subsystems

#### 3. `navigation.launch.py` (`gizmo_navigation`)
* **Why We Need It:** Provides the autonomous navigation engine. It evaluates environmental costmaps, plans optimal paths around obstacles, generates motor velocities, coordinates recoveries, and tracks multi-stop waypoints.
* **How It Works:**
  * **Composed Mode (`use_composition:=true`, Default):** Following the official Navigation2 architecture, starts a single `component_container` with isolated single-threaded executors (`--isolated --executor-type single-threaded`) and loads the 5 core servers into memory:
    * `nav2_controller::ControllerServer`: DWB local planner computing `/cmd_vel` at $20\text{ Hz}$.
    * `nav2_planner::PlannerServer`: NavFn global Dijkstra/A* path planner.
    * `behavior_server::BehaviorServer`: Recovery behaviors (`Spin`, `BackUp`, `Wait`).
    * `nav2_bt_navigator::BtNavigator`: Behavior Tree orchestrator running `navigate_to_pose`.
    * `nav2_waypoint_follower::WaypointFollower`: Multi-goal sequential dispatcher.
    * *Benefit:* Eliminates multi-process network discovery delays, reduces memory overhead, and allows instant service synchronization.
  * **Standalone Mode (`use_composition:=false`):** Spawns each server as an independent OS process for targeted debugging.
  * **Lifecycle Management:** Includes `lifecycle_manager_navigation` to transition all 5 servers through `Configure` and `Activate` with `bond_timeout: 0.0`.
* **Command:**
  ```bash
  ros2 launch gizmo_navigation navigation.launch.py
  ```

---

#### 4. `slam.launch.py` (`gizmo_navigation`)
* **Why We Need It:** Performs 2D Simultaneous Localization and Mapping when navigating an unfamiliar environment with zero prior knowledge.
* **How It Works:**
  * Launches `async_slam_toolbox_node` (`slam_toolbox`), which subscribes to raw LiDAR scans (`/scan`) and odometry (`/odometry/filtered`).
  * Runs Google Ceres Solver non-linear optimization to construct an exact pose-graph, streams the dynamic `/map` occupancy grid, and broadcasts the authoritative `map ➔ odom` transform.
  * Uses `lifecycle_manager_slam` to automatically manage the node's lifecycle state machine without requiring manual service calls.
* **Command:**
  ```bash
  ros2 launch gizmo_navigation slam.launch.py
  ```

---

#### 5. `localization.launch.py` (`gizmo_navigation`)
* **Why We Need It:** Used during production navigation in known, pre-mapped environments. Rather than recalculating the map continuously, it relies on a saved map to minimize CPU usage and prevent temporary obstacles from corrupting the layout.
* **How It Works:**
  * `nav2_map_server`: Reads pre-saved `.yaml` and `.pgm` occupancy files from disk and publishes the static `/map` grid with Transient Local QoS.
  * `nav2_amcl`: Adaptive Monte Carlo Localization. Spawns a particle filter that compares incoming LiDAR `/scan` beams against the static map to track the robot's pose and broadcast `map ➔ odom`.
  * `lifecycle_manager_localization`: Coordinates the sequential activation of `map_server` followed by `amcl`.
* **Command:**
  ```bash
  ros2 launch gizmo_navigation localization.launch.py map:=/root/ros2_ws/src/gizmo/gizmo_navigation/maps/simple_world_map.yaml
  ```

---

### Layer 1: Simulation & Robot Model

#### 6. `gazebo.launch.py` (`gizmo_gazebo`)
* **Why We Need It:** The base simulation engine that simulates the physical robot, physics world, contact dynamics, motor actuation, LiDAR raytracing, and IMU inertial measurements.
* **How It Works:**
  * Starts Gazebo Sim (`gz sim`) with the specified `.sdf` world.
  * Evaluates `gizmo.urdf.xacro` with sensor flags (`camera_type`, `use_lidar`, `use_camera`, `use_imu`) and feeds it to `robot_state_publisher`.
  * Invokes `ros_gz_sim create` to dynamically spawn Gizmo at the origin.
  * Starts `ros_gz_bridge parameter_bridge` mapping topics (`/clock`, `/cmd_vel`, `/wheel_odom`, `/joint_states`, `/scan`, `/imu/data`).
  * Runs `robot_localization` EKF (`ekf_node`) fusing wheel encoders and IMU gyro rates to broadcast a drift-compensated `odom ➔ base_footprint` transform.
* **Command:**
  ```bash
  ros2 launch gizmo_gazebo gazebo.launch.py world_file:=empty.sdf
  ```

---

#### 7. `gazebo_simpleBiggerWorld.launch.py` (`gizmo_gazebo`)
* **Why We Need It:** Provides a standardized launch shortcut for the larger arena world (`simpleBiggerWorld.sdf`) designed for extensive multi-room mapping, waypoint navigation, and long-range path planning.
* **How It Works:** Includes `gazebo.launch.py` with pre-configured parameters: sets `world_file` to `simpleBiggerWorld.sdf`, disables resource-heavy RGB cameras (`use_camera: 'false'`), and suppresses duplicate RViz windows (`run_rviz2: 'false'`).
* **Command:**
  ```bash
  ros2 launch gizmo_gazebo gazebo_simpleBiggerWorld.launch.py
  ```

---

#### 8. `gazebo_simpleWorld.launch.py` (`gizmo_gazebo`)
* **Why We Need It:** Provides a lightweight testing environment using `simpleWorld.sdf` for quick local planner tuning, controller verification, and obstacle clearance testing.
* **How It Works:** Includes `gazebo.launch.py` with `world_file` pointed to `simpleWorld.sdf`, optimizing resources for faster startup on development workstations.
* **Command:**
  ```bash
  ros2 launch gizmo_gazebo gazebo_simpleWorld.launch.py
  ```

---

#### 9. `display.launch.py` (`gizmo_description`)
* **Why We Need It:** Allows developers to inspect the robot's physical URDF/Xacro model, verify kinematic transform trees (`base_footprint ➔ chassis ➔ wheels ➔ sensors`), check visual/collision mesh alignment, and test joint limits without booting a physics simulator.
* **How It Works:** Compiles `gizmo.urdf.xacro` using `xacro`, starts `robot_state_publisher` to publish the static link tree, launches `joint_state_publisher_gui` providing interactive GUI sliders for wheel rotation, and opens RViz with `display.rviz`.
* **Command:**
  ```bash
  ros2 launch gizmo_description display.launch.py
  ```

---

#### 10. `teleop.launch.py` (`gizmo_teleop`)
* **Why We Need It:** Enables direct human control over the robot for manual exploration, emergency maneuvering, or creating an initial baseline map before autonomous navigation is engaged.
* **How It Works:** Runs the `joystick_gui` node, which presents an interactive 2D control widget. User drag inputs are converted to `geometry_msgs/msg/Twist` velocity commands, strictly clamped to safe hardware limits (`max_linear: 0.73` m/s, `max_angular: 3.14` rad/s), and published to `/cmd_vel` at $20\text{ Hz}$.
* **Command:**
  ```bash
  ros2 launch gizmo_teleop teleop.launch.py
  ```

---

## ⚡ Quickstart Guide

### 1. Build the Workspace
```bash
cd /root/ros2_ws
source /opt/ros/lyrical/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 2. Autonomous Online SLAM Navigation (One Command)
```bash
ros2 launch gizmo_bringup slamNavigation_simpleBiggerWorld.launch.py
```
* Once RViz opens, select **Nav2 Goal** on the toolbar and click any unmapped region.
* Gizmo will plan a path through the unknown frontier while the LiDAR maps walls in real-time.

### 3. Execute Autonomous Waypoint Patrol from Python
While the navigation stack is active, run the YAML-driven waypoint follower:
```bash
python3 /root/ros2_ws/src/gizmo/gizmo_scripts/gizmo_scripts/practice/waypointFromYaml.py
```
Gizmo reads checkpoint coordinates from `test_waypoint.yaml`, navigates through each target sequentially, and streams real-time distance remaining.

### 4. Save the Generated Map
```bash
ros2 run nav2_map_server map_saver_cli -f /root/ros2_ws/src/gizmo/gizmo_navigation/maps/my_new_map
```

---

## 📚 Deep-Dive Documentation Chapters

For detailed mathematical formulations, parameter tuning, and operational guides:
1. [📖 Chapter 1: Mastering 2D SLAM & SLAM Toolbox](docs/01_slamtoolbox.md) — Exact Pose-Graph SLAM, Ceres optimization, scan matching, and lifecycle orchestration.
2. [📖 Chapter 2: Mastering Autonomous Navigation with Nav2](docs/02_nav2.md) — Costmaps 2D, DWB trajectory evaluation, Behavior Tree orchestration, and process composition.
3. [📖 Chapter 3: Mastering Online SLAM Navigation (Mapping + Nav2)](docs/03_online_slam_nav2.md) — Single-parent TF rules, dynamic frontier planning, and 2-phase lifecycle transitions.
4. [📖 Chapter 4: Mastering Nav2 Simple Commander API](docs/04_nav2_simple_commander.md) — Programmatic Python missions, `BasicNavigator`, feedback polling, and YAML waypoint missions.
