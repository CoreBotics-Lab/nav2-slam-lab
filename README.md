# Gizmo Nav2 & SLAM Robotics Lab (`nav2-slam-lab`)

An autonomous mobile robotics engineering repository for **2D Graph SLAM**, **Nav2 Autonomous Navigation**, and **Python Mission Automation** on the **Gizmo 2WD Differential Drive Robot** using **ROS 2 Lyrical**.

---

## Architectural Overview

The Gizmo software architecture consists of modular functional layers: physics simulation, hardware abstraction and sensor bridging, state estimation (EKF), simultaneous localization and mapping (SLAM Toolbox), autonomous navigation (Nav2 with Regulated Pure Pursuit), and high-level mission scripting (Nav2 Simple Commander API).

```
                                  [Python Missions / RViz2 Dashboard]
                                                   │
                               ┌───────────────────┴───────────────────┐
                               ▼                                       ▼
                     [Online SLAM / AMCL]                     [nav2_container]
                   ┌───────────────────────┐         ┌─────────────────────────────────┐
                   │ • slam_toolbox (SLAM) │         │ • controller_server (RPP)       │
                   │ • nav2_amcl (Loc)     │         │ • planner_server (NavFn)        │
                   └───────────┬───────────┘         │ • behavior_server (Recoveries)  │
                               │                     │ • bt_navigator (Behavior Tree)  │
                               │ (map -> odom TF)    │ • waypoint_follower             │
                               ▼                     └────────────────┬────────────────┘
                          [ekf_node]                                  │ (/cmd_vel)
                     (robot_localization)                             ▼
                               │                               [ros_gz_bridge]
                               │ (odom -> base_footprint TF)          │
                               ▼                                      ▼
                    [robot_state_publisher]                  [Gazebo Simulation]
                               │ (base -> link/sensor TFs)    (Gizmo Robot Model)
                               └──────────────────────────────────────┘
```

---

## Workspace Packages

The repository is organized into 7 modular ROS 2 packages inside `src/`:

| Package | Build Type | Primary Responsibility |
| :--- | :--- | :--- |
| **`gizmo_description`** | `ament_cmake` | Kinematic URDF/Xacro robot description, visual and collision geometry, physical inertial parameters, and sensor frame definitions. |
| **`gizmo_gazebo`** | `ament_cmake` | Simulation environments (`simpleWorld.sdf`, `simpleBiggerWorld.sdf`), bidirectional ROS-Gazebo transport bridges, and robot spawning infrastructure. |
| **`gizmo_teleop`** | `ament_cmake` | Velocity command publishing with hardware acceleration and safety limit clamping (`max_linear: 0.73` m/s, `max_angular: 3.14` rad/s). |
| **`gizmo_mapping`** | `ament_cmake` | SLAM Toolbox asynchronous mapping configuration (`slam_toolbox.yaml`), 2D occupancy grid maps (`.yaml`, `.pgm`), and online mapping launch pipelines. |
| **`gizmo_navigation`** | `ament_cmake` | Nav2 configuration (`nav2.yaml`), Regulated Pure Pursuit (RPP) controller, global planners, costmaps, and standalone navigation/localization launch files. |
| **`gizmo_bringup`** | `ament_cmake` | Master bringup orchestration providing unified simulation (`simulation.launch.py`) and hardware-agnostic runtime (`bringup.launch.py`). |
| **`gizmo_scripts`** | `ament_python`| High-level programmatic mission control utilizing the Nav2 Simple Commander API (`BasicNavigator`), YAML waypoint parsing, and automated patrol loops. |

---

## Launch Architecture

The workspace provides launch pipelines structured across 3 architectural layers:

```
[Layer 3: Master Bringup & Simulation Orchestration]
  ├── simulation.launch.py (Full simulation + Gazebo delay + Bringup in mapping or localization mode)
  └── bringup.launch.py    (Hardware-agnostic: Mapping [SLAM] or Localization [AMCL] + Nav2 + RViz2)

[Layer 2: Core Subsystems]
  ├── navigation.launch.py   (Nav2 5-server stack: Composed or Standalone)
  ├── slam.launch.py         (SLAM Toolbox async mapping + lifecycle management)
  └── localization.launch.py (Map Server + AMCL particle filter localization)

[Layer 1: Simulation & Kinematics]
  ├── gazebo.launch.py                   (Base simulator + ros_gz_bridge + EKF)
  ├── gazebo_simpleBiggerWorld.launch.py (Expanded arena world spawner)
  ├── gazebo_simpleWorld.launch.py       (Compact arena world spawner)
  ├── display.launch.py                  (URDF kinematic transform tree inspection in RViz)
  └── teleop.launch.py                   (Manual velocity command publisher)
```

---

### Layer 3: Master Bringup & Simulation Orchestration

#### 1. `simulation.launch.py` (`gizmo_bringup`)
* **Purpose**: Coordinates full end-to-end simulation. Launches Gazebo physics, dynamic robot spawning, ROS-Gazebo transport bridges, EKF state estimation, and triggers the Nav2 software stack after a synchronization delay.
* **Mechanism**:
  1. Spawns the Gazebo physics simulation via `gazebo.launch.py` with the selected world (default `simpleBiggerWorld.sdf`).
  2. Applies a `TimerAction(period=5.0)` delay allowing Gazebo physics, the simulation `/clock`, sensor bridges (`/scan`, `/imu/data`), and EKF odometry (`odom -> base_footprint`) to stabilize.
  3. Includes `bringup.launch.py` with `use_sim_time:=true` in either `localization` or `mapping` mode.
* **Commands**:
  ```bash
  # Autonomous Navigation with pre-built map (AMCL + Map Server + Nav2):
  ros2 launch gizmo_bringup simulation.launch.py mode:=localization

  # Online SLAM Mapping + Autonomous Navigation (SLAM Toolbox + Nav2):
  ros2 launch gizmo_bringup simulation.launch.py mode:=mapping

  # Headless execution (without Gazebo GUI):
  ros2 launch gizmo_bringup simulation.launch.py mode:=localization headless:=true
  ```

---

#### 2. `bringup.launch.py` (`gizmo_bringup`)
* **Purpose**: The hardware-agnostic ROS 2 navigation bringup. Operates without simulator dependencies and can run against an existing simulation instance or on physical robot hardware.
* **Mechanism**:
  * Evaluates the `mode` parameter:
    * `mode:=localization`: Includes `localization.launch.py` from `gizmo_navigation` to run `map_server` and `nav2_amcl`.
    * `mode:=mapping`: Includes `slam.launch.py` from `gizmo_mapping` to run `async_slam_toolbox_node`.
  * Includes `navigation.launch.py` to start the Nav2 controller, planner, behavior server, and BT navigator.
  * Launches RViz2 using the unified configuration [`src/gizmo_navigation/rviz/nav2.rviz`](src/gizmo_navigation/rviz/nav2.rviz) with full Nav2 Goal tooling, costmaps, and real-time path visualizers.
* **Commands**:
  ```bash
  # Localization mode with pre-built map:
  ros2 launch gizmo_bringup bringup.launch.py mode:=localization map:=src/gizmo_mapping/maps/simple_bigger_world_map.yaml

  # Online SLAM mapping mode:
  ros2 launch gizmo_bringup bringup.launch.py mode:=mapping
  ```

---

### Layer 2: Core Subsystems

#### 3. `navigation.launch.py` (`gizmo_navigation`)
* **Purpose**: Launches the core Nav2 autonomous navigation engine.
* **Mechanism**:
  * **Composed Mode (`use_composition:=true`, Default)**: Loads the 5 core navigation servers as composable nodes inside a single `component_container` with isolated single-threaded executors:
    * `nav2_controller::ControllerServer`: Configured with `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController` running at $20\text{ Hz}$ with in-place rotation to heading, lookahead distance scaling, and curvature regulation.
    * `nav2_planner::PlannerServer`: NavFn global Dijkstra/A* path planner operating on the global costmap.
    * `behavior_server::BehaviorServer`: Recovery behaviors (`Spin`, `BackUp`, `Wait`).
    * `nav2_bt_navigator::BtNavigator`: Behavior Tree engine executing `navigate_to_pose` logic.
    * `nav2_waypoint_follower::WaypointFollower`: Sequential multi-pose dispatcher.
  * **Standalone Mode (`use_composition:=false`)**: Spawns each server as an independent OS process for targeted debugging.
  * **Lifecycle Management**: Managed by `lifecycle_manager_navigation` with zero bond timeouts (`bond_timeout: 0.0`) to avoid transient network timeouts.
* **Command**:
  ```bash
  ros2 launch gizmo_navigation navigation.launch.py
  ```

---

#### 4. `slam.launch.py` (`gizmo_mapping`)
* **Purpose**: Performs 2D Simultaneous Localization and Mapping in unknown environments.
* **Mechanism**:
  * Executes `async_slam_toolbox_node` using Google Ceres Solver non-linear graph optimization.
  * Subscribes to laser scans (`/scan`) and odometry (`/odometry/filtered`), constructs the spatial pose-graph, publishes the dynamic `/map` occupancy grid, and broadcasts the authoritative `map -> odom` transform.
  * Managed by `lifecycle_manager_slam` for deterministic lifecycle state transitions.
* **Command**:
  ```bash
  ros2 launch gizmo_mapping slam.launch.py
  ```

---

#### 5. `localization.launch.py` (`gizmo_navigation`)
* **Purpose**: Global localization against a known, pre-recorded occupancy grid map.
* **Mechanism**:
  * `nav2_map_server`: Loads the occupancy grid `.yaml` and `.pgm` files from disk and broadcasts the static `/map` topic with Transient Local QoS.
  * `nav2_amcl`: Adaptive Monte Carlo Localization particle filter tracking the robot pose by matching laser range measurements to the static map and broadcasting `map -> odom`.
  * `lifecycle_manager_localization`: Coordinates transition of `map_server` and `amcl` to the active lifecycle state.
* **Command**:
  ```bash
  ros2 launch gizmo_navigation localization.launch.py map:=src/gizmo_mapping/maps/simple_bigger_world_map.yaml
  ```

---

### Layer 1: Simulation & Kinematics

#### 6. `gazebo.launch.py` (`gizmo_gazebo`)
* **Purpose**: Base simulation runtime managing physics, kinematics, sensor simulation, and odometry estimation.
* **Mechanism**: Starts Gazebo Sim, evaluates the robot Xacro model, spawns the robot entity, configures the `ros_gz_bridge` transport bridge, and runs `robot_localization` EKF fusing wheel odometry and IMU angular velocity.
* **Command**:
  ```bash
  ros2 launch gizmo_gazebo gazebo.launch.py world_file:=empty.sdf
  ```

---

#### 7. `gazebo_simpleBiggerWorld.launch.py` (`gizmo_gazebo`)
* **Purpose**: Convenience launcher for the large multi-room arena world (`simpleBiggerWorld.sdf`).
* **Command**:
  ```bash
  ros2 launch gizmo_gazebo gazebo_simpleBiggerWorld.launch.py
  ```

---

#### 8. `gazebo_simpleWorld.launch.py` (`gizmo_gazebo`)
* **Purpose**: Convenience launcher for the compact obstacle testing arena (`simpleWorld.sdf`).
* **Command**:
  ```bash
  ros2 launch gizmo_gazebo gazebo_simpleWorld.launch.py
  ```

---

#### 9. `display.launch.py` (`gizmo_description`)
* **Purpose**: Kinematic visualization of the URDF robot model in RViz without physics simulation.
* **Mechanism**: Publishes the static transform tree via `robot_state_publisher` and provides joint state inspection tools.
* **Command**:
  ```bash
  ros2 launch gizmo_description display.launch.py
  ```

---

#### 10. `teleop.launch.py` (`gizmo_teleop`)
* **Purpose**: Velocity command publishing for manual motion testing.
* **Mechanism**: Converts input commands to clamped `geometry_msgs/msg/Twist` messages published to `/cmd_vel` at $20\text{ Hz}$.
* **Command**:
  ```bash
  ros2 launch gizmo_teleop teleop.launch.py
  ```

---

## Quickstart Guide

### 1. Build the Workspace
Build all workspace packages using Pixi:
```bash
pixi run build
```

### 2. Autonomous Navigation with Pre-Built Map (Localization Mode)
```bash
ros2 launch gizmo_bringup simulation.launch.py mode:=localization
```
* When RViz opens, select **2D Goal Pose** on the top toolbar and click any target destination on the map.
* Gizmo computes the global path and navigates using the Regulated Pure Pursuit controller.

### 3. Online SLAM Mapping and Autonomous Navigation (Mapping Mode)
```bash
ros2 launch gizmo_bringup simulation.launch.py mode:=mapping
```
* When RViz opens, select **2D Goal Pose** on the top toolbar and set a goal into unknown space.
* Gizmo explores the frontier while SLAM Toolbox incrementally expands and optimizes the map.

### 4. Execute Autonomous Waypoint Patrol from Python
While the navigation stack is active, run the waypoint mission script:
```bash
python3 src/gizmo_scripts/gizmo_scripts/practice/waypointFromYaml.py
```

### 5. Save the Generated Map
Save the SLAM occupancy grid map to disk:
```bash
ros2 run nav2_map_server map_saver_cli -f src/gizmo_mapping/maps/my_new_map
```

---

## Technical Documentation Chapters

For in-depth mathematical formulations, parameter documentation, and controller tuning:
1. [Chapter 1: 2D SLAM & SLAM Toolbox](docs/01_slamtoolbox.md): Pose-Graph SLAM, Ceres optimization, scan matching, and lifecycle orchestration.
2. [Chapter 2: Autonomous Navigation with Nav2](docs/02_nav2.md): 2D Costmaps, Regulated Pure Pursuit, Behavior Tree orchestration, and process composition.
3. [Chapter 3: Online SLAM Navigation](docs/03_online_slam_nav2.md): Single-parent TF rules, dynamic frontier planning, and multi-stage lifecycle transitions.
4. [Chapter 4: Nav2 Simple Commander API](docs/04_nav2_simple_commander.md): Programmatic Python missions, `BasicNavigator`, feedback streaming, and YAML waypoint execution.
