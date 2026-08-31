# 🗺️ Chapter 3: Mastering Online SLAM Navigation (Mapping + Nav2)

Welcome to the comprehensive reference guide for **Simultaneous Online SLAM & Autonomous Navigation** in ROS 2. This document explains how to navigate an unknown building **without pre-loading a static map**, allowing the user to click `Nav2 Goal` into unmapped frontiers while **SLAM Toolbox** constructs and refines the occupancy grid in real time.

---

## 📌 Table of Contents
1. [Architectural Philosophy: Static Nav2 vs. Online SLAM Nav2](#1-architectural-philosophy-static-nav2-vs-online-slam-nav2)
2. [Coordinate Frames & The Dynamic Transform Chain](#2-coordinate-frames--the-dynamic-transform-chain)
3. [Lifecycle Orchestration: Merging SLAM Toolbox & Nav2](#3-lifecycle-orchestration-merging-slam-toolbox--nav2)
4. [Navigating into the Unknown: Costmap & Planner Mechanics](#4-navigating-into-the-unknown-costmap--planner-mechanics)
5. [Pose-Graph Loop Closures During Active Navigation](#5-pose-graph-loop-closures-during-active-navigation)
6. [Step-by-Step Launch & Operational Workflow](#6-step-by-step-launch--operational-workflow)
7. [Saving the Dynamically Generated Map On-The-Fly](#7-saving-the-dynamically-generated-map-on-the-fly)
8. [Comparison Matrix: Static Nav2 vs Online SLAM vs Auto-Exploration](#8-comparison-matrix-static-nav2-vs-online-slam-vs-auto-exploration)

---

## 1. Architectural Philosophy: Static Nav2 vs. Online SLAM Nav2

In conventional static navigation (Chapter 2), a robot requires a pre-recorded map file (`.yaml` + `.pgm`) loaded into `map_server`, and `amcl` estimates the robot's pose on that static map.

In **Online SLAM Navigation**, `map_server` and `amcl` are **completely removed**. Instead, **`slam_toolbox`** acts as both the **Live Map Generator** and the **Global Localization Provider**!

```
                                  [RViz2 / User Interaction]
                                               │
                                       (Click Nav2 Goal)
                                               ▼
                              ┌──────────────────────────────────┐
                              │           bt_navigator           │
                              └───────┬──────────────────┬───────┘
                                      │                  │
                                      ▼                  ▼
                         ┌──────────────────────┐  ┌──────────────────────┐
                         │    planner_server    │  │   behavior_server    │
                         │ (allow_unknown=true) │  └──────────┬───────────┘
                         └──────────┬───────────┘             │
                                    │ (Path into Frontier)    │
                                    ▼                         │
                         ┌──────────────────────┐             │
                         │  controller_server   │             │
                         │    (DWB Controller)  │             │
                         └──────────┬───────────┘             │
                                    │ (/cmd_vel)              │
                                    ▼                         │
                        [Gizmo Drives Forward]                │
                                    │                         │
                                    ▼ (New Laser Scans)       │
                         ┌──────────────────────┐             │
                         │     slam_toolbox     │◄────────────┘
                         │ (Online Async SLAM)  │
                         └──────────┬───────────┘
                                    │
               ┌────────────────────┴────────────────────┐
               ▼ (Publishes /map)                        ▼ (Publishes map ➔ odom TF)
      [global_costmap]                           [Global State Estimation]
```

---

## 2. Coordinate Frames & The Dynamic Transform Chain

In both systems, the coordinate hierarchy remains strictly:
$$\mathbf{map} \longrightarrow \mathbf{odom} \longrightarrow \mathbf{base\_footprint} \longrightarrow \mathbf{scan\_frame}$$

```mermaid
graph LR
    map["map<br/>(Dynamic World Origin)"] -->|"slam_toolbox (Pose-Graph Ceres Solver)"| odom["odom<br/>(Smooth Local Odometry)"]
    odom -->|"EKF (robot_localization)"| base_footprint["base_footprint<br/>(Robot Ego Center)"]
    base_footprint -->|"URDF Joint"| scan_frame["scan_frame<br/>(LiDAR Sensor)"]
```

### Key Differences in Transform Authority:

| Responsibility | Static Navigation (`navigation.launch.py`) | Online SLAM Navigation (`slam_navigation.launch.py`) |
| :--- | :--- | :--- |
| **`map` Frame Authority** | `nav2_amcl` (Particle filter matching static map) | `slam_toolbox` (Scan-to-map Ceres optimization) |
| **`/map` Topic Publisher** | `nav2_map_server` (Static `.pgm` file) | `slam_toolbox` (Dynamic ROS OccupancyGrid stream) |
| **Map Expansion** | Fixed boundaries (e.g. $237 \times 242$) | Dynamically expands canvas as robot explores new rooms |
| **Pre-requisite Files** | `map.yaml` and `map.pgm` required | **None! Zero prior knowledge needed.** |

---

## 3. Lifecycle Orchestration: Merging SLAM Toolbox & Nav2

In `slam_navigation.launch.py`, a single unified `lifecycle_manager` orchestrates all nodes:

```python
lifecycle_nodes = [
    'slam_toolbox',        # 1. Starts SLAM engine & map generator
    'controller_server',   # 2. Starts DWB local controller
    'planner_server',      # 3. Starts NavFn global path planner
    'behavior_server',     # 4. Starts Recovery behaviors
    'bt_navigator',        # 5. Starts Behavior Tree engine
    'waypoint_follower'    # 6. Starts Multi-waypoint follower
]
```

### Startup Sequence:
1. **`slam_toolbox` Configures & Activates:**  
   Immediately takes the robot's starting spot as $(0, 0, 0)$ in the `map` frame and begins broadcasting `map ➔ odom`.
2. **Costmaps Initialize:**  
   `global_costmap` and `local_costmap` subscribe to `/map` and `/odometry/filtered`. Because `map ➔ odom` is already live, costmaps activate with zero TF timeout errors.
3. **Action Servers Stand Up:**  
   `bt_navigator` and `waypoint_follower` connect to `planner_server` and `controller_server`, unlocking the `Nav2 Goal` tool in RViz.

---

## 4. Navigating into the Unknown: Costmap & Planner Mechanics

When navigating in an unmapped building, the robot must plan through **Unknown Space** (grey cells in RViz):

```
Occupancy Values:
[-1: Unknown Space] ──► [0: Free Space] ──► [100: Lethal Wall]
```

### Essential Parameter Configurations:

#### 1. `planner_server` Parameter:
```yaml
planner_server:
  ros__parameters:
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      allow_unknown: true   # CRITICAL: Allows Dijkstra to chart paths through unmapped areas!
```
* **If `allow_unknown: false`:** If you click `Nav2 Goal` in an unmapped grey room, the planner will immediately abort with *"Failed to compute path: Goal is in unknown space"*.
* **If `allow_unknown: true`:** Dijkstra assumes unmapped space is temporarily traversable free space until the LiDAR arrives and updates the costmap!

#### 2. `global_costmap` Parameter:
```yaml
global_costmap:
  global_costmap:
    ros__parameters:
      track_unknown_space: true # Differentiates free space (0) from undiscovered space (-1 / 255)
```

---

## 5. Pose-Graph Loop Closures During Active Navigation

As Gizmo explores a long corridor and loops back into the starting room, **SLAM Toolbox performs a Loop Closure**:

```
Before Loop Closure: Odometry drift accumulated (virtual wall offset)
After Loop Closure:  Ceres Solver snaps graph nodes into place!
```

### How Nav2 Reacts to Loop Closures:
1. **Instant TF Update:** SLAM Toolbox adjusts `map ➔ odom` by $\Delta x, \Delta y, \Delta \theta$.
2. **Behavior Tree Replanning:** The Behavior Tree (`bt_navigator`) is running a $1.0\text{ Hz}$ replanning loop. When the map shifts, `ComputePathToPose` automatically recalibrates the path to the updated goal position!
3. **Zero Interruption:** Gizmo smoothly curves its trajectory toward the corrected real-world target without stopping or glitching.

---

## 6. Step-by-Step Launch & Operational Workflow

### Terminal 1: Launch Gazebo Simulation World
```bash
ros2 launch gizmo_gazebo gazebo_simpleWorld.launch.py
```

### Terminal 2: Launch Online SLAM Navigation
```bash
ros2 launch gizmo_navigation slam_navigation.launch.py
```

### Operating in RViz:
1. When RViz opens, you will see Gizmo standing in an initially small circular clearing of discovered map.
2. Click **`Nav2 Goal`** on the top toolbar.
3. Click and drag an arrow into the **dark/grey unmapped area** down a distant hallway.
4. Watch Gizmo autonomously plan a path through the unknown frontier, driving forward while the LiDAR continuously expands the white room walls on the fly! 🏁

---

## 7. Saving the Dynamically Generated Map On-The-Fly

Once you have driven Gizmo around the entire building using `Nav2 Goal` and are happy with the completed map, you can save it to disk directly from Terminal 3:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/ros2_ws/src/gizmo/gizmo_navigation/maps/my_new_world_map
```

This will instantly write:
* `my_new_world_map.yaml` (Metadata & Origin)
* `my_new_world_map.pgm` (High-resolution $0.05\text{m/cell}$ occupancy image)

You can now use this saved map for static navigation (`navigation.launch.py`) anytime in the future!

---

## 8. Comparison Matrix: Static Nav2 vs Online SLAM vs Auto-Exploration

| Feature | Static Nav2 (`navigation.launch.py`) | Online SLAM Nav2 (`slam_navigation.launch.py`) | Autonomous Frontier Exploration |
| :--- | :--- | :--- | :--- |
| **Map State** | Fixed, pre-loaded from disk | Dynamically created in real-time | Dynamically created in real-time |
| **Localization** | AMCL (Particle Filter) | SLAM Toolbox (Pose-Graph Ceres) | SLAM Toolbox (Pose-Graph Ceres) |
| **Goal Generation** | User clicks known room goal | User clicks unmapped frontier goal | **Algorithm autonomously finds frontiers** |
| **CPU Usage** | Lowest (Ideal for low-power edge compute) | Moderate (SLAM optimization + Nav2) | High (SLAM + Nav2 + Frontier clustering) |
| **Best Used For** | Production delivery, warehouse AMR routes | Initial site mapping, dynamic environments | Completely autonomous robotic surveying |

