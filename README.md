# 🧪 nav2-slam-lab

A hands-on learning and experimentation lab dedicated to exploring **Simultaneous Localization and Mapping (SLAM)** and the **ROS 2 Navigation (Nav2)** stack. 

This repository serves as a step-by-step development space to build, tune, and benchmark mapping algorithms, autonomous path planning, custom costmap layers, and behavior trees from the ground up—progressing from basic simulation setups to full real-time navigation pipelines.

### 🎯 Key Focus Areas
* **SLAM Workflows:** Mapping with `slam_toolbox`, `cartographer`, and multi-sensor fusion.
* **Transform Trees (TF2):** Robust coordinate frame configuration (`map` ➔ `odom` ➔ `base_footprint` ➔ sensors).
* **Nav2 Architecture:** Tuning global/local planners, controllers (DWB, MPPI), recovery behaviors, and costmaps.
* **Simulation to Hardware:** Testing configurations in Gazebo/Ignition before bridging to physical MCU/embedded robotic platforms.
