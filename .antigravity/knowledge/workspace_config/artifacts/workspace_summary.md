# ROS 2 Workspace Configuration Summary

This document summarizes the build and environment configurations for the `ros2_ws` workspace, extracted from Antigravity IDE settings.

## 🛠️ Build Environment
- **ROS 2 Distribution**: `lyrical`
- **Compiler**: `/usr/bin/g++`
- **C++ Standard**: `c++17`
- **C Standard**: `c17`
- **Compile Commands**: Located at `/root/ros2_ws/build/compile_commands.json`

## 📂 Include & Search Paths

### C++ (`c_cpp_properties.json`)
- `${workspaceFolder}/**`
- `/opt/ros/lyrical/include/**`
- `/usr/include/**`

### Python (`settings.json`)
- `/opt/ros/lyrical/lib/python3.14/site-packages`
- `/root/ros2_ws/install/ros2_interfaces/lib/python3.14/site-packages`

## 🚀 Debugging & Tasks
- **Debugger**: GDB (`/usr/bin/gdb`)
- **Launch Configurations**: Supports both C++ and Python ROS 2 nodes with dynamic package/node name inputs.
- **Tasks**: `Launch Terminator` - Opens a terminal window.

## 💡 IntelliSense Notes
- Antigravity IDE IntelliSense engine is disabled in favor of `clangd`.
- `clangd` is configured to use the build directory for compile commands.
