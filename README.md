# Leo Drives Around - Autonomous Rover Navigation Stack

A ROS 2 Jazzy navigation and localization workspace configured for the Leo Rover operating in rough-terrain simulated environments (Gazebo Mars Yard).

## Overview

The stack integrates 3D LiDAR odometry, kinematic sensor fusion, traversability cost analysis, and Nav2 to achieve autonomous waypoint navigation in rough outdoor terrain:

- **Gazebo Simulation (`gazebo_stuff` & `leo_*`)**: Simulates the Leo Rover equipped with a 3D LiDAR, IMU, GPS, and wheel encoders in a custom Mars Yard environment.
- **Localization (`rover_localization`)**: Multi-sensor fusion using KISS-ICP for 3D point cloud scan matching, combined with an EKF fusing wheel odometry, IMU, LiDAR odometry, and global GPS/satellite corrections.
- **Traversability Analysis (`rover_traversability`)**: Evaluates terrain steepness, step heights, and point cloud elevation to produce continuous traversability costmaps for path planning.
- **Navigation (`rover_navigation`)**: Nav2 integration with global planning, local path regulation (Regulated Pure Pursuit), costmap management, and an automated multi-waypoint mission runner.

## Directory Structure

```text
├── .devcontainer/         # VS Code Dev Container configuration (ROS 2 Jazzy Desktop)
├── src/
│   ├── gazebo_stuff/             # Mars Yard world, rover model configuration, and simulation launch
│   ├── leo_common-ros2/          # Leo Rover descriptions, meshes, and common definitions
│   ├── leo_simulator-ros2/       # Leo Rover Gazebo plugins and simulation bindings
│   ├── rover_localization/       # Sensor fusion (EKF, KISS-ICP, GPS datum alignment)
│   ├── rover_traversability/     # Point cloud processing and terrain traversability cost generation
│   ├── rover_navigation/         # Nav2 configuration, mission runner, and bringup launch scripts
│   ├── kiss_icp/                 # LiDAR odometry registration
│   ├── linefit_ground_segmentation/ # Ground point segmentation library
│   ├── elevation_mapping_cpu/    # CPU-based elevation mapping
│   └── kindr / kindr_ros/        # Kinematics and coordinate transformation libraries
```

## Prerequisites

- Ubuntu 24.04 with ROS 2 Jazzy (or Docker with the provided devcontainer)
- VS Code Dev Containers (recommended)
- Required system packages:
  - `ros-jazzy-desktop-full`
  - `ros-jazzy-navigation2`
  - `ros-jazzy-nav2-bringup`
  - `ros-jazzy-robot-localization`

## Build Instructions

Inside the workspace directory (or within the Dev Container):

```bash
# Source ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Install dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source workspace overlay
source install/setup.bash
```

## Quick Start

### 1. Full Autonomous Stack Bringup

Launch Gazebo, the rover model, localization, traversability mapping, and Nav2:

```bash
ros2 launch rover_navigation marsyard_bringup.launch.py
```

### 2. Run the Autonomous Mission

Once Nav2 has transitioned to the active lifecycle state and localization has initialized, trigger the waypoint mission:

```bash
ros2 run rover_navigation mission_runner
```

## Launch Modules

Individual components can also be launched independently:

- **Simulation only:**
  ```bash
  ros2 launch gazebo_stuff rover_sim.launch.py
  ```
- **Localization stack:**
  ```bash
  ros2 launch rover_localization localization.launch.py
  ```
- **Traversability node:**
  ```bash
  ros2 launch rover_traversability traversability.launch.py
  ```
- **Navigation (Nav2):**
  ```bash
  ros2 launch rover_navigation navigation.launch.py
  ```
