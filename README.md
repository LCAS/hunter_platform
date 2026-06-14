# Hunter Platform ROS 2 Packages

This repository contains three ROS 2 packages designed for the Hunter platform: `hunter_pltf_bringup`, `hunter_pltf_description`, and `hunter_pltf_gazebo`. These packages facilitate bringing up the robot, providing its description, and simulating it in Gazebo.

## Table of Contents
- [Overview](#overview)
- [Package Details](#package-details)
  - [1. hunter_pltf_bringup](#1-hunter_pltf_bringup)
  - [2. hunter_pltf_description](#2-hunter_pltf_description)
  - [3. hunter_pltf_gazebo](#3-hunter_pltf_gazebo)
- [Getting Started](#getting-started)
- [Usage](#usage)
- [License](#license)

---

## Overview
This repository provides essential tools for working with the Hunter platform, enabling:
- Robot bring-up with ROS 2.
- Viewing and managing the robot's description.
- Simulating the robot in Gazebo with physics and environment interactions.

These packages use the ROS 2 launch system to manage nodes and configurations. This repository is dependent of [hunter_robot](https://github.com/LCAS/hunter_robot.git) repository. 

## Package Details

### 1. `hunter_pltf_bringup`

**Description**: This package is responsible for bringing up the Hunter platform, including controllers and robot state publishers.

**Features**:
- Declares launch arguments for various configurations, such as simulation time, PID gains, and RViz.
- Includes robot description and controllers.
- Uses nodes like `controller_manager` and `robot_state_publisher`.

**Key Launch Arguments**:
- `gui`: Launch RViz2 GUI (default: `true`)
- `use_mock_hardware`: Use mock hardware (default: `true`)
- `kp_v`, `kd_v`: Linear velocity PID gains (default: `40.0`, `0.1`)
- `kp_w`, `kd_w`: Angular velocity PID gains (default: `35.0`, `0.1`)

---

### 2. `hunter_pltf_description`

**Description**: Provides the URDF description of the Hunter platform using `xacro` files.

**Features**:
- Parses the URDF from `hunter_pltf.urdf.xacro`.
- Publishes the robot state with `robot_state_publisher`.
- Optionally launches RViz and Joint State Publisher GUI.

**Key Launch Arguments**:
- `use_sim_time`: Use simulation time (default: `false`)
- `gui`: Launch RViz2 and Joint State Publisher GUI (default: `true`)

---

### 3. `hunter_pltf_gazebo`

**Description**: Enables simulation of the Hunter platform in Gazebo.

**Features**:
- Integrates with Gazebo for robot simulation.
- Loads the robot description and spawns the entity in Gazebo.
- Supports custom world files and simulation time.
- Includes controllers such as `joint_state_broadcaster` and `ackermann_controller`.
- Bridges all sensors (3D LiDAR point clouds, IMU, GPS/NavSat, RGBD cameras)
  from the new Gazebo to ROS 2 via `ros_gz_bridge` (see `config/gz_bridge.yaml`).

**Key Launch Arguments**:
- `use_sim_time`: Use simulation time (default: `true`)
- `world_path`: Path to the Gazebo world file (default: `empty_world.world`)
- `x_pose`, `y_pose`, `roll`, `pitch`, `yaw`: Initial robot pose in Gazebo (default: `0.0`, `0.0`, `0.0`, `0.0`, `1.45`)

---

## Getting Started

### Prerequisites
- ROS 2 Jazzy
- The new Gazebo (gz-sim, Harmonic) with `ros_gz` (`ros_gz_sim`, `ros_gz_bridge`)
- `gz_ros2_control` and `ros2_controllers` (provides `ackermann_steering_controller`)
- Python 3.10 or higher

### Installation
1. Clone this repository into your ROS 2 workspace:
    ```bash
    cd ~/ros2_ws/src
    https://github.com/LCAS/hunter_platform.git
    ```
2. Install dependencies:
    ```bash
    rosdep update
    rosdep install --from-paths . --ignore-src -r -y
    ```
3. Build the workspace:
    ```bash
    cd ~/ros2_ws
    colcon build
    source install/setup.bash
    ```
---

## Usage

### Launch Robot Bringup
To bring up the Hunter platform:
```bash
ros2 launch hunter_pltf_bringup hunter_pltf_bringup.launch.py
```

### Launch Robot Description
To view the robot's description in RViz:
```bash
ros2 launch hunter_pltf_description pltf_rsp.launch.py gui:=true
```

![image](https://github.com/user-attachments/assets/7ff86992-def5-4c62-9f37-51771e4b07b4)


### Simulate in Gazebo
To simulate the robot in Gazebo:
```bash
ros2 launch hunter_pltf_gazebo launch_sim.launch.py
```

![image](https://github.com/user-attachments/assets/c660514b-c3f4-4281-b5c2-db79db238cf1)

---

## License
This repository is licensed under the [Apache License](LICENSE).

---

## Migration: Gazebo Classic to the new Gazebo (Harmonic) + ROS 2 Jazzy

The simulation was updated to run on the new Gazebo (gz-sim / Harmonic) with
ROS 2 Jazzy. Summary of changes and the reasoning behind each:

- World (`hunter_pltf_gazebo/worlds/empty_world.world`): added the required
  gz-sim system plugins (`Physics`, `UserCommands`, `SceneBroadcaster`,
  `Sensors`, `Imu`, `NavSat`). A custom world does not auto-load these, so
  without them no sensor (lidar/camera/IMU/GPS) produces any data.

- 3D LiDAR: replaced the Gazebo-Classic Livox setup (`mid360.xacro` +
  `libros2_livox.so` + the `ros2_livox_simulation` package) with a generic,
  vendor-neutral `lidar_3d.urdf.xacro` built on the native gz-sim `gpu_lidar`
  sensor. It is not tied to a Mid-360 and can model any 3D LiDAR via macro
  parameters. It outputs a real 3D point cloud (`<topic>/points`).

- Point cloud support: `gpu_lidar` and the RGBD cameras publish
  `gz.msgs.PointCloudPacked`, now bridged to `sensor_msgs/PointCloud2`.

- GPS: the GPS sensor type was renamed from `gps` to `navsat` in the new
  Gazebo. The `navsat` sensor is driven by the `NavSat` world system and is
  bridged to `sensor_msgs/NavSatFix` on `gps_base/fix`.

- Depth cameras: switched `type="depth"` to `type="rgbd_camera"`, which natively
  renders both image and registered point cloud.

- Sensor bridging: `hunter_pltf_gazebo/config/gz_bridge.yaml` was expanded from
  clock-only to cover lidar clouds/scans, IMUs, GPS, and the RGBD cameras, and
  the launch file now loads this config instead of a single inline clock bridge.

- ros2_control / Ackermann steering: the robot has two steered front wheels and
  two driven rear wheels, which matches
  `ackermann_steering_controller/AckermannSteeringController`. The controller
  config now documents that mapping and the unused, mismatched
  `tricycle_controller/TricycleController` entry was removed. The hardware
  plugin is the new-Gazebo `gz_ros2_control/GazeboSimSystem`.

- Removed unused assets: `lidar.urdf.xacro` (2D lidar, only in commented code),
  `zed_multi.urdf.xacro` (never instantiated), the `livox360.dae` and `r200.dae`
  meshes, and the `ros2_livox_simulation` dependency in `hunter_pltf_bringup`.

---

## Contributing
Contributions are welcome! Please fork the repository and submit a pull request.

---

For any issues or questions, please open an issue on the repository.

