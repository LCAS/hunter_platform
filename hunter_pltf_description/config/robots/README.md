# Generic Robot Configuration System

This directory contains a **generic, reusable** robot configuration system that automatically maps YAML configurations to URDF xacro arguments. The system is not specific to Hunter platforms and can be used with any ROS robot.

## Key Features

- **Load from anywhere**: Local files, absolute paths, file:// URLs, or https:// URLs
- **Generic mapping**: Automatically maps YAML structure to xacro arguments (no hardcoded sensor names)
- **Inheritance-based**: Robot-specific configs only override what differs from defaults
- **Reusable**: Works with any URDF/xacro structure, not just Hunter platform

## Overview

The configuration system allows you to:
- Define baseline sensor poses in `default.yaml` or any custom file
- Create robot-specific overrides in separate YAML files
- Load configurations from local files, remote URLs, or any accessible location
- Automatically map nested YAML structures to flat xacro arguments
- Maintain only the differences from the baseline, minimizing duplication
- Version control calibration data for each robot instance

## File Structure

```
config/robots/
├── README.md          # This file
├── default.yaml       # Baseline configuration for all robots
├── hunter_01.yaml     # Configuration for Hunter-01 robot
└── hunter_XX.yaml     # Additional robot-specific configurations
```

## Configuration File Format

Each configuration file contains:

```yaml
robot_id: "hunter_01"  # Unique identifier for the robot

sensors:
  sensor_name:
    x: 0.0      # Position in meters (X-axis, forward)
    y: 0.0      # Position in meters (Y-axis, left)
    z: 0.0      # Position in meters (Z-axis, up)
    roll: 0.0   # Orientation in radians (rotation around X-axis)
    pitch: 0.0  # Orientation in radians (rotation around Y-axis)
    yaw: 0.0    # Orientation in radians (rotation around Z-axis)
```

### Sensor Names (Hunter Platform Example)

For the Hunter platform, the following sensors are configurable:

- **`imu`**: Rear IMU sensor (at GPS base location)
- **`imu1`**: Front IMU sensor
- **`front_camera`**: Front depth camera
- **`back_camera`**: Rear depth camera
- **`gps_base`**: GPS base antenna
- **`front_lidar`**: Front Mid-360 LiDAR
- **`back_lidar`**: Rear Mid-360 LiDAR

**Note**: The sensor names in your YAML should match the xacro argument prefixes in your URDF. The system automatically flattens nested structures (e.g., `sensors.imu.x` → `imu_x`).

### LiDAR Configuration

LiDAR sensors have separate orientation values for simulation and real hardware:

```yaml
front_lidar_link:
  x: 0.56
  y: 0.235
  z: 0.46
  # Simulation orientations
  roll_sim: 0.0
  pitch_sim: 0.0
  yaw_sim: 0.0
  # Real hardware orientations (calibrated)
  roll_real: -0.0174533
  pitch_real: 0.0
  yaw_real: 0.00872665
  topic: "front_lidar/points"
```

## Creating a New Robot Configuration

1. **Copy the default configuration:**
   ```bash
   cp default.yaml hunter_XX.yaml
   ```

2. **Update the robot_id:**
   ```yaml
   robot_id: "hunter_XX"
   ```

3. **Remove entries that match the default** - only keep values that differ

4. **Calibrate sensor positions:**
   - Physically measure sensor positions relative to `base_link`
   - Update the YAML file with measured values
   - Test in simulation first, then on real hardware

5. **Document your calibration:**
   - Add calibration date and notes in comments
   - Include any special considerations or known issues

## Using Robot Configurations

### In Launch Files

The system supports multiple ways to specify robot configurations:

#### 1. Simple Robot ID (Local File)
```bash
# Using default configuration from config/robots/default.yaml
ros2 launch hunter_pltf_description pltf_rsp.launch.py

# Using hunter_01 from config/robots/hunter_01.yaml
ros2 launch hunter_pltf_description pltf_rsp.launch.py robot_id:=hunter_01

# Using hunter_01 for bringup
ros2 launch hunter_pltf_bringup hunter_pltf_bringup.launch.py robot_id:=hunter_01

# Using hunter_01 for simulation
ros2 launch hunter_pltf_gazebo launch_sim.launch.py robot_id:=hunter_01
```

#### 2. Absolute File Path
```bash
# Load from absolute path
ros2 launch hunter_pltf_description pltf_rsp.launch.py \
    robot_id:=/path/to/my/robot_config.yaml
```

#### 3. File URL
```bash
# Load from file:// URL
ros2 launch hunter_pltf_description pltf_rsp.launch.py \
    robot_id:=file:///path/to/my/robot_config.yaml
```

#### 4. Remote HTTPS URL
```bash
# Load from remote server (e.g., central configuration repository)
ros2 launch hunter_pltf_description pltf_rsp.launch.py \
    robot_id:=https://config.example.com/robots/hunter_01.yaml
```

### Environment Variable (Alternative)

You can also set the robot ID via environment variable:

```bash
export HUNTER_ROBOT_ID=hunter_01
ros2 launch hunter_pltf_description pltf_rsp.launch.py

# Or with full path/URL
export HUNTER_ROBOT_ID=https://config.example.com/robots/hunter_01.yaml
ros2 launch hunter_pltf_description pltf_rsp.launch.py
```

## Inheritance and Overrides

Robot-specific configuration files **inherit** from `default.yaml`. Only specify values that differ:

**Example:** If Hunter-02 only has a different front camera position:

```yaml
# hunter_02.yaml
robot_id: "hunter_02"

sensors:
  front_camera:
    x: 0.553  # Only this value differs from default
    # All other sensors use default values
```

## Validation and Testing

After creating or modifying a configuration:

1. **Validate YAML syntax:**
   ```bash
   python3 -c "import yaml; yaml.safe_load(open('hunter_XX.yaml'))"
   ```

2. **Test in simulation:**
   ```bash
   ros2 launch hunter_pltf_gazebo launch_sim.launch.py robot_id:=hunter_XX
   ```

3. **Visualize in RViz:**
   ```bash
   ros2 launch hunter_pltf_description pltf_rsp.launch.py robot_id:=hunter_XX gui:=true
   ```

4. **Verify sensor frames:**
   ```bash
   ros2 run tf2_tools view_frames
   ```

## Generic YAML-to-Xacro Mapping

The configuration system uses **automatic, generic mapping** from nested YAML structures to flat xacro arguments.

### How It Works

The system:
1. Extracts the `sensors` section from your YAML (configurable)
2. Flattens the nested structure using underscore separators
3. Filters based on simulation mode (`_sim` vs `_real` suffixes)
4. Converts all values to strings for xacro

### Example Mapping

**YAML Input:**
```yaml
sensors:
  imu:
    x: -0.25
    y: 0.0
    topic: "/imu/data"
  front_lidar:
    x: 0.56
    roll_sim: 0.0
    roll_real: -0.0174533
```

**Xacro Arguments (is_sim=True):**
```
imu_x:=-0.25
imu_y:=0.0
imu_topic:=/imu/data
front_lidar_x:=0.56
front_lidar_roll_sim:=0.0
```

**Xacro Arguments (is_sim=False):**
```
imu_x:=-0.25
imu_y:=0.0
imu_topic:=/imu/data
front_lidar_x:=0.56
front_lidar_roll_real:=-0.0174533
```

### Reusability for Other URDFs

This system works with **any URDF**, not just Hunter platform:

1. **Define your YAML structure** to match your xacro argument names
2. **Use nested dictionaries** for organization (e.g., `sensors`, `joints`, `links`)
3. **Flattening is automatic** - `sensors.camera.x` becomes `camera_x`
4. **Use _sim/_real suffixes** for mode-specific values

**Example for a different robot:**
```yaml
# my_robot_config.yaml
robot_id: "myrobot_01"

# Your own structure - not limited to "sensors"
manipulator:
  joint1:
    position: 0.0
    velocity_limit: 2.0
  joint2:
    position: 1.57
    velocity_limit: 1.5

sensors:
  laser:
    x: 0.3
    y: 0.0
    range: 30.0
```

This maps to: `joint1_position`, `joint1_velocity_limit`, `joint2_position`, etc.

## Calibration Guidelines

### Tools Needed
- Tape measure or laser distance meter
- Level
- Reference markers on robot chassis

### Coordinate Frame
All positions are relative to `base_link`:
- **X-axis**: Forward (positive = front of robot)
- **Y-axis**: Left (positive = left side of robot)
- **Z-axis**: Up (positive = above robot)

### Measurement Process
1. Mark the `base_link` origin on the robot
2. Measure X, Y, Z distances from origin to sensor center
3. Measure orientation angles if sensor is tilted
4. Record values in robot-specific YAML file
5. Test and verify in RViz and during operation

### Typical Tolerances
- Position: ±2mm typical manufacturing tolerance
- Orientation: ±1 degree typical mounting tolerance
- Critical sensors (LiDAR, cameras): ±1mm, ±0.5 degrees

## Troubleshooting

### Configuration Not Loading
- Check YAML syntax (indentation, colons, quotes)
- Verify file is in `config/robots/` directory
- Check console output for error messages

### Sensor Position Incorrect
- Verify coordinate frame (base_link reference)
- Check sign of values (positive X = forward)
- Confirm units (meters, not millimeters)

### Launch File Issues
- Ensure `robot_id` argument is passed correctly
- Check that configuration file exists
- Verify launch file has been updated to support `robot_id`

## Version Control Best Practices

- **Commit calibration files** to repository
- **Document changes** in commit messages
- **Tag releases** when calibration is stable
- **Track calibration history** for maintenance

## Additional Resources

- [ROS 2 URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [xacro Documentation](http://wiki.ros.org/xacro)
- [TF2 Debugging](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Debugging-Tf2-Problems.html)
