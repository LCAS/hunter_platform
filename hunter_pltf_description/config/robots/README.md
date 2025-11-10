# Hunter Platform Robot Configuration Files

This directory contains robot-specific configuration files for individual Hunter platform instances. Each robot has unique sensor mounting positions due to manufacturing tolerances and calibration requirements.

## Overview

The configuration system allows you to:
- Define baseline sensor poses in `default.yaml`
- Create robot-specific overrides in separate YAML files (e.g., `hunter_01.yaml`)
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

### Sensor Names

The following sensors are configurable:

- **`imu`**: Rear IMU sensor (at GPS base location)
- **`imu1`**: Front IMU sensor
- **`front_camera`**: Front depth camera
- **`back_camera`**: Rear depth camera
- **`gps_base`**: GPS base antenna
- **`front_lidar_link`**: Front Mid-360 LiDAR
- **`back_lidar_link`**: Rear Mid-360 LiDAR

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

Specify the robot configuration using the `robot_id` argument:

```bash
# Using default configuration
ros2 launch hunter_pltf_description pltf_rsp.launch.py

# Using hunter_01 configuration
ros2 launch hunter_pltf_description pltf_rsp.launch.py robot_id:=hunter_01

# Using hunter_01 for bringup
ros2 launch hunter_pltf_bringup hunter_pltf_bringup.launch.py robot_id:=hunter_01

# Using hunter_01 for simulation
ros2 launch hunter_pltf_gazebo launch_sim.launch.py robot_id:=hunter_01
```

### Environment Variable (Alternative)

You can also set the robot ID via environment variable:

```bash
export HUNTER_ROBOT_ID=hunter_01
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
