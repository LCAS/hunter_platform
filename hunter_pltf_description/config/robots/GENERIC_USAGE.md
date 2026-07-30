# Generic Configuration System - Usage for Any Robot

This document demonstrates how the configuration system can be used with **any robot**, not just Hunter platform.

## Overview

The configuration system is **completely generic** and reusable. It automatically maps nested YAML structures to flat xacro arguments, making it suitable for any ROS robot.

## Example 1: Simple Mobile Robot

### YAML Configuration

```yaml
# my_mobile_robot.yaml
robot_id: "mobile_bot_01"

sensors:
  laser:
    x: 0.15
    y: 0.0
    z: 0.25
    range: 30.0
    angle: 3.14159
  
  camera:
    x: 0.20
    y: 0.0
    z: 0.30
    fov: 1.57
    resolution: "640x480"

wheels:
  left:
    radius: 0.1
    width: 0.05
  right:
    radius: 0.1
    width: 0.05
```

### Generated Xacro Arguments

The system automatically generates:

```bash
laser_x:=0.15
laser_y:=0.0
laser_z:=0.25
laser_range:=30.0
laser_angle:=3.14159
camera_x:=0.20
camera_y:=0.0
camera_z:=0.30
camera_fov:=1.57
camera_resolution:=640x480
left_radius:=0.1
left_width:=0.05
right_radius:=0.1
right_width:=0.05
```

### URDF Xacro File

Your xacro file would define these arguments:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://ros.org/wiki/xacro" name="mobile_bot">
  <!-- Arguments automatically populated from YAML -->
  <xacro:arg name="laser_x" default="0.15"/>
  <xacro:arg name="laser_y" default="0.0"/>
  <xacro:arg name="laser_z" default="0.25"/>
  <xacro:arg name="laser_range" default="30.0"/>
  <!-- ... etc ... -->
  
  <!-- Use arguments in your URDF -->
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="$(arg laser_x) $(arg laser_y) $(arg laser_z)"/>
  </joint>
</robot>
```

## Example 2: Robotic Manipulator

### YAML Configuration

```yaml
# manipulator.yaml
robot_id: "arm_01"

joints:
  shoulder_pan:
    position: 0.0
    velocity_limit: 2.0
    effort_limit: 100.0
  shoulder_lift:
    position: -1.57
    velocity_limit: 2.0
    effort_limit: 100.0
  elbow:
    position: 1.57
    velocity_limit: 2.0
    effort_limit: 50.0
  wrist_1:
    position: 0.0
    velocity_limit: 3.0
    effort_limit: 25.0

gripper:
  max_opening: 0.08
  force_limit: 20.0
```

### Generated Xacro Arguments

```bash
shoulder_pan_position:=0.0
shoulder_pan_velocity_limit:=2.0
shoulder_pan_effort_limit:=100.0
shoulder_lift_position:=-1.57
shoulder_lift_velocity_limit:=2.0
shoulder_lift_effort_limit:=100.0
elbow_position:=1.57
elbow_velocity_limit:=2.0
elbow_effort_limit:=50.0
wrist_1_position:=0.0
wrist_1_velocity_limit:=3.0
wrist_1_effort_limit:=25.0
max_opening:=0.08
force_limit:=20.0
```

## Example 3: Drone with Multiple Cameras

### YAML Configuration

```yaml
# drone.yaml
robot_id: "drone_alpha"

cameras:
  front:
    x: 0.15
    y: 0.0
    z: 0.02
    pitch: 0.0
    fov: 1.91
    resolution: "1920x1080"
  
  bottom:
    x: 0.0
    y: 0.0
    z: -0.05
    pitch: -1.57
    fov: 1.57
    resolution: "640x480"
  
  gimbal:
    x: 0.0
    y: 0.0
    z: -0.10
    pitch_range: 1.57
    yaw_range: 3.14
    stabilized: true

propellers:
  diameter: 0.25
  thrust_coefficient: 8.5e-6
  count: 4
```

### Generated Xacro Arguments

```bash
front_x:=0.15
front_y:=0.0
front_z:=0.02
front_pitch:=0.0
front_fov:=1.91
front_resolution:=1920x1080
bottom_x:=0.0
bottom_y:=0.0
bottom_z:=-0.05
bottom_pitch:=-1.57
bottom_fov:=1.57
bottom_resolution:=640x480
gimbal_x:=0.0
gimbal_y:=0.0
gimbal_z:=-0.10
gimbal_pitch_range:=1.57
gimbal_yaw_range:=3.14
gimbal_stabilized:=True
diameter:=0.25
thrust_coefficient:=8.5e-06
count:=4
```

## Loading Configurations from Different Sources

### Local File (Simple Robot ID)

```python
from config_loader import load_robot_config, get_xacro_args_from_config

# Looks in config/robots/mobile_bot_01.yaml
config = load_robot_config('mobile_bot_01')
args = get_xacro_args_from_config(config, prefix='sensors')
```

### Absolute Path

```python
# Load from anywhere on filesystem
config = load_robot_config('/opt/robot_configs/production/arm_01.yaml')
args = get_xacro_args_from_config(config, prefix='joints')
```

### Remote URL (HTTPS)

```python
# Load from central configuration server
config = load_robot_config('https://robots.company.com/configs/drone_alpha.yaml')
args = get_xacro_args_from_config(config, prefix='cameras')
```

## Customizing the Mapping

### Using Different Prefixes

```python
# Extract from 'joints' section instead of 'sensors'
config = load_robot_config('manipulator')
args = get_xacro_args_from_config(config, prefix='joints')
```

### No Prefix (Top-Level Mapping)

```python
# Map entire YAML structure
config = load_robot_config('simple_robot')
args = get_xacro_args_from_config(config, prefix=None)
```

### Excluding Keys

```python
# Exclude metadata from xacro arguments
config = load_robot_config('myrobot')
args = get_xacro_args_from_config(
    config, 
    prefix='sensors',
    exclude_keys=['robot_id', 'calibration_date', 'notes']
)
```

## Simulation vs Real Hardware

The system automatically handles `_sim` and `_real` suffixes:

### YAML Configuration

```yaml
sensors:
  lidar:
    x: 0.3
    y: 0.0
    z: 0.5
    # Different values for sim vs real
    roll_sim: 0.0
    roll_real: -0.0175
    noise_sim: 0.01
    noise_real: 0.05
```

### Generated Arguments (is_sim=True)

```bash
lidar_x:=0.3
lidar_y:=0.0
lidar_z:=0.5
lidar_roll_sim:=0.0
lidar_noise_sim:=0.01
```

### Generated Arguments (is_sim=False)

```bash
lidar_x:=0.3
lidar_y:=0.0
lidar_z:=0.5
lidar_roll_real:=-0.0175
lidar_noise_real:=0.05
```

## Integration with Launch Files

### Python Launch File Example

```python
from launch import LaunchDescription
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from pathlib import Path
import sys

def generate_robot_description(context, *args, **kwargs):
    robot_config_uri = LaunchConfiguration('robot_config').perform(context)
    is_sim = LaunchConfiguration('is_sim').perform(context).lower() == 'true'
    
    # Import config loader
    config_dir = Path(__file__).parent / 'config' / 'robots'
    sys.path.insert(0, str(config_dir))
    from config_loader import load_robot_config, get_xacro_args_from_config, format_xacro_args
    
    # Load configuration (works with any URI)
    config = load_robot_config(robot_config_uri)
    
    # Generate xacro arguments (customize prefix for your robot)
    xacro_args = get_xacro_args_from_config(config, is_sim=is_sim, prefix='sensors')
    
    # Use in xacro command
    robot_description_content = Command([
        'xacro',
        ' ',
        'path/to/your/robot.urdf.xacro',
        ' ',
        format_xacro_args(xacro_args)
    ])
    
    return robot_description_content

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_config',
            default_value='default',
            description='Robot configuration (ID, path, or URL)'
        ),
        DeclareLaunchArgument(
            'is_sim',
            default_value='true',
            description='Simulation mode'
        ),
        OpaqueFunction(function=generate_robot_description)
    ])
```

### Usage

```bash
# Simple robot ID
ros2 launch my_robot robot.launch.py robot_config:=mobile_bot_01

# Absolute path
ros2 launch my_robot robot.launch.py robot_config:=/path/to/config.yaml

# Remote URL
ros2 launch my_robot robot.launch.py robot_config:=https://server.com/config.yaml
```

## Best Practices

### 1. Structure Your YAML to Match Xacro

Organize your YAML to mirror your xacro argument naming:

```yaml
# If xacro has: camera_front_x, camera_front_y
sensors:
  camera_front:
    x: 0.5
    y: 0.0
```

### 2. Use Descriptive Keys

Make keys self-documenting:

```yaml
sensors:
  laser_scanner:  # Not just "laser"
    max_range: 30.0
    min_range: 0.1
```

### 3. Group Related Parameters

Use nested structures for organization:

```yaml
gripper:
  fingers:
    left:
      length: 0.1
      width: 0.02
    right:
      length: 0.1
      width: 0.02
```

### 4. Document Units

Include units in comments:

```yaml
sensors:
  camera:
    x: 0.15  # meters
    fov: 1.57  # radians
    framerate: 30  # Hz
```

## Validation

Test your configuration before deployment:

```bash
# Test configuration loading
python3 config_loader.py /path/to/your/robot.yaml true

# Verify xacro argument generation
python3 config_loader.py https://server.com/robot.yaml false
```

## Summary

The configuration system is **completely generic** and works with:
- ✅ Any robot type (mobile, manipulator, drone, etc.)
- ✅ Any YAML structure (sensors, joints, links, etc.)
- ✅ Any source location (local, remote, URLs)
- ✅ Any URDF/xacro structure
- ✅ Simulation and real hardware modes

It automatically handles:
- ✅ Flattening nested structures
- ✅ Type conversion to strings
- ✅ Simulation vs real hardware variants
- ✅ Inheritance and overrides
- ✅ URI resolution (paths, URLs)

No modifications needed for different robots - just structure your YAML appropriately!
