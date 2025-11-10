# Example Usage: Robot-Specific Configuration

This document provides practical examples of using the robot configuration system.

## Quick Start

### 1. Launch with Default Configuration

```bash
# Launch robot state publisher with default sensor poses
ros2 launch hunter_pltf_description pltf_rsp.launch.py

# Launch in simulation with default configuration
ros2 launch hunter_pltf_gazebo launch_sim.launch.py

# Bringup real robot with default configuration
ros2 launch hunter_pltf_bringup hunter_pltf_bringup.launch.py
```

### 2. Launch with Hunter-01 Configuration

```bash
# Launch robot state publisher with hunter_01 calibration
ros2 launch hunter_pltf_description pltf_rsp.launch.py robot_id:=hunter_01

# Launch in simulation with hunter_01 configuration
ros2 launch hunter_pltf_gazebo launch_sim.launch.py robot_id:=hunter_01

# Bringup real robot hunter_01
ros2 launch hunter_pltf_bringup hunter_pltf_bringup.launch.py robot_id:=hunter_01
```

## Configuration File Examples

### Default Configuration (Baseline)

```yaml
# config/robots/default.yaml
robot_id: "default"

sensors:
  imu:
    x: -0.25
    y: 0.0
    z: 0.47
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
    topic: "/gps_base/yaw"
  
  front_camera:
    x: 0.55
    y: 0.0
    z: 0.72
    # ... more sensors
```

### Robot-Specific Configuration (Hunter-01)

The hunter_01 configuration only specifies differences from default:

```yaml
# config/robots/hunter_01.yaml
robot_id: "hunter_01"

sensors:
  imu:
    x: -0.251  # 1mm offset
    y: 0.001   # 1mm offset
    z: 0.471   # 1mm offset
    # roll, pitch, yaw, topic inherited from default
  
  front_camera:
    x: 0.552   # 2mm offset
    y: 0.001   # 1mm offset
    z: 0.721   # 1mm offset
    # roll, pitch, yaw inherited from default
```

## Testing Your Configuration

### 1. Validate Configuration File

```bash
cd /path/to/hunter_pltf_description/config/robots
python3 config_loader.py hunter_01 true
```

Expected output:
```
Loading configuration for: hunter_01
Simulation mode: True
------------------------------------------------------------
Robot ID: hunter_01

Sensors configured: ['imu', 'imu1', 'front_camera', 'back_camera', 'gps_base', 'front_lidar_link', 'back_lidar_link']

Xacro arguments:
  imu_x: -0.251
  imu_y: 0.001
  imu_z: 0.471
  ...
```

### 2. Visualize in RViz

```bash
# Launch with RViz to visualize sensor frames
ros2 launch hunter_pltf_description pltf_rsp.launch.py robot_id:=hunter_01 gui:=true
```

This will:
- Load the hunter_01 configuration
- Start robot_state_publisher with calibrated poses
- Open RViz to visualize the TF tree
- Show joint_state_publisher_gui

### 3. Check TF Frames

```bash
# List all frames
ros2 run tf2_ros tf2_echo base_link front_camera_link

# View entire TF tree
ros2 run tf2_tools view_frames
```

## Creating a New Robot Configuration

### Step 1: Create Configuration File

```bash
cd /path/to/hunter_pltf_description/config/robots
cp default.yaml hunter_02.yaml
```

### Step 2: Edit Configuration

```yaml
# hunter_02.yaml
robot_id: "hunter_02"

# Calibration date: 2024-11-15
# Notes: Front camera mounted 3mm higher than baseline

sensors:
  front_camera:
    z: 0.723  # Only specify what changed
```

### Step 3: Test Configuration

```bash
# Test configuration loading
python3 config_loader.py hunter_02 true

# Visualize in RViz
ros2 launch hunter_pltf_description pltf_rsp.launch.py robot_id:=hunter_02 gui:=true
```

### Step 4: Deploy

```bash
# Use in simulation
ros2 launch hunter_pltf_gazebo launch_sim.launch.py robot_id:=hunter_02

# Use on real robot
ros2 launch hunter_pltf_bringup hunter_pltf_bringup.launch.py robot_id:=hunter_02
```

## Environment Variable Alternative

You can also use environment variables:

```bash
# Set robot ID via environment
export HUNTER_ROBOT_ID=hunter_01

# Launch without robot_id argument (will use environment variable)
ros2 launch hunter_pltf_description pltf_rsp.launch.py

# Or override with argument
ros2 launch hunter_pltf_description pltf_rsp.launch.py robot_id:=hunter_02
```

## Comparison: Default vs Hunter-01

| Sensor | Parameter | Default | Hunter-01 | Difference |
|--------|-----------|---------|-----------|------------|
| IMU | x | -0.25 | -0.251 | +1mm |
| IMU | y | 0.0 | 0.001 | +1mm |
| IMU | z | 0.47 | 0.471 | +1mm |
| IMU1 | x | 0.25 | 0.249 | -1mm |
| IMU1 | y | 0.0 | -0.002 | -2mm |
| IMU1 | z | 0.47 | 0.469 | -1mm |
| Front Camera | x | 0.55 | 0.552 | +2mm |
| Front Camera | y | 0.0 | 0.001 | +1mm |
| Front Camera | z | 0.72 | 0.721 | +1mm |
| Back Camera | x | -0.55 | -0.548 | +2mm |
| Back Camera | z | 0.72 | 0.719 | -1mm |
| Front LiDAR | x | 0.56 | 0.561 | +1mm |
| Front LiDAR | y | 0.235 | 0.236 | +1mm |
| Front LiDAR | z | 0.46 | 0.461 | +1mm |
| Front LiDAR | roll (real) | -0.0174533 | -0.0175 | -0.0467° |
| Front LiDAR | pitch (real) | 0.0 | 0.001 | +0.0573° |

These small variations (±1-3mm, <1°) are typical of manufacturing tolerances and calibration adjustments.

## Troubleshooting

### Configuration Not Loading

**Symptom:** Robot uses default configuration despite specifying `robot_id`

**Solutions:**
1. Check file exists: `ls config/robots/hunter_01.yaml`
2. Validate YAML: `python3 -c "import yaml; yaml.safe_load(open('config/robots/hunter_01.yaml'))"`
3. Check console output for error messages

### Incorrect Sensor Positions

**Symptom:** Sensors appear in wrong locations in RViz

**Solutions:**
1. Verify coordinate frame (base_link reference)
2. Check sign of values (positive X = forward)
3. Confirm units are meters (not millimeters)
4. Use `ros2 run tf2_tools view_frames` to inspect TF tree

### Launch File Issues

**Symptom:** Launch fails with Python error

**Solutions:**
1. Check Python syntax: `python3 -m py_compile config_loader.py`
2. Verify config directory is installed: `ros2 pkg prefix hunter_pltf_description`
3. Check import errors in launch file output

## Advanced: Programmatic Access

You can also use the config loader in your own Python code:

```python
from config_loader import load_robot_config, get_xacro_args

# Load configuration
config = load_robot_config('hunter_01')

# Get xacro arguments for simulation
xacro_args = get_xacro_args(config, is_sim=True)

# Access specific sensor parameters
imu_x = config['sensors']['imu']['x']
print(f"IMU X position: {imu_x}m")
```

## Best Practices

1. **Always start from default.yaml** when creating new robot configs
2. **Only specify differences** to minimize duplication
3. **Document calibration** in YAML comments (date, notes, measurements)
4. **Test in simulation first** before deploying to real robot
5. **Version control** robot configs in the repository
6. **Use descriptive robot IDs** (e.g., hunter_01, hunter_lab_01)
7. **Validate before committing** using `config_loader.py` test script

## Additional Resources

- Full documentation: `README.md`
- Configuration format: `default.yaml`
- Example calibration: `hunter_01.yaml`
- Python API: `config_loader.py`
