#!/usr/bin/env python3
"""
Robot Configuration Loader for Hunter Platform

This module provides utilities to load robot-specific YAML configurations
and merge them with default values. It supports inheritance where robot-specific
configurations only need to specify values that differ from the baseline.

Usage:
    from config_loader import load_robot_config, get_xacro_args
    
    config = load_robot_config('hunter_01')
    xacro_args = get_xacro_args(config, is_sim=True)
"""

import os
import yaml
from typing import Dict, Any, Optional
from pathlib import Path


def get_config_dir() -> Path:
    """Get the path to the robot configuration directory."""
    return Path(__file__).parent


def load_yaml_file(filepath: Path) -> Dict[str, Any]:
    """
    Load a YAML file and return its contents.
    
    Args:
        filepath: Path to the YAML file
        
    Returns:
        Dictionary containing the YAML contents
        
    Raises:
        FileNotFoundError: If the file doesn't exist
        yaml.YAMLError: If the file is not valid YAML
    """
    if not filepath.exists():
        raise FileNotFoundError(f"Configuration file not found: {filepath}")
    
    with open(filepath, 'r') as f:
        try:
            return yaml.safe_load(f) or {}
        except yaml.YAMLError as e:
            raise yaml.YAMLError(f"Error parsing YAML file {filepath}: {e}")


def deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    """
    Deep merge two dictionaries, with override values taking precedence.
    
    Args:
        base: Base dictionary
        override: Override dictionary
        
    Returns:
        Merged dictionary
    """
    result = base.copy()
    
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    
    return result


def load_robot_config(robot_id: str = 'default') -> Dict[str, Any]:
    """
    Load robot-specific configuration, merging with default values.
    
    Args:
        robot_id: Robot identifier (e.g., 'hunter_01', 'default')
        
    Returns:
        Complete configuration dictionary with all sensor parameters
        
    Raises:
        FileNotFoundError: If configuration files don't exist
        yaml.YAMLError: If YAML files are invalid
    """
    config_dir = get_config_dir()
    
    # Load default configuration
    default_file = config_dir / 'default.yaml'
    default_config = load_yaml_file(default_file)
    
    # If requesting default, return it directly
    if robot_id == 'default':
        return default_config
    
    # Load robot-specific configuration
    robot_file = config_dir / f'{robot_id}.yaml'
    if not robot_file.exists():
        print(f"Warning: Configuration for '{robot_id}' not found, using default")
        return default_config
    
    robot_config = load_yaml_file(robot_file)
    
    # Merge configurations (robot-specific overrides default)
    merged_config = deep_merge(default_config, robot_config)
    
    return merged_config


def get_sensor_param(config: Dict[str, Any], sensor_name: str, param_name: str, 
                     default: Any = None) -> Any:
    """
    Get a sensor parameter from the configuration.
    
    Args:
        config: Configuration dictionary
        sensor_name: Name of the sensor (e.g., 'imu', 'front_camera')
        param_name: Name of the parameter (e.g., 'x', 'y', 'z')
        default: Default value if parameter not found
        
    Returns:
        Parameter value or default
    """
    try:
        return config['sensors'][sensor_name][param_name]
    except (KeyError, TypeError):
        return default


def get_xacro_args(config: Dict[str, Any], is_sim: bool = True) -> Dict[str, str]:
    """
    Convert configuration dictionary to xacro arguments.
    
    Args:
        config: Configuration dictionary from load_robot_config()
        is_sim: Whether running in simulation mode
        
    Returns:
        Dictionary of xacro argument names and values (all as strings)
    """
    args = {}
    sensors = config.get('sensors', {})
    
    # IMU sensor
    if 'imu' in sensors:
        imu = sensors['imu']
        args['imu_x'] = str(imu.get('x', -0.25))
        args['imu_y'] = str(imu.get('y', 0.0))
        args['imu_z'] = str(imu.get('z', 0.47))
        args['imu_roll'] = str(imu.get('roll', 0.0))
        args['imu_pitch'] = str(imu.get('pitch', 0.0))
        args['imu_yaw'] = str(imu.get('yaw', 0.0))
        args['imu_topic'] = str(imu.get('topic', '/gps_base/yaw'))
    
    # IMU1 sensor
    if 'imu1' in sensors:
        imu1 = sensors['imu1']
        args['imu1_x'] = str(imu1.get('x', 0.25))
        args['imu1_y'] = str(imu1.get('y', 0.0))
        args['imu1_z'] = str(imu1.get('z', 0.47))
        args['imu1_roll'] = str(imu1.get('roll', 0.0))
        args['imu1_pitch'] = str(imu1.get('pitch', 0.0))
        args['imu1_yaw'] = str(imu1.get('yaw', 0.0))
        args['imu1_topic'] = str(imu1.get('topic', '/imu/data'))
    
    # Front camera
    if 'front_camera' in sensors:
        cam = sensors['front_camera']
        args['front_camera_x'] = str(cam.get('x', 0.55))
        args['front_camera_y'] = str(cam.get('y', 0.0))
        args['front_camera_z'] = str(cam.get('z', 0.72))
        args['front_camera_roll'] = str(cam.get('roll', 0.0))
        args['front_camera_pitch'] = str(cam.get('pitch', 0.0))
        args['front_camera_yaw'] = str(cam.get('yaw', 0.0))
    
    # Back camera
    if 'back_camera' in sensors:
        cam = sensors['back_camera']
        args['back_camera_x'] = str(cam.get('x', -0.55))
        args['back_camera_y'] = str(cam.get('y', 0.0))
        args['back_camera_z'] = str(cam.get('z', 0.72))
        args['back_camera_roll'] = str(cam.get('roll', 0.0))
        args['back_camera_pitch'] = str(cam.get('pitch', 0.0))
        args['back_camera_yaw'] = str(cam.get('yaw', 3.14159265359))
    
    # GPS base
    if 'gps_base' in sensors:
        gps = sensors['gps_base']
        args['gps_base_x'] = str(gps.get('x', -0.25))
        args['gps_base_y'] = str(gps.get('y', 0.0))
        args['gps_base_z'] = str(gps.get('z', 0.47))
        args['gps_base_roll'] = str(gps.get('roll', 0.0))
        args['gps_base_pitch'] = str(gps.get('pitch', 0.0))
        args['gps_base_yaw'] = str(gps.get('yaw', 0.0))
    
    # Front LiDAR
    if 'front_lidar_link' in sensors:
        lidar = sensors['front_lidar_link']
        args['front_lidar_x'] = str(lidar.get('x', 0.56))
        args['front_lidar_y'] = str(lidar.get('y', 0.235))
        args['front_lidar_z'] = str(lidar.get('z', 0.46))
        args['front_lidar_topic'] = str(lidar.get('topic', 'front_lidar/points'))
        
        # Sim vs real orientation
        if is_sim:
            args['front_lidar_roll_sim'] = str(lidar.get('roll_sim', 0.0))
            args['front_lidar_pitch_sim'] = str(lidar.get('pitch_sim', 0.0))
            args['front_lidar_yaw_sim'] = str(lidar.get('yaw_sim', 0.0))
        else:
            args['front_lidar_roll_real'] = str(lidar.get('roll_real', -0.0174533))
            args['front_lidar_pitch_real'] = str(lidar.get('pitch_real', 0.0))
            args['front_lidar_yaw_real'] = str(lidar.get('yaw_real', 0.00872665))
    
    # Back LiDAR
    if 'back_lidar_link' in sensors:
        lidar = sensors['back_lidar_link']
        args['back_lidar_x'] = str(lidar.get('x', -0.56))
        args['back_lidar_y'] = str(lidar.get('y', -0.235))
        args['back_lidar_z'] = str(lidar.get('z', 0.46))
        args['back_lidar_topic'] = str(lidar.get('topic', 'back_lidar/points'))
        
        # Sim vs real orientation
        if is_sim:
            args['back_lidar_roll_sim'] = str(lidar.get('roll_sim', 0.0))
            args['back_lidar_pitch_sim'] = str(lidar.get('pitch_sim', 0.0))
            args['back_lidar_yaw_sim'] = str(lidar.get('yaw_sim', 3.14159265359))
        else:
            args['back_lidar_roll_real'] = str(lidar.get('roll_real', 0.0))
            args['back_lidar_pitch_real'] = str(lidar.get('pitch_real', 0.0))
            args['back_lidar_yaw_real'] = str(lidar.get('yaw_real', 3.13286335))
    
    return args


def format_xacro_args(args: Dict[str, str]) -> str:
    """
    Format xacro arguments as a command-line string.
    
    Args:
        args: Dictionary of argument names and values
        
    Returns:
        Formatted string for use in xacro command
    """
    return ' '.join([f'{key}:={value}' for key, value in args.items()])


if __name__ == '__main__':
    # Test the configuration loader
    import sys
    
    robot_id = sys.argv[1] if len(sys.argv) > 1 else 'default'
    is_sim = sys.argv[2].lower() == 'true' if len(sys.argv) > 2 else True
    
    print(f"Loading configuration for: {robot_id}")
    print(f"Simulation mode: {is_sim}")
    print("-" * 60)
    
    try:
        config = load_robot_config(robot_id)
        print(f"Robot ID: {config.get('robot_id', 'unknown')}")
        print(f"\nSensors configured: {list(config.get('sensors', {}).keys())}")
        
        print(f"\nXacro arguments:")
        xacro_args = get_xacro_args(config, is_sim)
        for key, value in sorted(xacro_args.items()):
            print(f"  {key}: {value}")
        
        print(f"\nCommand line format:")
        print(f"  {format_xacro_args(xacro_args)}")
        
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
