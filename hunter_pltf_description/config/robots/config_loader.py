#!/usr/bin/env python3
"""
Generic Robot Configuration Loader

This module provides utilities to load robot-specific YAML configurations
from various sources (local files, remote URLs) and automatically map them
to xacro arguments. It supports inheritance where robot-specific
configurations only need to specify values that differ from the baseline.

Usage:
    from config_loader import load_robot_config, get_xacro_args_from_config
    
    # Load from local file, URI, or URL
    config = load_robot_config('hunter_01')  # Local in config/robots/
    config = load_robot_config('/path/to/config.yaml')  # Absolute path
    config = load_robot_config('https://example.com/robot.yaml')  # Remote URL
    
    # Automatically map to xacro arguments
    xacro_args = get_xacro_args_from_config(config, is_sim=True)
"""

import os
import yaml
import urllib.request
import urllib.parse
from typing import Dict, Any, Optional, Union
from pathlib import Path


def get_config_dir() -> Path:
    """Get the path to the robot configuration directory."""
    return Path(__file__).parent


def load_yaml_from_uri(uri: str) -> Dict[str, Any]:
    """
    Load a YAML file from a URI (local file path, file:// URL, or https:// URL).
    
    Args:
        uri: URI to the YAML file. Can be:
             - Absolute file path: /path/to/file.yaml
             - Relative file path: relative/path.yaml
             - File URL: file:///path/to/file.yaml
             - HTTPS URL: https://example.com/config.yaml
        
    Returns:
        Dictionary containing the YAML contents
        
    Raises:
        FileNotFoundError: If the file doesn't exist (local files)
        urllib.error.URLError: If URL cannot be accessed
        yaml.YAMLError: If the content is not valid YAML
    """
    parsed = urllib.parse.urlparse(uri)
    
    # Handle HTTPS URLs
    if parsed.scheme in ('http', 'https'):
        try:
            with urllib.request.urlopen(uri, timeout=10) as response:
                content = response.read().decode('utf-8')
                return yaml.safe_load(content) or {}
        except urllib.error.URLError as e:
            raise urllib.error.URLError(f"Failed to fetch configuration from {uri}: {e}")
        except yaml.YAMLError as e:
            raise yaml.YAMLError(f"Error parsing YAML from {uri}: {e}")
    
    # Handle file:// URLs and local paths
    if parsed.scheme == 'file':
        filepath = Path(parsed.path)
    else:
        filepath = Path(uri)
    
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


def load_robot_config(robot_id_or_uri: str = 'default', 
                      default_config_uri: Optional[str] = None) -> Dict[str, Any]:
    """
    Load robot-specific configuration, merging with default values.
    
    This function supports loading configurations from:
    - Simple robot ID (looks in config/robots/ directory)
    - Absolute or relative file paths
    - file:// URLs
    - https:// URLs
    
    Args:
        robot_id_or_uri: Robot identifier, file path, or URL. Examples:
                         - 'hunter_01' -> loads config/robots/hunter_01.yaml
                         - '/path/to/robot.yaml' -> loads from absolute path
                         - 'file:///path/to/robot.yaml' -> loads from file URL
                         - 'https://example.com/robot.yaml' -> loads from HTTPS
        default_config_uri: Optional URI to default configuration. If not provided,
                           uses 'default.yaml' from config/robots/ directory.
        
    Returns:
        Complete configuration dictionary with all sensor parameters
        
    Raises:
        FileNotFoundError: If configuration files don't exist
        urllib.error.URLError: If URL cannot be accessed
        yaml.YAMLError: If YAML files are invalid
    """
    # Determine if input is a URI (path or URL) or just a robot ID
    parsed = urllib.parse.urlparse(robot_id_or_uri)
    is_uri = (parsed.scheme in ('http', 'https', 'file') or 
              '/' in robot_id_or_uri or 
              '\\' in robot_id_or_uri or
              Path(robot_id_or_uri).exists())
    
    # Load default configuration
    if default_config_uri:
        default_config = load_yaml_from_uri(default_config_uri)
    else:
        config_dir = get_config_dir()
        default_file = config_dir / 'default.yaml'
        default_config = load_yaml_from_uri(str(default_file))
    
    # If requesting default by ID, return it directly
    if robot_id_or_uri == 'default' and not is_uri:
        return default_config
    
    # Load robot-specific configuration
    if is_uri:
        # Direct URI provided
        robot_config = load_yaml_from_uri(robot_id_or_uri)
    else:
        # Simple robot ID - look in config/robots/ directory
        config_dir = get_config_dir()
        robot_file = config_dir / f'{robot_id_or_uri}.yaml'
        if not robot_file.exists():
            print(f"Warning: Configuration for '{robot_id_or_uri}' not found, using default")
            return default_config
        robot_config = load_yaml_from_uri(str(robot_file))
    
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


def flatten_dict(d: Dict[str, Any], parent_key: str = '', sep: str = '_') -> Dict[str, Any]:
    """
    Flatten a nested dictionary into a single-level dictionary with concatenated keys.
    
    Args:
        d: Dictionary to flatten
        parent_key: Parent key for recursion
        sep: Separator between keys
        
    Returns:
        Flattened dictionary
        
    Example:
        {'sensors': {'imu': {'x': 1.0, 'y': 2.0}}}
        -> {'sensors_imu_x': 1.0, 'sensors_imu_y': 2.0}
    """
    items = []
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.extend(flatten_dict(v, new_key, sep=sep).items())
        else:
            items.append((new_key, v))
    return dict(items)


def get_xacro_args_from_config(config: Dict[str, Any], 
                                 is_sim: bool = True,
                                 prefix: str = 'sensors',
                                 exclude_keys: Optional[list] = None) -> Dict[str, str]:
    """
    Generic converter: automatically map nested YAML configuration to xacro arguments.
    
    This function flattens the configuration dictionary and converts it to xacro
    argument format. It's designed to work with any URDF structure, not just
    specific hardcoded sensor names.
    
    Args:
        config: Configuration dictionary from load_robot_config()
        is_sim: Whether running in simulation mode (affects _sim/_real suffix handling)
        prefix: Top-level key to extract and flatten (default: 'sensors')
        exclude_keys: List of keys to exclude from the output (e.g., ['robot_id'])
        
    Returns:
        Dictionary of xacro argument names and values (all as strings)
        
    Example:
        Input config:
        {
            'sensors': {
                'imu': {'x': -0.25, 'y': 0.0, 'topic': '/imu'},
                'camera': {'x': 0.5, 'roll_sim': 0.0, 'roll_real': 0.1}
            }
        }
        
        Output (is_sim=True):
        {
            'imu_x': '-0.25',
            'imu_y': '0.0', 
            'imu_topic': '/imu',
            'camera_x': '0.5',
            'camera_roll_sim': '0.0'
        }
    """
    if exclude_keys is None:
        exclude_keys = ['robot_id']
    
    # Extract the section to process (e.g., 'sensors')
    data_to_process = config.get(prefix, {}) if prefix else config
    
    # Flatten the nested dictionary
    flattened = flatten_dict(data_to_process)
    
    # Convert to xacro arguments
    args = {}
    for key, value in flattened.items():
        # Skip excluded keys
        if any(excluded in key for excluded in exclude_keys):
            continue
        
        # Handle sim/real variants - only include the relevant one
        if '_sim' in key and not is_sim:
            continue  # Skip _sim keys when not in simulation
        if '_real' in key and is_sim:
            continue  # Skip _real keys when in simulation
        
        # Convert value to string
        args[key] = str(value)
    
    return args


# Backward compatibility: keep old function name but redirect to new one
def get_xacro_args(config: Dict[str, Any], is_sim: bool = True) -> Dict[str, str]:
    """
    Convert configuration dictionary to xacro arguments (backward compatibility).
    
    This function is maintained for backward compatibility. New code should use
    get_xacro_args_from_config() which is more flexible and generic.
    
    Args:
        config: Configuration dictionary from load_robot_config()
        is_sim: Whether running in simulation mode
        
    Returns:
        Dictionary of xacro argument names and values (all as strings)
    """
    return get_xacro_args_from_config(config, is_sim=is_sim, prefix='sensors')


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
    
    robot_id_or_uri = sys.argv[1] if len(sys.argv) > 1 else 'default'
    is_sim = sys.argv[2].lower() == 'true' if len(sys.argv) > 2 else True
    
    print(f"Loading configuration for: {robot_id_or_uri}")
    print(f"Simulation mode: {is_sim}")
    print("-" * 60)
    
    try:
        config = load_robot_config(robot_id_or_uri)
        print(f"Robot ID: {config.get('robot_id', 'unknown')}")
        print(f"\nSensors configured: {list(config.get('sensors', {}).keys())}")
        
        print(f"\nXacro arguments (generic mapping):")
        xacro_args = get_xacro_args_from_config(config, is_sim)
        for key, value in sorted(xacro_args.items()):
            print(f"  {key}: {value}")
        
        print(f"\nCommand line format:")
        print(f"  {format_xacro_args(xacro_args)}")
        
        # Show that we can load from URIs
        if robot_id_or_uri not in ['default', 'hunter_01']:
            print(f"\nNote: Loaded from URI: {robot_id_or_uri}")
        
    except Exception as e:
        import traceback
        print(f"Error: {e}", file=sys.stderr)
        traceback.print_exc()
        sys.exit(1)
