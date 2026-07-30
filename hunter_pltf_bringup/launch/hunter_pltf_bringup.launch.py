# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
import sys
from pathlib import Path
 
def generate_launch_description():
    
    # Initialize Arguments
    gui = LaunchConfiguration("gui", default="true")
    kp_v = LaunchConfiguration('kp_v', default='40.0')
    kd_v = LaunchConfiguration('kd_v', default='0.1') 
    kp_w = LaunchConfiguration('kp_w', default='35.0')
    kd_w = LaunchConfiguration("kd_w", default="0.1")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="true")
    is_sim = LaunchConfiguration('is_sim' , default='false')
    enable_pd_regulator = LaunchConfiguration('enable_pd_regulator', default='False')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    robot_id = LaunchConfiguration('robot_id', default='default')
  
    gui_declare = DeclareLaunchArgument(
            "gui", default_value=gui, description="Start RViz2 automatically with this launch file.")
   
    # IH: THIS PARAM IS NOT USED
    use_mock_hardware_declare = DeclareLaunchArgument(
            "use_mock_hardware", default_value=use_mock_hardware,description="Start robot with mock hardware mirroring command to its states.")
    
    use_sim_time_declare = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                                                    description='Use simulation clock if true')
    kp_v_val_declare = DeclareLaunchArgument('kp_v', default_value=kp_v, description='Proportional gain for linear velocity')
    kd_v_val_declare = DeclareLaunchArgument('kd_v', default_value=kd_v, description='Derivative gain for linear velocity')
    kp_w_val_declare = DeclareLaunchArgument('kp_w', default_value=kp_w, description='Proportional gain for angular velocity')
    kd_w_val_declare = DeclareLaunchArgument('kd_w', default_value=kd_w, description='Derivative gain for angular velocity')
    enable_pd_regulator_declare = DeclareLaunchArgument('enable_pd_regulator', default_value=enable_pd_regulator
        , description='Use PD regulator estimate residual control to the robot')
    
    def generate_robot_description(context, *args, **kwargs):
        """Generate robot description with robot-specific configuration."""
        is_sim_val = LaunchConfiguration('is_sim').perform(context)
        robot_id_val = LaunchConfiguration('robot_id').perform(context)
        
        # Import config loader
        config_dir = Path(get_package_share_directory('hunter_pltf_description')) / 'config' / 'robots'
        sys.path.insert(0, str(config_dir))
        from config_loader import load_robot_config, get_xacro_args, format_xacro_args
        
        # Load robot configuration
        is_sim_bool = is_sim_val.lower() == 'true'
        config = load_robot_config(robot_id_val)
        xacro_args = get_xacro_args(config, is_sim_bool)
        
        # Build xacro command with all arguments
        xacro_file = PathJoinSubstitution(
            [FindPackageShare("hunter_pltf_description"), "description", "hunter_pltf.urdf.xacro"]
        )
        
        # Format all arguments
        args_str = f"is_sim:={is_sim_val} prefix:='' {format_xacro_args(xacro_args)}"
        
        robot_description_content = Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                xacro_file,
                " ",
                args_str,
            ]
        )
        
        return robot_description_content
    
    # Note: robot_description will be generated in the OpaqueFunction
    # to allow runtime evaluation of robot_id
 
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hunter_base"),
            "config",
            "hardware_controllers.yaml",
        ]
    )

    base_launch = os.path.join(get_package_share_directory("hunter_base"), "launch", "hunter_base.launch.py")
 
    def launch_setup(context, *args, **kwargs):
        """Setup launch nodes with robot configuration."""
        robot_description_content = generate_robot_description(context)
        robot_description = {"robot_description": robot_description_content}
        
        control_node = Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[robot_description, robot_controllers],
            output="both",
        )
        
        robot_state_pub_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[robot_description],
        )
        
        hunter_base_node = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(base_launch),
                launch_arguments={
                    'use_sim_time': use_sim_time,
                    'kp_v': kp_v,
                    'kd_v': kd_v,
                    'kp_w': kp_w,
                    'kd_w': kd_w,
                    'enable_pd_regulator': enable_pd_regulator
                    }.items(),
        )
        
        return [
            robot_state_pub_node,
            hunter_base_node,
        ]
    
    robot_id_declare = DeclareLaunchArgument(
            'robot_id', default_value='default',
            description='Robot instance ID (e.g., hunter_01, hunter_02). Uses robot-specific \
        configuration from config/robots/<robot_id>.yaml. Defaults to default.yaml.')
    
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(gui_declare)
    ld.add_action(use_mock_hardware_declare)
    ld.add_action(kp_v_val_declare)
    ld.add_action(kd_v_val_declare)
    ld.add_action(kp_w_val_declare)
    ld.add_action(kd_w_val_declare)
    ld.add_action(enable_pd_regulator_declare)
    ld.add_action(use_sim_time_declare)
    ld.add_action(robot_id_declare)
    
    ld.add_action(OpaqueFunction(function=launch_setup))
   
    return ld