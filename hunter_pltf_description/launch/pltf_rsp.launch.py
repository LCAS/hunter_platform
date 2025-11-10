from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os
import sys
from pathlib import Path

def generate_robot_description(context, *args, **kwargs):
    """Generate robot description with robot-specific configuration."""
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    robot_id = LaunchConfiguration('robot_id').perform(context)
    
    # Import config loader
    config_dir = Path(__file__).parent.parent / 'config' / 'robots'
    sys.path.insert(0, str(config_dir))
    from config_loader import load_robot_config, get_xacro_args, format_xacro_args
    
    # Load robot configuration
    is_sim = use_sim_time.lower() == 'true'
    config = load_robot_config(robot_id)
    xacro_args = get_xacro_args(config, is_sim)
    
    # Build xacro command with all arguments
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("hunter_pltf_description"), "description", "hunter_pltf.urdf.xacro"]
    )
    
    # Format all arguments
    args_str = f"is_sim:={use_sim_time} prefix:='' {format_xacro_args(xacro_args)}"
    
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

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration("gui")
    robot_id = LaunchConfiguration('robot_id')
    
    # Note: robot_description_content will be generated in the OpaqueFunction
    # to allow runtime evaluation of robot_id

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("hunter_description"), "rviz", "robot_view.rviz"]
    )
    
    def launch_setup(context, *args, **kwargs):
        """Setup launch nodes with robot configuration."""
        robot_description_content = generate_robot_description(context)
        
        # Create a robot_state_publisher node
        params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
        node_robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[params]
        )

        joint_state_publisher_node = Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(gui),
        )

        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config_file],
            condition=IfCondition(gui),
        )
        
        return [
            node_robot_state_publisher,
            joint_state_publisher_node,
            rviz_node,
        ]

    # Launch!
    return LaunchDescription([  
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true',
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically \
        with this launch file.",
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='default',
            description='Robot instance ID (e.g., hunter_01, hunter_02). Uses robot-specific \
        configuration from config/robots/<robot_id>.yaml. Defaults to default.yaml.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
