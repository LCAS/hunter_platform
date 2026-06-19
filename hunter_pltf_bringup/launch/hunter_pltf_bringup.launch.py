import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Initialize Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_xacro_file = LaunchConfiguration('robot_xacro_file')
    is_sim = LaunchConfiguration('is_sim')
    prefix = LaunchConfiguration('prefix')
    port_name = LaunchConfiguration('port_name')
    odom_frame = LaunchConfiguration('odom_frame')
    base_frame = LaunchConfiguration('base_frame')
    odom_topic_name = LaunchConfiguration('odom_topic_name')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    robot_model = LaunchConfiguration('robot_model')
    simulated_robot = LaunchConfiguration('simulated_robot')
    control_rate = LaunchConfiguration('control_rate')

    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'robot_xacro_file',
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare('hunter_pltf_description'),
                    'description',
                    'hunter_pltf.urdf.xacro',
                ]
            ),
            description='Path to the robot URDF xacro file',
        ),
        DeclareLaunchArgument(
            'is_sim',
            default_value='false',
            description='Generate robot description for simulation if true',
        ),
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='TF/link prefix passed to the robot xacro',
        ),
        DeclareLaunchArgument(
            'port_name',
            default_value='can0',
            description='CAN interface used by hunter_base, e.g. can0',
        ),
        DeclareLaunchArgument(
            'odom_frame',
            default_value='odom',
            description='Odometry frame id published by hunter_base',
        ),
        DeclareLaunchArgument(
            'base_frame',
            default_value='base_link',
            description='Robot base frame id published by hunter_base',
        ),
        DeclareLaunchArgument(
            'odom_topic_name',
            default_value='/ackermann_controller/odometry',
            description='Odometry topic name published by hunter_base',
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='Command velocity topic consumed by hunter_base',
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='hunter2',
            description='Hunter base model parameter, e.g. hunter2 or hunter_se',
        ),
        DeclareLaunchArgument(
            'simulated_robot',
            default_value='false',
            description='Run hunter_base in simulation mode',
        ),
        DeclareLaunchArgument(
            'control_rate',
            default_value='50',
            description='hunter_base simulation control loop rate',
        ),
    ]

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            robot_xacro_file,
            " ",
            "is_sim:=",
            is_sim,
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )

    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    base_launch = os.path.join(get_package_share_directory("hunter_base"), "launch", "hunter_base.launch.py")
     
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': use_sim_time}],
    )
    
    hunter_base_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'port_name': port_name,
            'odom_frame': odom_frame,
            'base_frame': base_frame,
            'odom_topic_name': odom_topic_name,
            'cmd_vel_topic': cmd_vel_topic,
            'robot_model': robot_model,
            'simulated_robot': simulated_robot,
            'control_rate': control_rate,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_pub_node,
            hunter_base_node,
        ]
    )
