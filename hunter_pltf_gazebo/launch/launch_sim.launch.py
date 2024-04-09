from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory

from pathlib import Path
import os


def generate_launch_description():

    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!
    hunter_gazebo_pkg_dir = get_package_share_directory('hunter_pltf_gazebo')
    world_path = LaunchConfiguration('world_path' , default=os.path.join(hunter_gazebo_pkg_dir, 'worlds', 'empty_world.world'))
    # Launch configuration variables specific to simulation
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='1.45')
    gazebo_verbose = LaunchConfiguration('gazebo_verbose', default='true')
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('hunter_pltf_description'),'launch','pltf_rsp.launch.py'
                )]), launch_arguments={'use_sim_time': use_sim_time , 'is_sim': 'true'}.items()
    )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    gzserver = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')]),
                    launch_arguments={'world': world_path, 'verbose': gazebo_verbose, 'shell':'false'}.items())
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Run the spawner node from the gazebo_ros package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'hunter_gazebo' , 
                                   '-x', x_pose,
                                   '-y', y_pose,
                                   '-z', '0.01',     
                                   '-R', roll,
                                   '-P', pitch,
                                   '-Y', yaw],
                        output='screen')


    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
    )

    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # diffdrive_controller_spawn_callback = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_broad_spawner,
    #         on_exit=[diff_drive_spawner],
    #     )
    # )
    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'x_pose',
            default_value='20.7508434296',
            description='Start pose, x'),
        DeclareLaunchArgument(
            'y_pose',
            default_value='-4.37950954437',
            description='Start pose, y'),
        DeclareLaunchArgument(
            'roll',
            default_value='0.0',
            description='Start roll angle'),
        DeclareLaunchArgument(
            'pitch',
            default_value='0.0',
            description='Start pitch angle'),
        DeclareLaunchArgument(
            'yaw',
            default_value=yaw,
            description='Start yaw angle'),
        DeclareLaunchArgument(
            'world_path',
            default_value=world_path,
            description='Gazebo sim world'),
        DeclareLaunchArgument(
            'gazebo_verbose',
            default_value='true',
            description='Log the whole processing'),
        DeclareLaunchArgument(
            'with_gui',
            default_value='true',
            description='Run Gazebo Sim GUI'),
        rsp,
        # gazebo,
        gzserver,
        gzclient,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
    #   diffdrive_controller_spawn_callback  
    ])
