import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    hunter_gazebo_pkg_dir = get_package_share_directory('hunter_pltf_gazebo')

    # Launch configurations
    world_path = LaunchConfiguration(
        'world_path', default=os.path.join(hunter_gazebo_pkg_dir, 'worlds', 'empty_world.world')
    )
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    roll = LaunchConfiguration('roll', default='0.0')
    pitch = LaunchConfiguration('pitch', default='0.0')
    yaw = LaunchConfiguration('yaw', default='1.45')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gazebo = LaunchConfiguration('use_gazebo', default='true')
    with_gui = LaunchConfiguration('with_gui', default='true')
    gz_ip = LaunchConfiguration('gz_ip', default='127.0.0.1')

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hunter_pltf_description"), "description" ,"hunter_pltf.urdf.xacro"]
            ),
            " ",
            "is_sim:=",
             use_sim_time,
             " ",
            "prefix:=''",
            " ",
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    # Create a robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': use_sim_time}]
    )

    # Gazebo stamps gpu_lidar messages with scoped SDF frame names. Publish
    # matching TF frames so RViz/Nav2 can transform the bridged scans/clouds.
    front_lidar_scoped_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '0.56',
            '--y', '0.235',
            '--z', '0.46',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'hunter_gazebo/base_link/front_lidar_link',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    back_lidar_scoped_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.56',
            '--y', '-0.235',
            '--z', '0.46',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '3.14159',
            '--frame-id', 'base_link',
            '--child-frame-id', 'hunter_gazebo/base_link/back_lidar_link',
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Start the Gazebo Sim server explicitly. In headless/container environments
    # `gz sim` may bring up only the GUI client, leaving ros_gz_sim/create
    # waiting forever for /gazebo/worlds.
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': ['-s -r -v 4 ', world_path],
        }.items(),
        condition=IfCondition(use_gazebo)
    )

    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': '-g -v 4',
        }.items(),
        condition=IfCondition(PythonExpression([
            "'", use_gazebo, "' == 'true' and '", with_gui, "' == 'true'"
        ]))
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'default',
            '-topic', 'robot_description',
            '-name', 'hunter_gazebo',
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01',
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ros_gz_bridge: bridges clock + all sensors (lidar point clouds, IMU,
    # GPS/NavSat, cameras) from gz-transport to ROS 2. The full mapping lives
    # in config/gz_bridge.yaml (see that file for per-topic documentation).
    bridge_params = os.path.join(
        hunter_gazebo_pkg_dir, 'config', 'gz_bridge.yaml'
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'],
        output='screen',
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_ackermann_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'ackermann_controller'],
        output='screen'
    )
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            os.path.join(os.path.join(get_package_share_directory('hunter_description')), 'rviz/robot_view.rviz'),
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz)
    )

   # Launch description
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
        DeclareLaunchArgument('x_pose', default_value='20.0', description='Start x position'),
        DeclareLaunchArgument('y_pose', default_value='-4.0', description='Start y position'),
        DeclareLaunchArgument('roll', default_value='0.0', description='Start roll angle'),
        DeclareLaunchArgument('pitch', default_value='0.0', description='Start pitch angle'),
        DeclareLaunchArgument('yaw', default_value='1.45', description='Start yaw angle'),
        DeclareLaunchArgument('world_path', default_value=world_path, description='Gazebo world file path'),
        DeclareLaunchArgument('use_rviz', default_value='true', description='Whether to start RViZ'),
        DeclareLaunchArgument('use_gazebo', default_value='true', description='Whether to start Gazebo'),
        DeclareLaunchArgument('with_gui', default_value='true', description='Whether to start the Gazebo GUI client'),
        DeclareLaunchArgument(
            'gz_ip',
            default_value='127.0.0.1',
            description='Gazebo Transport IP address used for local discovery',
        ),
        SetEnvironmentVariable('GZ_IP', gz_ip),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_ackermann_controller],
            )
        ),
        gazebo_server,
        gazebo_gui,
        ros_gz_bridge,
        front_lidar_scoped_tf,
        back_lidar_scoped_tf,
        rviz,
        node_robot_state_publisher,
        spawn_entity,
    ])
