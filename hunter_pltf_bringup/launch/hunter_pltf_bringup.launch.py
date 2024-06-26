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
from launch.actions import DeclareLaunchArgument, RegisterEventHandler , IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
import ament_index_python.packages
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml
import os
 
def generate_launch_description():
    # Declare arguments
    
    gps_config_directory = os.path.join( ament_index_python.packages
                                        .get_package_share_directory('hunter_pltf_bringup'),'config')
    gps_param_config = os.path.join(gps_config_directory, 'gps_config.yaml')
    
    gps_config_file = LaunchConfiguration(
        'gps_config', default=gps_param_config)
    
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="true",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
 
 
    declared_arguments.append(DeclareLaunchArgument(
            'gps_config', default_value=gps_config_file , description='config file for gps'))
    # Initialize Arguments
    # gui = LaunchConfiguration("gui")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    is_sim = LaunchConfiguration('is_sim' , default='false')
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("hunter_pltf_description"), "description" ,"hunter_pltf.urdf.xacro"]
            ),
            " ",
            "is_sim:=",
             is_sim,
             " ",
            "prefix:=''",
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}
 
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("hunter_base"),
            "config",
            "hardware_controllers.yaml",
        ]
    )

    base_launch = os.path.join(get_package_share_directory("hunter_base"), "launch", "hunter_base.launch.py")
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("ros2_control_demo_description"), "diffbot/rviz", "diffbot.rviz"]
    # t
 
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
        # remappings=[
            # ("/diff_drive_controller/cmd_vel_unstamped", "/cmd_vel"),
        # ],
    )


    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="log",
    #     arguments=["-d", rviz_config_file],
    #     condition=IfCondition(gui),
    # )
 
    # joint_state_broadcaster_spawner = Node(
        # package="controller_manager",
        # executable="spawner",
        # arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )
 
    # robot_controller_spawner = Node(
        # package="controller_manager",
        # executable="spawner",
        # arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
    # )
 
    # Delay rviz start after `joint_state_broadcaster`
    # delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #         target_action=joint_state_broadcaster_spawner,
    #         on_exit=[rviz_node],
    #     )
    # )
 
    # Delay start of robot_controller after `joint_state_broadcaster`
    # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        # event_handler=OnProcessExit(
            # target_action=joint_state_broadcaster_spawner,
            # on_exit=[robot_controller_spawner],
        # )
    # )
    # --------------------- SENSOR BRINGUPS ----------
    nodes = [
        # control_node,
        robot_state_pub_node,
        # joint_state_broadcaster_spawner,
        # delay_rviz_after_joint_state_broadcaster_spawner,
        # delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
    ]
 
    launches=  [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch)
        ),]
    return LaunchDescription(declared_arguments +nodes  + launches)