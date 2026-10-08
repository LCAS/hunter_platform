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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
	gui = LaunchConfiguration("gui", default="true")
	kp_v = LaunchConfiguration('kp_v', default='40.0')
	kd_v = LaunchConfiguration('kd_v', default='0.1')
	kp_w = LaunchConfiguration('kp_w', default='35.0')
	kd_w = LaunchConfiguration("kd_w", default="0.1")
	use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="true")
	is_sim = LaunchConfiguration('is_sim', default='false')
	enable_pd_regulator = LaunchConfiguration('enable_pd_regulator', default='False')
	use_sim_time = LaunchConfiguration('use_sim_time', default='False')
	port_name = LaunchConfiguration('port_name', default='can0')
	odom_frame = LaunchConfiguration('odom_frame', default='odom')
	base_frame = LaunchConfiguration('base_frame', default='base_link')
	odom_topic_name = LaunchConfiguration('odom_topic_name', default='odom')
	cmd_vel_topic = LaunchConfiguration('cmd_vel_topic', default='/cmd_vel')
	robot_model = LaunchConfiguration('robot_model', default='hunter2')
	simulated_robot = LaunchConfiguration('simulated_robot', default='false')
	publish_odom_tf = LaunchConfiguration('publish_odom_tf', default='true')

	gui_declare = DeclareLaunchArgument(
		"gui", default_value=gui,
		description="Start RViz2 automatically with this launch file.")
	use_mock_hardware_declare = DeclareLaunchArgument(
		"use_mock_hardware", default_value=use_mock_hardware,
		description="Start robot with mock hardware mirroring command to its states.")
	use_sim_time_declare = DeclareLaunchArgument(
		'use_sim_time', default_value=use_sim_time,
		description='Use simulation clock if true')
	kp_v_val_declare = DeclareLaunchArgument(
		'kp_v', default_value=kp_v, description='Proportional gain for linear velocity')
	kd_v_val_declare = DeclareLaunchArgument(
		'kd_v', default_value=kd_v, description='Derivative gain for linear velocity')
	kp_w_val_declare = DeclareLaunchArgument(
		'kp_w', default_value=kp_w, description='Proportional gain for angular velocity')
	kd_w_val_declare = DeclareLaunchArgument(
		'kd_w', default_value=kd_w, description='Derivative gain for angular velocity')
	enable_pd_regulator_declare = DeclareLaunchArgument(
		'enable_pd_regulator', default_value=enable_pd_regulator,
		description='Use PD regulator estimate residual control to the robot')
	port_name_declare = DeclareLaunchArgument(
		'port_name', default_value=port_name, description='CAN bus name, e.g. can0')
	odom_frame_declare = DeclareLaunchArgument(
		'odom_frame', default_value=odom_frame, description='Odometry frame id')
	base_frame_declare = DeclareLaunchArgument(
		'base_frame', default_value=base_frame, description='Base link frame id')
	odom_topic_name_declare = DeclareLaunchArgument(
		'odom_topic_name', default_value=odom_topic_name, description='Odometry topic name')
	cmd_vel_topic_declare = DeclareLaunchArgument(
		'cmd_vel_topic', default_value=cmd_vel_topic, description='Command velocity topic')
	robot_model_declare = DeclareLaunchArgument(
		'robot_model', default_value=robot_model, description='Hunter base model')
	simulated_robot_declare = DeclareLaunchArgument(
		'simulated_robot', default_value=simulated_robot,
		description='Whether running with simulator')
	publish_odom_tf_declare = DeclareLaunchArgument(
		'publish_odom_tf', default_value=publish_odom_tf,
		description='Whether hunter_base broadcasts odom to base TF')

	robot_description_content = Command([
		PathJoinSubstitution([FindExecutable(name="xacro")]),
		" ",
		PathJoinSubstitution([
			FindPackageShare("hunter_pltf_description"), "description",
			"hunter_pltf.urdf.xacro"]),
		" ", "is_sim:=", is_sim, " ", "prefix:=''", " ",
	])
	robot_description = {"robot_description": robot_description_content}
	robot_controllers = PathJoinSubstitution([
		FindPackageShare("hunter_base"), "config", "hardware_controllers.yaml"])
	base_launch = os.path.join(
		get_package_share_directory("hunter_base"), "launch", "hunter_base.launch.py")

	control_node = Node(
		package="controller_manager", executable="ros2_control_node",
		parameters=[robot_description, robot_controllers], output="both")
	robot_state_pub_node = Node(
		package="robot_state_publisher", executable="robot_state_publisher",
		output="both", parameters=[robot_description])
	hunter_base_node = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(base_launch),
		launch_arguments={
			'use_sim_time': use_sim_time,
			'kp_v': kp_v,
			'kd_v': kd_v,
			'kp_w': kp_w,
			'kd_w': kd_w,
			'enable_pd_regulator': enable_pd_regulator,
			'port_name': port_name,
			'odom_frame': odom_frame,
			'base_frame': base_frame,
			'odom_topic_name': odom_topic_name,
			'cmd_vel_topic': cmd_vel_topic,
			'robot_model': robot_model,
			'simulated_robot': simulated_robot,
			'publish_odom_tf': publish_odom_tf,
		}.items(),
	)

	ld = LaunchDescription()
	for action in [
		gui_declare, use_mock_hardware_declare, kp_v_val_declare,
		kd_v_val_declare, kp_w_val_declare, kd_w_val_declare,
		enable_pd_regulator_declare, use_sim_time_declare, port_name_declare,
		odom_frame_declare, base_frame_declare, odom_topic_name_declare,
		cmd_vel_topic_declare, robot_model_declare, simulated_robot_declare,
		publish_odom_tf_declare, robot_state_pub_node, hunter_base_node,
	]:
		ld.add_action(action)

	return ld
