#!/usr/bin/env python3
import math
from copy import deepcopy

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_msgs.msg import TFMessage


def _quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _yaw_to_quaternion(yaw):
    half_yaw = 0.5 * yaw
    q = Imu().orientation
    q.z = math.sin(half_yaw)
    q.w = math.cos(half_yaw)
    return q


class GazeboPoseYawImu(Node):
    def __init__(self):
        super().__init__('gazebo_pose_yaw_imu')

        self.declare_parameter('input_topic', '/gazebo/dynamic_pose/info')
        self.declare_parameter('output_topic', '/gps_base/yaw')
        self.declare_parameter('source_child_frame_id', 'hunter_gazebo')
        self.declare_parameter('frame_id', 'hunter_gazebo/base_link/imu_imu')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('yaw_variance', 1.0e-4)
        self.declare_parameter('fallback_to_first_transform', True)
        self.declare_parameter('gps_odom_input_topic', '')
        self.declare_parameter('gps_odom_output_topic', '/gps_base/odometry')
        self.declare_parameter('gps_odom_frame_id', 'map')
        self.declare_parameter('gps_odom_child_frame_id', 'base_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        gps_odom_input_topic = self.get_parameter('gps_odom_input_topic').value
        gps_odom_output_topic = self.get_parameter('gps_odom_output_topic').value
        self.source_child_frame_id = self.get_parameter('source_child_frame_id').value
        self.frame_id = self.get_parameter('frame_id').value
        publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.yaw_variance = float(self.get_parameter('yaw_variance').value)
        self.fallback_to_first_transform = bool(
            self.get_parameter('fallback_to_first_transform').value
        )
        self.gps_odom_frame_id = self.get_parameter('gps_odom_frame_id').value
        self.gps_odom_child_frame_id = self.get_parameter(
            'gps_odom_child_frame_id'
        ).value

        self.last_transform = None
        self.logged_fallback = False
        self.publisher = self.create_publisher(Imu, output_topic, 10)
        self.subscription = self.create_subscription(
            TFMessage,
            input_topic,
            self._tf_callback,
            10,
        )
        self.timer = self.create_timer(1.0 / publish_rate_hz, self._publish)

        self.gps_odom_publisher = None
        self.gps_odom_subscription = None
        if gps_odom_input_topic:
            self.gps_odom_publisher = self.create_publisher(
                Odometry,
                gps_odom_output_topic,
                10,
            )
            self.gps_odom_subscription = self.create_subscription(
                Odometry,
                gps_odom_input_topic,
                self._gps_odom_callback,
                10,
            )

        self.get_logger().info(
            f'Publishing {output_topic} yaw from {input_topic} frame '
            f'{self.source_child_frame_id}'
        )
        if gps_odom_input_topic:
            self.get_logger().info(
                f'Republishing GPS odometry from {gps_odom_input_topic} to '
                f'{gps_odom_output_topic} in frame {self.gps_odom_frame_id}'
            )

    def _tf_callback(self, msg):
        for transform in msg.transforms:
            if self._matches_source(transform.child_frame_id):
                self.last_transform = transform
                return

        if self.fallback_to_first_transform and msg.transforms:
            self.last_transform = msg.transforms[0]
            if not self.logged_fallback:
                self.get_logger().warn(
                    f'No transform named {self.source_child_frame_id} in '
                    'Gazebo pose TFMessage; using transform[0]. This is '
                    'expected when ros_gz_bridge strips Pose_V names.'
                )
                self.logged_fallback = True

    def _matches_source(self, child_frame_id):
        return (
            child_frame_id == self.source_child_frame_id
            or child_frame_id.endswith('/' + self.source_child_frame_id)
            or child_frame_id.endswith('::' + self.source_child_frame_id)
        )

    def _publish(self):
        if self.last_transform is None:
            return

        yaw = _quaternion_to_yaw(self.last_transform.transform.rotation)
        msg = Imu()
        msg.header.stamp = self._stamp_or_now(self.last_transform.header.stamp)
        msg.header.frame_id = self.frame_id
        msg.orientation = _yaw_to_quaternion(yaw)
        msg.orientation_covariance = [
            999.0, 0.0, 0.0,
            0.0, 999.0, 0.0,
            0.0, 0.0, self.yaw_variance,
        ]
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        self.publisher.publish(msg)

    def _gps_odom_callback(self, msg):
        if self.gps_odom_publisher is None:
            return

        fixed_msg = deepcopy(msg)
        fixed_msg.header.frame_id = self.gps_odom_frame_id
        if self.gps_odom_child_frame_id:
            fixed_msg.child_frame_id = self.gps_odom_child_frame_id
        self.gps_odom_publisher.publish(fixed_msg)

    def _stamp_or_now(self, stamp):
        if stamp.sec == 0 and stamp.nanosec == 0:
            return self.get_clock().now().to_msg()
        return stamp


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseYawImu()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
