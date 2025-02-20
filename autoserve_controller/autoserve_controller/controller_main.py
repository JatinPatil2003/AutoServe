#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import numpy as np
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class DualRobotTFController(Node):
    def __init__(self):
        super().__init__('dual_robot_tf_controller')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.update_tf)  # 10 Hz

    def update_tf(self):
        try:
            tf1 = self.tf_buffer.lookup_transform_full(
                'odom', rclpy.time.Time(), 'autoserve_1_base_footprint', rclpy.time.Time(), 'odom', timeout=rclpy.duration.Duration(seconds=1.0)
            )
            tf2 = self.tf_buffer.lookup_transform_full(
                'odom', rclpy.time.Time(), 'autoserve_2_base_footprint', rclpy.time.Time(), 'odom', timeout=rclpy.duration.Duration(seconds=1.0)
            )

            mean_tf = TransformStamped()
            mean_tf.header.stamp = self.get_clock().now().to_msg()
            mean_tf.header.frame_id = 'odom'
            mean_tf.child_frame_id = 'autoserve_base_footprint'
            
            # Compute mean translation
            mean_tf.transform.translation.x = (tf1.transform.translation.x + tf2.transform.translation.x) / 2.0
            mean_tf.transform.translation.y = (tf1.transform.translation.y + tf2.transform.translation.y) / 2.0
            
            # Compute mean rotation using quaternion averaging
            tf2 = self.tf_buffer.lookup_transform_full(
                'autoserve_1_base_footprint', rclpy.time.Time(), 'autoserve_2_base_footprint', rclpy.time.Time(), 'odom', timeout=rclpy.duration.Duration(seconds=1.0)
            )

            q1 = np.array([tf1.transform.rotation.x, tf1.transform.rotation.y, tf1.transform.rotation.z, tf1.transform.rotation.w])
            q2 = np.array([tf2.transform.rotation.x, tf2.transform.rotation.y, tf2.transform.rotation.z, tf2.transform.rotation.w])
            # mean_quat = (q1 + q2) / np.linalg.norm(q1 + q2)  # Normalize

            euler1 = euler_from_quaternion(q1)
            euler2 = euler_from_quaternion(q2)

            # yaw1 = euler1[2]
            # yaw2 = euler2[2]

            # # Unwrap angles to prevent averaging issues (handles ±π wrap-around)
            # yaw1 = np.arctan2(np.sin(yaw1), np.cos(yaw1))
            # yaw2 = np.arctan2(np.sin(yaw2), np.cos(yaw2))

            # # Compute weighted average with wrap-around handling
            # mean_euler = np.arctan2(np.sin(yaw1) + np.sin(yaw2), np.cos(yaw1) + np.cos(yaw2))

            mean_euler = euler1[2] + euler2[2] / 2.0
            
            euler = [0.0, 0.0, mean_euler]

            mean_quat = quaternion_from_euler(*euler)

            mean_tf.transform.rotation.x = mean_quat[0]
            mean_tf.transform.rotation.y = mean_quat[1]
            mean_tf.transform.rotation.z = mean_quat[2]
            mean_tf.transform.rotation.w = mean_quat[3]
            
            self.tf_broadcaster.sendTransform(mean_tf)
        except Exception as e:
            self.get_logger().warn(f'Failed to get transforms: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = DualRobotTFController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
