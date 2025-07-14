#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from threading import Timer


class AutoLocalizer(Node):
    def __init__(self):
        super().__init__('auto_localizer')

        # --- Parameters ---
        self.target_x = 0.0  # set your known point X
        self.target_y = 0.0   # set your known point Y
        self.rotation_speed = 0.3        # rad/s
        self.rotation_duration = 20.0    # seconds
        self.localized = False

        # --- Publishers & Subscribers ---
        self.initpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)

        # --- Start process ---
        self.get_logger().info('🔄 Starting auto localization...')
        self.set_initial_pose()
        self.start_rotation()

    def set_initial_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = self.target_x
        pose.pose.pose.position.y = self.target_y
        pose.pose.pose.orientation.w = 1.0  # no rotation assumed initially

        # Covariance: allow ~0.5m position, high yaw (angle) uncertainty
        cov = [0.0] * 36
        cov[0] = 0.5    # x covariance
        cov[7] = 0.5    # y covariance
        cov[35] = 1.5   # yaw covariance (theta)

        pose.pose.covariance = cov

        self.initpose_pub.publish(pose)
        self.get_logger().info('📍 Initial pose with covariance published.')

    def start_rotation(self):
        twist = Twist()
        twist.angular.z = self.rotation_speed
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'🔁 Rotating in place for {self.rotation_duration:.1f} seconds...')

        Timer(self.rotation_duration, self.stop_rotation).start()

    def stop_rotation(self):
        twist = Twist()  # Zero velocity
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info('🛑 Rotation stopped.')

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        cov = msg.pose.covariance
        cov_x = cov[0]
        cov_y = cov[7]
        cov_yaw = cov[35]

        if not self.localized and cov_x < 0.05 and cov_y < 0.05 and cov_yaw < 0.1:
            self.localized = True
            x = msg.pose.pose.position.x
            y = msg.pose.pose.position.y
            self.get_logger().info(f'✅ Localization converged at x={x:.2f}, y={y:.2f}')
            self.stop_rotation()
            # Trigger next behavior here if needed


def main(args=None):
    rclpy.init(args=args)
    node = AutoLocalizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
