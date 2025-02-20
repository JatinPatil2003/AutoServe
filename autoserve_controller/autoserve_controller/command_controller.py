#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros import TransformListener, Buffer
import numpy as np

class VelocityTransformer(Node):
    def __init__(self):
        super().__init__('velocity_transformer')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher_1 = self.create_publisher(Twist, '/autoserve_1_cmd_vel', 10)
        self.publisher_2 = self.create_publisher(Twist, '/autoserve_2_cmd_vel', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.current_cmd_vel = Twist()
        # self.timer = self.create_timer(0.05, self.update_velocities)  # 20 Hz
    
    # def transform_velocity(self, v_mid, d_x, d_y, theta):
    #     v_x, v_y, omega = v_mid
    #     v_x_side_global = v_x - omega * d_y
    #     v_y_side_global = v_y + omega * d_x
    #     R_theta = np.array([
    #         [np.cos(theta), np.sin(theta)],
    #         [-np.sin(theta), np.cos(theta)]
    #     ])
    #     v_side_local = R_theta @ np.array([v_x_side_global, v_y_side_global])
    #     return (v_side_local[0], v_side_local[1], omega)

    def transform_velocity(self, v_mid, d_x, d_y, theta):
        v_x, v_y, omega = v_mid

        factor = 0.7
        # Compute velocity in global frame considering rotation around the base frame
        v_x_side_global = v_x - omega * d_y * factor
        v_y_side_global = v_y + omega * d_x * factor

        # Compute additional velocity caused by rotation
        # v_x_rot = -omega * d_y
        # v_y_rot = omega * d_x

        # # Total velocity in global frame
        # v_x_side_global -= v_x_rot
        # v_y_side_global -= v_y_rot

        # Rotate into local robot frame
        R_theta = np.array([
            [np.cos(theta), np.sin(theta)],
            [-np.sin(theta), np.cos(theta)]
        ])
        v_side_local = R_theta @ np.array([v_x_side_global, v_y_side_global])

        return (v_side_local[0], v_side_local[1], omega)

    
    def cmd_vel_callback(self, msg):
        self.current_cmd_vel = msg
        self.update_velocities()
    
    def update_velocities(self):
        try:
            tf1 = self.tf_buffer.lookup_transform_full(
                'autoserve_base_footprint', rclpy.time.Time(), 'autoserve_1_base_footprint', rclpy.time.Time(), 'odom', timeout=rclpy.duration.Duration(seconds=1.0)
            )
            tf2 = self.tf_buffer.lookup_transform_full(
                'autoserve_base_footprint', rclpy.time.Time(), 'autoserve_2_base_footprint', rclpy.time.Time(), 'odom', timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            d_x1, d_y1 = tf1.transform.translation.x, tf1.transform.translation.y
            d_x2, d_y2 = tf2.transform.translation.x, tf2.transform.translation.y
            
            theta1 = np.arctan2(tf1.transform.rotation.z, tf1.transform.rotation.w) * 2
            theta2 = np.arctan2(tf2.transform.rotation.z, tf2.transform.rotation.w) * 2
            
            if not self.current_cmd_vel.linear.x == 0.0 or not self.current_cmd_vel.linear.y == 0.0 or not self.current_cmd_vel.angular.z == 0.0:
                v_mid = (self.current_cmd_vel.linear.x, self.current_cmd_vel.linear.y, self.current_cmd_vel.angular.z)
                v1 = self.transform_velocity(v_mid, d_x1, d_y1, theta1)
                v2 = self.transform_velocity(v_mid, d_x2, d_y2, theta2)
                
                msg1 = Twist()
                msg2 = Twist()

                msg1.linear.x, msg1.linear.y, msg1.angular.z = v1
                msg2.linear.x, msg2.linear.y, msg2.angular.z = v2

                self.publisher_1.publish(msg1)
                self.publisher_2.publish(msg2)
            else:
                self.publisher_1.publish(Twist())
                self.publisher_2.publish(Twist())
        
        except Exception as e:
            self.get_logger().warn(f'Failed to get transforms: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = VelocityTransformer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
