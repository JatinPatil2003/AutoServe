#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter
import time
class LifecycleNodeManager(Node):
    def __init__(self):
        super().__init__("lifecycle_manager")
        self.client = self.create_client(ChangeState, '/collision_monitor/change_state')
        self.set_param_client = self.create_client(SetParameters, '/collision_monitor/set_parameters')
    def change_state(self, transition_id):
        self.client.wait_for_service()
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
    def set_parameter(self, param_name, param_value):
        if not self.set_param_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Parameter service not available, exiting...')
            return
        param = Parameter(param_name, Parameter.Type.DOUBLE_ARRAY, param_value)
        request = SetParameters.Request()
        request.parameters = [param.to_parameter_msg()]
        future = self.set_param_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'Successfully set parameter: {param_name} to {param_value}')
        else:
            self.get_logger().error('Failed to set parameter')
    def initialization_sequence(self):
        self.set_parameter('PolygonStop.points', [1.8, 0.17, 1.8, -0.17, 0.0, -0.17, 0.0, 0.17])
        self.get_logger().info("Switching to configuring")
        self.change_state(Transition.TRANSITION_DEACTIVATE)
        self.get_logger().info("Configuring OK, now deactive")
        time.sleep(3)
def main(args=None):
    rclpy.init(args=args)
    node = LifecycleNodeManager()
    node.initialization_sequence()
    rclpy.shutdown()
if __name__ == "__main__":
    main()