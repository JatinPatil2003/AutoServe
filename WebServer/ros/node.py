from rclpy.node import Node

class ROSNode(Node):
    def __init__(self):
        super().__init__('web_server')
