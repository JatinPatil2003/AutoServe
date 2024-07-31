from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
import json
import os

from ros.node import ros_node

map_msg = {}
location_msg = {}

def map_callback(msg):
    print('\n\n\n\n\n\n\nRunning Map Callback\n\n\n\n\n')
    global map_msg
    map_msg = {
            'info': {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'position': {
                        'x': msg.info.origin.position.x,
                        'y': msg.info.origin.position.y,
                        'z': msg.info.origin.position.z,
                    },
                    'orientation': {
                        'x': msg.info.origin.orientation.x,
                        'y': msg.info.origin.orientation.y,
                        'z': msg.info.origin.orientation.z,
                        'w': msg.info.origin.orientation.w,
                    }
                }
            },
            'data': list(msg.data)
        }
    print(map_msg)

def get_map_msg():
    global map_msg
    return map_msg

def location_callback(msg):
    global location_msg
    location_msg = {
            'pose': {
                'position': {
                    'x': msg.pose.pose.position.x,
                    'y': msg.pose.pose.position.y,
                    'z': msg.pose.pose.position.z,
                },
                'orientation': {
                    'x': msg.pose.pose.orientation.x,
                    'y': msg.pose.pose.orientation.y,
                    'z': msg.pose.pose.orientation.z,
                    'w': msg.pose.pose.orientation.w,
                }
            }
        }

ros_node.create_subscription(OccupancyGrid, 
                             '/map', map_callback, 10)

ros_node.create_subscription(PoseWithCovarianceStamped,
                             '/amcl_pose', location_callback, 10)


