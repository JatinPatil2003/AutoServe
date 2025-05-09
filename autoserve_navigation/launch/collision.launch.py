#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    autoserve_navigation_dir = get_package_share_directory("autoserve_navigation")

    # Bumperbot Navigation parameter file
    collision_monitor_file = os.path.join(autoserve_navigation_dir,"config","collision_monitor.yaml")

    # Bumperbot default map yaml file
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(name='use_sim_time',
                                                     default_value='true',
                                                     description='Use simulation (Gazebo) clock if true')

    robot_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('nav2_collision_monitor'),'launch', 'collision_monitor_node.launch.py')),
        launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': collision_monitor_file}.items()
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(robot_navigation_cmd)

    return ld