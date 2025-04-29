#!/usr/bin/python3

import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():

    nav2_launch_path = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch', 'bringup_launch.py']
    )

    rviz_config_path = PathJoinSubstitution(
        [FindPackageShare('autoserve_navigation'), 'rviz', 'navigation.rviz']
    )

    MAP_NAME = LaunchConfiguration("map_name")

    default_map_path = PathJoinSubstitution(
        [TextSubstitution(text=os.getcwd()), 'maps', MAP_NAME]
    )

    nav2_config_path = PathJoinSubstitution(
        [FindPackageShare('autoserve_navigation'), 'config', 'navigation_mppi.yaml']
    )

    # autoserve_3dmapping_path = PathJoinSubstitution(
    #     [FindPackageShare('autoserve_3dmapping'), 'launch', 'octomap2gridmap.launch.py']
    # )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='sim', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),

        DeclareLaunchArgument(
            name='rviz', 
            default_value='true',
            description='Run rviz'
        ),

        DeclareLaunchArgument(
            name='map', 
            default_value=default_map_path,
            description='Navigation map path'
        ),

        DeclareLaunchArgument(
            name='map_name', 
            default_value='map',
            description='Navigation map name'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_path),
            launch_arguments={
                'map': LaunchConfiguration("map"),
                'use_sim_time': LaunchConfiguration("sim"),
                'params_file': nav2_config_path
            }.items()
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            condition=IfCondition(LaunchConfiguration("rviz")),
            parameters=[{'use_sim_time': LaunchConfiguration("sim")}]
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(autoserve_3dmapping_path),
        # ),
    ])
