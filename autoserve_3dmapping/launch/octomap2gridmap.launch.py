import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    autoserve_3dmapping_dir = get_package_share_directory('autoserve_3dmapping')

    param_file = LaunchConfiguration('param_file')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=os.path.join(
            autoserve_3dmapping_dir, 'config', 'octomap2gridmap.yaml'),
        description='Full path to the config file to use')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(
            autoserve_3dmapping_dir, 'rviz', 'octomap2gridmap.rviz'),
        description='Full path to the RVIZ config file to use')

    # Declare node actions

    octomap_server_node = Node(
        package='octomap_server',
        executable='octomap_server_static_node',
        name='octomap_server',
        output='screen',
        parameters=[
            param_file,
            {
                'octomap_path': os.path.join(autoserve_3dmapping_dir, 'maps', 'cafe.bt')
            }
        ],
    )

    octomap_to_gridmap_demo_node = Node(
        package='grid_map_demos',
        executable='octomap_to_gridmap_demo',
        name='octomap_to_gridmap_demo',
        output='screen'
    )

    grid_map_visualization_node = Node(
        package='grid_map_visualization',
        executable='grid_map_visualization',
        name='grid_map_visualization',
        output='screen',
        parameters=[param_file]
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    ld = LaunchDescription()

    ld.add_action(declare_param_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(octomap_server_node)
    ld.add_action(octomap_to_gridmap_demo_node)
    ld.add_action(grid_map_visualization_node)
    # ld.add_action(rviz2_node)

    return ld
