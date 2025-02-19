import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    autoserve_gazebo_dir = get_package_share_directory("autoserve_gazebo")
    # autonav_gazebo_dir = get_package_share_directory("autonav_gazebo")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    autoserve_description_share = os.path.join(
        get_package_prefix("autoserve_description"), "share"
    )
    gazebo_models_path = os.path.join(autoserve_gazebo_dir, 'models')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = gazebo_models_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  gazebo_models_path

    env_var = SetEnvironmentVariable("GAZEBO_MODEL_PATH", autoserve_description_share)

    robot_description_file_1 = ParameterValue(
        Command(
            [
                "xacro ", os.path.join(get_package_share_directory("autoserve_description"),"urdf","autoserve.xacro")," namespace:=autoserve_1"
            ]
        ),
        value_type=str,
    )
    robot_description_1 = {"robot_description": robot_description_file_1}

    robot_description_file_2 = ParameterValue(
        Command(
            [
                "xacro ", os.path.join(get_package_share_directory("autoserve_description"),"urdf","autoserve.xacro")," namespace:=autoserve_2"
            ]
        ),
        value_type=str,
    )
    robot_description_2 = {"robot_description": robot_description_file_2}

    robot_state_publisher_node_1 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_1],
        namespace="autoserve_1",
    )

    robot_state_publisher_node_2 = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description_2],
        namespace="autoserve_2",
    )

    world = os.path.join(
        autoserve_gazebo_dir,
        'worlds',
        'cafe.world'
    )

    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py"),
        ),
        launch_arguments={'world': world}.items()
    )

    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    spawn_robot_1 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "autoserve_1",
            "-topic",
            "/autoserve_1/robot_description",
            "-x", "0", "-y", "0", "-z", "0.05",
        ],
        namespace="autoserve_1",
        output="screen",
    )

    spawn_robot_2 = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "autoserve_2",
            "-topic",
            "/autoserve_2/robot_description",
            "-x", "1", "-y", "0", "-z", "0.05",
        ],
        namespace="autoserve_2",
        output="screen",
    )

    delayed_spawner_1 = TimerAction(period=5.0, actions=[spawn_robot_1])

    delayed_spawner_2 = TimerAction(period=5.0, actions=[spawn_robot_2])


    return LaunchDescription(
        [
            env_var,
            start_gazebo_server,
            start_gazebo_client,
            robot_state_publisher_node_1,
            robot_state_publisher_node_2,
            delayed_spawner_1,
            delayed_spawner_2,
        ]
    )