#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro  
from launch.actions import SetEnvironmentVariable, TimerAction

def generate_launch_description():
    # Define the package and file paths
    package_name = "lab1_1"
    world_file_name = "basic.world"
    models_directory = os.path.join(
        get_package_share_directory(package_name), "models"
    )
    world_file_path = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )
    robot_description_file = os.path.join(
        get_package_share_directory(package_name), "descriptions", "robot.urdf"
    )
    rviz_config_file = os.path.join(
        get_package_share_directory(package_name), "rviz", "config_rviz.rviz"
    )

    # Check if required files exist
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")
    if not os.path.exists(robot_description_file):
        raise FileNotFoundError(f"Robot description file not found: {robot_description_file}")

    # Read the URDF file
    with open(robot_description_file, "r") as urdf_file:
        robot_description = urdf_file.read()

    # Set GAZEBO_MODEL_PATH to include the models directory
    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=models_directory,
    )

    # Include Gazebo with the specified world file and GUI enabled
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_file_path, "gui": "true", "use_sim_time": "true"}.items(),
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Spawn the robot entity in Gazebo
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "robot",
            "-file", robot_description_file,
            "-x", "0", "-y", "0", "-z", "0",
            "-R", "0", "-P", "0", "-Y", "0"
        ],
        output="screen",
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
    )

    # Controller Manager
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
    )

    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Robot Controller Spawner
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Static TF publisher
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "odom"],
        output="screen"
    )

    # Odometry Publisher Node (Added Here)
    odom_publisher = Node(
        package="lab1_1",
        executable="tf_publisher.py",
        name="tf_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Delay Spawner Nodes to Ensure Controller Manager is Ready
    delay_controller_spawners = TimerAction(
        period=5.0,  # Delay by 5 seconds
        actions=[joint_state_broadcaster_spawner, robot_controller_spawner],
    )

    joint_trajectory_position_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_position_controller", "--controller-manager", "/controller_manager"],
    output="screen",
    parameters=[{"use_sim_time": True}],
)


    # Define and return the launch description
    return LaunchDescription([   
        set_gazebo_model_path,
        gazebo,
        spawn_robot_node,
        robot_state_publisher,
        rviz_node,
        controller_manager,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        static_tf,
        odom_publisher,  # <---- Added Odometry Publisher Here
        joint_trajectory_position_controller_spawner
    ])
