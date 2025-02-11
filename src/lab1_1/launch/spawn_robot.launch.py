#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro  
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription, TimerAction

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
        "src", "lab1_1", "rviz", "config_rviz.rviz"
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
        # remappings=[('/joint_states', '/joint_states')]
    )

    # Spawn the robot entity in Gazebo using a Node
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
        # parameters=[{"use_sim_time": True}],
    )

    # Controller Manager (Ensures controllers are loaded)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
    )

    # Joint State Broadcaster Spawner Node
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Robot Controller Spawner Node
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["velocity_controllers", "--controller-manager", "/controller_manager"],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    # Delay Spawner Nodes to Ensure Controller Manager is Ready
    delay_controller_spawners = TimerAction(
        period=5.0,  # Delay by 5 seconds
        actions=[joint_state_broadcaster_spawner, robot_controller_spawner],
    )
    
    odom_publisher_node = Node(
    package="lab1_1",
    executable="odom_publisher.py",
    name="odom_publisher",
    output="screen",
    parameters=[{"use_sim_time": True}]
    )
    tf_static_publisher = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=["0", "0", "0", "0", "0", "0", "odom", "base"],
)

    # Define and return the launch description
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        spawn_robot_node,
        robot_state_publisher,
        rviz_node,
        tf_static_publisher,  # Add this line
        # controller_manager,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        # odom_publisher_node,
    ])
