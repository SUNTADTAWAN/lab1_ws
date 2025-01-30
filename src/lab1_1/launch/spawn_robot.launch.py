import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
        "src","lab1_1", "rviz", "config_rviz.rviz"
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

    # Include Gazebo with the specified world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        ),
        launch_arguments={"world": world_file_path}.items(),
    )

    # Robot State Publisher Node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    # Joint State Publisher GUI Node
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # Spawn the robot entity in Gazebo using a Node
    spawn_robot_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "robot",
            "-file", robot_description_file,
            "-x", "0", "-y", "0", "-z", "1"
        ],
        output="screen",
    )

    # RViz2 Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file]
    )

    # Define and return the launch description
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo,
        robot_state_publisher,
        joint_state_publisher,
        spawn_robot_node,
        rviz_node,
    ])
