import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Define the package and world file
    package_name = "lab1_1"  # Replace with your package name
    world_file_name = "basic.world"  # Replace with your Gazebo world file name
    world_file_path = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )

    # Include Gazebo with the specified world file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("gazebo_ros"),
                    "launch",
                    "gazebo.launch.py"
                )
            ]
        ),
        launch_arguments={"world": world_file_path}.items(),  # Pass the world file to Gazebo
    )

    # Define and return the launch description
    return LaunchDescription([
        gazebo
    ])
