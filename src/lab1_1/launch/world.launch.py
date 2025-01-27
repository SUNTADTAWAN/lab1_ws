import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Define the package and world file
    package_name = "lab1_1"
    world_file_name = "basic.world"
    models_directory = os.path.join(
        get_package_share_directory(package_name), "models"
    )
    world_file_path = os.path.join(
        get_package_share_directory(package_name), "worlds", world_file_name
    )

    # Check if the world file exists
    if not os.path.exists(world_file_path):
        raise FileNotFoundError(f"World file not found: {world_file_path}")

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

    # Define and return the launch description
    return LaunchDescription([
        set_gazebo_model_path,
        gazebo
    ])
