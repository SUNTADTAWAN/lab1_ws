cmake_minimum_required(VERSION 3.5)
project(lab1_1)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################ FIND DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)		# cpp (optional)
find_package(rclcpp REQUIRED)			# cpp (optional)
find_package(ament_cmake_python REQUIRED)   	# python (optional)
find_package(rclpy REQUIRED)                	# python (optional)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


######################################################

# Install Python Module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/lab1_1_basic_model.py
  scripts/lab1_1_double_track.py
  scripts/lab1_1_no_slip_model.py
  scripts/lab1_1_single_track.py
  scripts/lab1_1_yaw_rate.py
  scripts/path_trackling.py
  scripts/plot_path.py
  scripts/plot_auto_drive.py
  scripts/tf_publisher.py
  scripts/map.py


  DESTINATION lib/${PROJECT_NAME}
)


# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(cpp_node src/cpp_node.cpp)
ament_target_dependencies(cpp_node rclcpp)

# Install Cpp executables
install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  models
  meshes
  worlds
  config
  launch
  scripts
  rviz
  descriptions
  DESTINATION share/${PROJECT_NAME})

ament_package()
