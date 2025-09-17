cmake_minimum_required(VERSION 3.8)
project(bioloid_gazebo)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find core dependencies first
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Create base dependencies list
set(dependencies
  rclcpp
  rclpy
  sensor_msgs
  std_msgs
  geometry_msgs
)

# Try to find Gazebo packages quietly (won't fail if not found)
find_package(ros_gz_sim QUIET)
find_package(ros_gz_bridge QUIET)

# Add Gazebo dependencies only if found
if(ros_gz_sim_FOUND)
  list(APPEND dependencies ros_gz_sim)
  message(STATUS "? Found ros_gz_sim - Gazebo simulation support enabled")
else()
  message(STATUS "? ros_gz_sim not found - Gazebo simulation support disabled")
endif()

if(ros_gz_bridge_FOUND)
  list(APPEND dependencies ros_gz_bridge)
  message(STATUS "? Found ros_gz_bridge - ROS-Gazebo bridge enabled")
else()
  message(STATUS "? ros_gz_bridge not found - ROS-Gazebo bridge disabled")
endif()

###########
## Build ##
###########

# Include directories
include_directories(include)

#############
## Install ##
#############

# Install Python package - NOT work for ament_cmake project here!!!
#ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/trajectory_publisher.py
  scripts/hold_node.py
  scripts/reliable_joint_setter.py
  DESTINATION lib/${PROJECT_NAME}
  OPTIONAL
)

# Install launch files
install(DIRECTORY launch/ OPTIONAL
        DESTINATION share/${PROJECT_NAME}/launch
        FILES_MATCHING PATTERN "*.launch.py"
        )

# Install config files
install(DIRECTORY config/ OPTIONAL
        DESTINATION share/${PROJECT_NAME}/config
        FILES_MATCHING PATTERN "*.yaml" PATTERN "*.yml" PATTERN "*.config"
        )

# Install world files
install(DIRECTORY worlds/ OPTIONAL
        DESTINATION share/${PROJECT_NAME}/worlds
        FILES_MATCHING PATTERN "*.world" PATTERN "*.sdf"
        )

# Install URDF/SDF files
install(DIRECTORY urdf/ OPTIONAL
        DESTINATION share/${PROJECT_NAME}/urdf
        FILES_MATCHING PATTERN "*.urdf" PATTERN "*.xacro" PATTERN "*.sdf"
        )

# Install meshes
install(DIRECTORY meshes/ OPTIONAL
        DESTINATION share/${PROJECT_NAME}/meshes
        FILES_MATCHING PATTERN "*.dae" PATTERN "*.stl" PATTERN "*.obj"
        )

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Export dependencies
ament_export_dependencies(${dependencies})
ament_package()


