cmake_minimum_required(VERSION 3.16)
project(bioloid_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install directories
install(DIRECTORY
  urdf
  assets
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install launch files specifically (if using Python launch files)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.py" PATTERN "*.launch.py"
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export package
ament_package()
