cmake_minimum_required(VERSION 3.8)
project(whill_description)

# Set compile options for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)       # For parsing URDF files
find_package(xacro REQUIRED)      # For using xacro syntax in URDF files
find_package(robot_state_publisher REQUIRED) # A common dependency for publishing robot states
#find_package(joint_state_publisher REQUIRED) # To publish joint states (often a dependency of launch files)
#find_package(rviz2 REQUIRED)      # If using rviz2 for visualization

# Install launch files and other necessary directories
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
  # Use FILES_MATCHING to install only launch files, not the whole directory
  FILES_MATCHING PATTERN "*.launch.py"
)

# Install URDF and model files
install(
  DIRECTORY urdf/
  DESTINATION share/${PROJECT_NAME}
)

# Install model files (e.g., meshes) if they are in a separate directory
install(
  DIRECTORY model/
  DESTINATION share/${PROJECT_NAME}
)

# Add any executables or libraries here
# add_executable(my_node src/my_node.cpp)
# ament_target_dependencies(my_node rclcpp)

# Testing setup (your original was fine, just keeping it for completeness)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize the package build process
ament_package()
