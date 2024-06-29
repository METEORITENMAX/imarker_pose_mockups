cmake_minimum_required(VERSION 3.5)
project(imarker_pose_mockups)

# Find packages in the 'ament_cmake' and 'rclcpp' libraries
find_package(ament_cmake REQUIRED)

# Install Python executables
ament_python_install_package(${PROJECT_NAME})

# Add executables
install(PROGRAMS
  imarker_pose_mockups/interactive_marker_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*launch.py"
)

# # Install parameter files
# install(DIRECTORY config/
#   DESTINATION share/${PROJECT_NAME}/config
#   FILES_MATCHING PATTERN "*config.yaml"
# )

# Export dependencies
ament_export_dependencies(ament_cmake rclcpp)

ament_package()