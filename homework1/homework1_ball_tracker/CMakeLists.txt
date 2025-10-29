cmake_minimum_required(VERSION 3.8)
project(homework1_ball_tracker)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)

# Include directories
include_directories(include)

# Add the camera tracker node
add_executable(camera_tracker_node src/camera_tracker_node.cpp)
ament_target_dependencies(camera_tracker_node
  rclcpp sensor_msgs geometry_msgs cv_bridge image_geometry message_filters OpenCV)

add_executable(green_follower_node src/green_follower_node.cpp)
ament_target_dependencies(green_follower_node rclcpp geometry_msgs)

# Install the executable
install(TARGETS
  camera_tracker_node
  green_follower_node
  DESTINATION lib/${PROJECT_NAME})

# Install runtime data
install(DIRECTORY launch rviz config
  DESTINATION share/${PROJECT_NAME})

# Register this package in the ament index (this fixes “Package not found”)
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

# Good practice: install package.xml too
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()