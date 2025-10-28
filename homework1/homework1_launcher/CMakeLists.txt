cmake_minimum_required(VERSION 3.5)
project(homework1_launcher)

find_package(ament_cmake REQUIRED)

# Install assets
install(DIRECTORY
  launch
  config
  rviz
  worlds
  models
  DESTINATION share/${PROJECT_NAME}
)

# Install the ament resource marker so ROS 2 can discover the package
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Install package.xml (good practice and helps tooling)
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Environment hook so Gazebo finds worlds/models
ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv)

ament_package()
