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

# Install package.xml so tooling can discover the package
install(FILES package.xml
  DESTINATION share/${PROJECT_NAME}
)

# Environment hook so GZ finds worlds/models
ament_environment_hooks(${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv)

ament_package()
