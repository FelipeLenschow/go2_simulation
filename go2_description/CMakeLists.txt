cmake_minimum_required(VERSION 3.12)
project(go2_description)

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# Install all necessary directories
install(
  DIRECTORY urdf meshes dae config rviz gz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()