cmake_minimum_required(VERSION 3.12)
project(go2_remap)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)    

# Add include directories
include_directories(
  include
)

# Define Executables
add_executable(go2_remap
  src/go2_remap_topic.cpp
)

# Link Dependencies
ament_target_dependencies(go2_remap
  tf2_ros 
  tf2_sensor_msgs 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  std_msgs 
  unitree_go
)

add_executable(go2_remap_cloud
  src/go2_pointcloud_republisher.cpp
)

# Link Dependencies
ament_target_dependencies(go2_remap_cloud
  tf2_ros 
  tf2_sensor_msgs 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  std_msgs 
  unitree_go
)

# Install Executables
install(TARGETS go2_remap go2_remap_cloud
  DESTINATION lib/${PROJECT_NAME}
)

# Install Header Files
install(
    DIRECTORY include/
    DESTINATION include/
)

# Register Package
ament_package()