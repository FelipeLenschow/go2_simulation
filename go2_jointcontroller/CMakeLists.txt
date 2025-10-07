cmake_minimum_required(VERSION 3.8)
project(go2_jointcontroller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(unitree_ros2 REQUIRED)

## Check system architecture
message(STATUS "Current system architecture: ${CMAKE_SYSTEM_PROCESSOR}")

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_library( ${PROJECT_NAME}
  SHARED
    src/go2_jointcontroller.cpp
)

target_include_directories(${PROJECT_NAME} 
  PUBLIC
    include
)

target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
)
 
ament_target_dependencies(${PROJECT_NAME}
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  std_msgs
  unitree_go
  unitree_hg
  unitree_api
  unitree_ros2
)

pluginlib_export_plugin_description_file(controller_interface go2_jointcontroller.xml)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include DESTINATION include/go2_jointcontroller)
install(FILES ${DDS_LIBRARIES} DESTINATION lib)

ament_export_include_directories(
  include
)

ament_export_libraries (
  go2_jointcontroller
)
  
ament_export_dependencies(
  controller_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  go2_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()