cmake_minimum_required(VERSION 3.8)
project(kotek_manipulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(kotek_msgs REQUIRED)
find_package(kotek_base_control REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(piper_manipulator_node src/piper_manipulator_node.cpp)
ament_target_dependencies(piper_manipulator_node
  rclcpp
  rclcpp_action
  geometry_msgs
  control_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  kotek_msgs
  kotek_base_control
  tf2
  tf2_ros
  tf2_geometry_msgs
)
target_include_directories(piper_manipulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS piper_manipulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pregrasp_geometry test/test_pregrasp_geometry.cpp)
  target_include_directories(test_pregrasp_geometry PUBLIC include)
  ament_target_dependencies(test_pregrasp_geometry kotek_base_control)
endif()

ament_export_include_directories(include)
ament_package()
