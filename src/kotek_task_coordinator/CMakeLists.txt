cmake_minimum_required(VERSION 3.8)
project(kotek_task_coordinator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(kotek_msgs REQUIRED)
find_package(kotek_manipulation REQUIRED)
find_package(kotek_base_control REQUIRED)

include_directories(include)

add_executable(task_coordinator_node src/task_coordinator_node.cpp)
ament_target_dependencies(task_coordinator_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  std_srvs
  nav2_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  kotek_msgs
  kotek_manipulation
  kotek_base_control
)

install(TARGETS task_coordinator_node
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

  ament_add_gtest(test_state_machine test/test_state_machine.cpp)
  target_include_directories(test_state_machine PUBLIC include)
endif()

ament_export_include_directories(include)
ament_package()
