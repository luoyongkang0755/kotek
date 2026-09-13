cmake_minimum_required(VERSION 3.8)
project(kotek_base_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_library(${PROJECT_NAME}_lib
  src/simple_base_controller.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(simple_base_controller_node src/simple_base_controller_node.cpp)
target_link_libraries(simple_base_controller_node ${PROJECT_NAME}_lib)
ament_target_dependencies(simple_base_controller_node
  rclcpp
  rclcpp_action
  geometry_msgs
  nav_msgs
  nav2_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS simple_base_controller_node
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

  ament_add_gtest(test_pose_utils test/test_pose_utils.cpp)
  target_include_directories(test_pose_utils PUBLIC include)

  ament_add_gtest(test_control_law test/test_control_law.cpp)
  target_include_directories(test_control_law PUBLIC include)
  target_link_libraries(test_control_law ${PROJECT_NAME}_lib)

  ament_add_gtest(test_obstacle_avoidance test/test_obstacle_avoidance.cpp)
  target_include_directories(test_obstacle_avoidance PUBLIC include)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_lib)
ament_package()
