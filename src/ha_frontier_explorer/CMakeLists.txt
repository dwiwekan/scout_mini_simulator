cmake_minimum_required(VERSION 3.8)
project(ha_frontier_explorer)

# C++ 표준
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(explorer_deps
  rclcpp nav_msgs geometry_msgs nav2_msgs rclcpp_action tf2 tf2_ros tf2_geometry_msgs)

# YOLO object-goal pursuit is optional: built only when yolo_msgs is in the workspace
find_package(yolo_msgs QUIET)
if(yolo_msgs_FOUND)
  list(APPEND explorer_deps yolo_msgs)
  message(STATUS "yolo_msgs found: object-goal pursuit enabled")
else()
  message(STATUS "yolo_msgs not found: building frontier exploration only")
endif()

add_executable(frontier_explorer_node src/frontier_explorer_node.cpp)
ament_target_dependencies(frontier_explorer_node ${explorer_deps})
if(yolo_msgs_FOUND)
  target_compile_definitions(frontier_explorer_node PRIVATE HAVE_YOLO_MSGS)
endif()

# 실행 파일 설치
install(TARGETS frontier_explorer_node
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# 리소스(런치/설정) 설치
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()
