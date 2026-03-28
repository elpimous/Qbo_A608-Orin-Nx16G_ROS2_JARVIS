cmake_minimum_required(VERSION 3.8)
project(head_tracker_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(head_tracker_node src/head_tracker_node.cpp)

ament_target_dependencies(head_tracker_node
  rclcpp
  sensor_msgs
)

install(TARGETS
  head_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()