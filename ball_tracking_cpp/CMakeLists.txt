cmake_minimum_required(VERSION 3.5)
project(ball_tracking_cpp)

# C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
# image_transport non utilisé
find_package(OpenCV REQUIRED)

include_directories(
  include
)

add_executable(red_ball_tracker_node src/red_ball_tracker.cpp)
ament_target_dependencies(red_ball_tracker_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

target_include_directories(red_ball_tracker_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS red_ball_tracker_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()