cmake_minimum_required(VERSION 3.8)
project(qbo_tts)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(PORTAUDIO REQUIRED portaudio-2.0)
pkg_check_modules(SHERPA_ONNX REQUIRED sherpa-onnx)

add_executable(qbo_tts_node src/qbo_tts_node.cpp)

target_include_directories(qbo_tts_node PRIVATE
  ${SHERPA_ONNX_INCLUDE_DIRS}
  ${PORTAUDIO_INCLUDE_DIRS}
)

target_link_libraries(qbo_tts_node
  ${SHERPA_ONNX_LIBRARIES}
  ${PORTAUDIO_LIBRARIES}
  pthread
)

ament_target_dependencies(qbo_tts_node rclcpp std_msgs)

# Install executable
install(TARGETS qbo_tts_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install models (IMPORTANT !)
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models
)

ament_package()