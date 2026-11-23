cmake_minimum_required(VERSION 3.8)
project(davis_event_adapter)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(event_camera_msgs REQUIRED)
find_package(event_camera_codecs REQUIRED)
find_package(rt_of_event_msgs REQUIRED)

add_executable(davis_event_adapter_node
  src/davis_event_adapter_node.cpp
)

ament_target_dependencies(davis_event_adapter_node
  rclcpp
  event_camera_msgs
  event_camera_codecs
  rt_of_event_msgs
)

install(TARGETS davis_event_adapter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
