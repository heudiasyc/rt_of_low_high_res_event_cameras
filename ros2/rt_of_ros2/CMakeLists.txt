cmake_minimum_required(VERSION 3.8)
project(rt_of_ros2)
set(CMAKE_CUDA_ARCHITECTURES 75)
enable_language(CUDA)
find_package(CUDAToolkit REQUIRED)
set(CMAKE_CXX_STANDARD 17)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rt_of_event_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)
add_executable(events_txt_node src/events_txt_node.cpp)
ament_target_dependencies(events_txt_node rclcpp sensor_msgs cv_bridge)
target_include_directories(events_txt_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(events_txt_node ${OpenCV_LIBS})
install(TARGETS events_txt_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

add_executable(distance_surface_node src/distance_surface_node.cpp)
ament_target_dependencies(distance_surface_node rclcpp sensor_msgs cv_bridge)
target_include_directories(distance_surface_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(distance_surface_node ${OpenCV_LIBS})
install(TARGETS distance_surface_node DESTINATION lib/${PROJECT_NAME})


add_executable(distance_surface_original_node
  src/distance_surface_original_node.cpp
  cuda/distance_surface_gpu_ros2.cu)
ament_target_dependencies(distance_surface_original_node rclcpp sensor_msgs cv_bridge)
target_include_directories(distance_surface_original_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(distance_surface_original_node
  ${OpenCV_LIBS} CUDA::cudart)
install(TARGETS distance_surface_original_node DESTINATION lib/${PROJECT_NAME})


# GPU_FLOW_NODE_BEGIN
add_executable(optical_flow_gpu_node src/optical_flow_gpu_node.cpp)
ament_target_dependencies(optical_flow_gpu_node rclcpp sensor_msgs cv_bridge)
# External GPU optical-flow library.
# Override with -DFLOWFILTER_ROOT=/path/to/optical-flow-filter if needed.
set(FLOWFILTER_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../external/optical-flow-filter"
    CACHE PATH "Path to optical-flow-filter source tree")

find_path(FLOWFILTER_INCLUDE_DIR
    NAMES flowfilter/gpu/flowfilter.h
    PATHS "${FLOWFILTER_ROOT}/include"
    NO_DEFAULT_PATH
    REQUIRED)

find_library(FLOWFILTER_GPU_LIBRARY
    NAMES flowfilter_gpu
    PATHS "${FLOWFILTER_ROOT}/build/lib"
    NO_DEFAULT_PATH
    REQUIRED)

target_include_directories(optical_flow_gpu_node PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${FLOWFILTER_INCLUDE_DIR}
    ${CUDAToolkit_INCLUDE_DIRS})

target_link_libraries(optical_flow_gpu_node
    ${OpenCV_LIBS}
    ${FLOWFILTER_GPU_LIBRARY}
    CUDA::cudart)

get_filename_component(FLOWFILTER_LIBRARY_DIR
    "${FLOWFILTER_GPU_LIBRARY}" DIRECTORY)

set_target_properties(optical_flow_gpu_node PROPERTIES
    BUILD_RPATH "${FLOWFILTER_LIBRARY_DIR}"
    INSTALL_RPATH "${FLOWFILTER_LIBRARY_DIR}")
install(TARGETS optical_flow_gpu_node DESTINATION lib/${PROJECT_NAME})
# GPU_FLOW_NODE_END




# Streaming TXT replay: independent of the original in-memory adapter.
add_executable(events_txt_stream_node src/events_txt_stream_node.cpp)
ament_target_dependencies(events_txt_stream_node rclcpp sensor_msgs cv_bridge)
target_link_libraries(events_txt_stream_node ${OpenCV_LIBRARIES})
install(TARGETS events_txt_stream_node DESTINATION lib/${PROJECT_NAME})

# Independent ROS2 CPU denoising/filling module
add_executable(denoising_filling_node
  src/denoising_filling_node.cpp
  cuda/denoising_filling_gpu_ros2.cu)
ament_target_dependencies(denoising_filling_node rclcpp sensor_msgs cv_bridge)
target_include_directories(denoising_filling_node PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(denoising_filling_node ${OpenCV_LIBRARIES} CUDA::cudart)
install(TARGETS denoising_filling_node DESTINATION lib/${PROJECT_NAME})


# ROS2 native event accumulation
add_executable(
  events_accumulation_node
  src/events_accumulation_node.cpp
)

ament_target_dependencies(
  events_accumulation_node
  rclcpp
  sensor_msgs
  cv_bridge
  rt_of_event_msgs
)

target_include_directories(
  events_accumulation_node
  PRIVATE ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(
  events_accumulation_node
  ${OpenCV_LIBS}
)

install(
  TARGETS events_accumulation_node
  DESTINATION lib/${PROJECT_NAME}
)


# Optional HDF5 support for DSEC event replay.
option(ENABLE_HDF5 "Enable HDF5 event replay support" ON)

if(ENABLE_HDF5)
  find_package(HDF5 QUIET COMPONENTS CXX)

  if(HDF5_FOUND)
    add_executable(events_h5_node src/events_h5_node.cpp)

    ament_target_dependencies(
      events_h5_node
      rclcpp
      rt_of_event_msgs
    )

    target_include_directories(
      events_h5_node PRIVATE
      ${HDF5_INCLUDE_DIRS}
    )

    target_link_libraries(
      events_h5_node
      ${HDF5_CXX_LIBRARIES}
      ${HDF5_LIBRARIES}
    )

    install(
      TARGETS events_h5_node
      DESTINATION lib/${PROJECT_NAME}
    )


    # Direct HDF5-to-edge-image replay.
    add_executable(
      events_h5_edges_node
      src/events_h5_edges_node.cpp
    )

    ament_target_dependencies(
      events_h5_edges_node
      rclcpp sensor_msgs cv_bridge
    )

    target_include_directories(
      events_h5_edges_node PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${HDF5_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
    )

    target_link_libraries(
      events_h5_edges_node
      ${HDF5_CXX_LIBRARIES}
      ${HDF5_LIBRARIES}
      ${OpenCV_LIBS}
    )

    target_compile_options(
      events_h5_edges_node PRIVATE -O3
    )

    install(
      TARGETS events_h5_edges_node
      DESTINATION lib/${PROJECT_NAME}
    )

    message(STATUS "HDF5 replay support: ENABLED")
  else()
    message(WARNING "HDF5 not found: HDF5 replay disabled")
  endif()
else()
  message(STATUS "HDF5 replay support: DISABLED")
endif()

ament_package()
