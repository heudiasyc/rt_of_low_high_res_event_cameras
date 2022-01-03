cmake_minimum_required(VERSION 3.0.2)
project(rt_of_low_high_res_event_cameras)

# Use of catkin_simple
find_package(catkin_simple REQUIRED)

# OpenMP is required for CPU code optimization, find it
find_package(OpenMP REQUIRED)
set(OpenMP_LIBS gomp)

# CUDA is required for GPU code, find it
find_package(CUDA REQUIRED)

# Find the flowfilter_gpu library
# Be careful: it is installed by default in /usr/local/lib/, but if you changed
# the install path, simply modify it here
find_library(flowfilter_gpu_LIBS NAMES flowfilter_gpu PATHS /usr/local/include)

# Compile using C+11, -O3 for maximum optimization, enable all error alerts, and add OpenMP flags
add_compile_options(-std=c++11 -O3 -Werror -Wpedantic -Wall -Wextra ${OpenMP_CXX_FLAGS})
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11 -O3 -Xptxas -O3 -Werror all-warnings")

# Initialize catkin_simple
catkin_simple(ALL_DEPS_REQUIRED)

# Edges image computation node, from a live camera
cs_add_executable(edges_node_live src/edges_node_live.cpp src/edges_cpu.cpp src/utils.cpp)

# Edges image computation node, from a replay file
cs_add_executable(edges_node_replay src/edges_node_replay.cpp src/edges_cpu.cpp src/utils.cpp)

# Denoising and filling node
cuda_add_executable(denoising_filling_node src/denoising_filling_node.cpp src/denoising_filling_cpu.cpp src/denoising_filling_gpu.cu)
target_link_libraries(denoising_filling_node ${catkin_LIBRARIES})

# Distance surface node
cuda_add_executable(distance_surface_node src/distance_surface_node.cpp src/distance_surface_cpu.cpp src/distance_surface_gpu.cu)
target_link_libraries(distance_surface_node ${catkin_LIBRARIES} ${OpenMP_LIBS})

# Optical flow computation node
cs_add_executable(optical_flow_node src/optical_flow_node.cpp src/optical_flow_gpu.cpp)
target_link_libraries(optical_flow_node ${flowfilter_gpu_LIBS})

# Optical flow visualization node
cs_add_executable(optical_flow_viz_node src/optical_flow_viz_node.cpp src/optical_flow_viz_cpu.cpp)

# Installation
cs_install()

# Export
cs_export()
