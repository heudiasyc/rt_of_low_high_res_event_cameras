# ROS2 Jazzy Migration

This directory contains the ROS2 Jazzy migration of the
real-time optical flow pipeline for event cameras.

The original ROS1 implementation is preserved at the repository root.

## Environment

The migration was developed and tested with:

- Xubuntu 24.04
- ROS2 Jazzy
- NVIDIA Quadro RTX 5000
- CUDA Toolkit 12.6
- OpenCV
- optical-flow-filter GPU library

The CUDA architecture is currently set to 75 for the Quadro RTX 5000.
Other NVIDIA GPUs may require changing CMAKE_CUDA_ARCHITECTURES.

## ROS2 Packages

- `rt_of_event_msgs`: Event and EventArray message definitions.
- `davis_event_adapter`: Adapter for DAVIS event-camera messages.
- `rt_of_ros2`: Main optical-flow processing pipeline.

## Pipeline

The pipeline consists of four processing stages:

1. Events accumulation
2. Denoising and filling
3. Distance transform
4. Optical flow computation

## External GPU Dependency

The GPU optical-flow node depends on the external
`optical-flow-filter` library.

The library must be built separately.

By default, CMake searches for it at:

    external/optical-flow-filter/

relative to the repository root.

Alternatively, specify its location with:

    -DFLOWFILTER_ROOT=/absolute/path/to/optical-flow-filter

## CUDA Environment

Before building, configure CUDA 12.6:

```bash
export CUDA_HOME=/usr/local/cuda-12.6
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
export CUDACXX="$CUDA_HOME/bin/nvcc"
```

Verify the CUDA compiler:

```bash
nvcc --version
```

If CMake cannot locate CUDA, specify the compiler explicitly:

```bash
-DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc
-DCUDAToolkit_ROOT=/usr/local/cuda-12.6
```

## Build

Create a separate ROS2 workspace:

    mkdir -p ~/rt_of_jazzy_ws/src

Copy or symlink the three packages from this directory into
the workspace's src directory.

Source ROS2 Jazzy:

    source /opt/ros/jazzy/setup.bash

Build the packages:

    cd ~/rt_of_jazzy_ws

    colcon build --symlink-install \
  	--packages-select rt_of_event_msgs davis_event_adapter rt_of_ros2 \
  	--cmake-args \
  	-DCMAKE_CUDA_COMPILER=/usr/local/cuda-12.6/bin/nvcc \
  	-DCUDAToolkit_ROOT=/usr/local/cuda-12.6 \
  	-DFLOWFILTER_ROOT=/absolute/path/to/optical-flow-filter
Source the workspace:

    source install/setup.bash

## Launch

The complete pipeline can be started with:

    ros2 launch rt_of_ros2 full_pipeline.launch.py

The event input source must also be started separately
according to the selected camera or dataset.

## Notes

The original ROS1 implementation remains unchanged.

Large rosbag datasets, build artifacts, and local experimental
logs are not included in this repository.

A future standalone implementation without ROS may be
developed separately.
