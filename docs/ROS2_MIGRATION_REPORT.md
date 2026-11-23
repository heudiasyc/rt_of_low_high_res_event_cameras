# ROS1 to ROS2 Jazzy Migration Report

## 1. Project Overview

This project migrates the real-time optical flow pipeline for event cameras from ROS1 to ROS2 Jazzy.

The original implementation is preserved in the repository, while the ROS2 implementation is maintained separately under the `ros2/` directory.

The main objective is to preserve the original four-stage processing architecture while adapting the software to the ROS2 communication system, build environment, and GPU dependencies.

The migration was developed and validated on Xubuntu 24.04 with an NVIDIA Quadro RTX 5000 GPU and CUDA Toolkit 12.6.

## 2. Architecture Migration

The optical flow pipeline consists of four processing stages.

### 2.1 Events Accumulation

**ROS2 node:** `events_accumulation_node`

This module receives asynchronous events from the event camera and accumulates them within a configurable time window.

The accumulated events are converted into an image representation for subsequent processing.

In the validated configuration, the accumulation window was set to 15 ms.

### 2.2 Denoising and Filling

**ROS2 node:** `denoising_filling_node`

This module processes the accumulated event image to reduce noise and fill relevant regions.

The migrated implementation supports GPU processing.

During validation, the node was initialized with the following parameters:

- Processing mode: GPU
- `nd = 2`
- `nf = 3`

### 2.3 Distance Transform

**ROS2 node:** `distance_surface_original_node`

This module computes a distance-based representation from the processed event image.

The validated configuration used GPU processing with the exponential formulation.

The output is published as a ROS2 image message for the optical flow module.

### 2.4 Optical Flow Computation

**ROS2 node:** `optical_flow_gpu_node`

The final module computes optical flow from the distance surface using GPU acceleration.

The migrated implementation uses the external `optical-flow-filter` GPU library.

The validated configuration used three processing levels and produced optical flow frames at a resolution of 1280 × 720.

### 2.5 ROS2 Communication Architecture

The four modules communicate through ROS2 topics.

The validated processing chain is:

```text
Event camera / ROS2 event dataset
                |
                v
    events_accumulation_node
                |
                v
          /edge_image
                |
                v
     denoising_filling_node
                |
                v
  /denoised_filled_edge_image
                |
                v
 distance_surface_original_node
                |
                v
    /distance_surface_original
                |
                v
       optical_flow_gpu_node
                |
                v
          /optical_flow
```

The image-processing stages exchange `sensor_msgs/msg/Image` messages.

The event input uses the custom ROS2 message type `rt_of_event_msgs/msg/EventArray`.

## 3. Technical Challenges and Solutions

### 3.1 ROS1 and ROS2 Communication Differences

**Challenge**

The original project was developed for ROS1. Its communication mechanisms and build configuration could not be transferred directly to ROS2 Jazzy.

**Solution**

The processing stages were implemented as ROS2 nodes using ROS2 publishers, subscribers, and parameters.

The complete processing pipeline can now be started through a ROS2 launch file:

```bash
ros2 launch rt_of_ros2 full_pipeline.launch.py
```

**Validation**

All four processing nodes started successfully in a separate ROS2 workspace.

### 3.2 Custom Event Message Migration

**Challenge**

The original dataset uses event-camera messages that require compatible message definitions in ROS2.

**Solution**

A dedicated ROS2 package, `rt_of_event_msgs`, was created to provide the required `Event` and `EventArray` message definitions.

A separate package, `davis_event_adapter`, was also included for event-camera message adaptation.

**Validation**

The ROS2 pipeline successfully processed the converted checkerboard event dataset.

### 3.3 CUDA Compiler Detection

**Challenge**

During the independent release build, CMake initially failed to detect the CUDA compiler.

The reported error was:

```text
No CMAKE_CUDA_COMPILER could be found
```

**Solution**

CUDA Toolkit 12.6 was configured explicitly:

```bash
export CUDA_HOME=/usr/local/cuda-12.6
export PATH="$CUDA_HOME/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64:${LD_LIBRARY_PATH:-}"
export CUDACXX="$CUDA_HOME/bin/nvcc"
```

The build command also explicitly specified the CUDA compiler and toolkit location.

**Validation**

The independent build completed successfully, and `nvcc --version` confirmed CUDA 12.6.

### 3.4 External GPU Library Integration

**Challenge**

The optical flow module depends on the external `optical-flow-filter` GPU library.

During development, the build initially encountered a missing CUDA header:

```text
fatal error: cuda.h: No such file or directory
```

The GPU library also needed to be located and linked correctly.

**Solution**

The CUDA include paths and external GPU library location were configured in the build environment.

The ROS2 CMake configuration supports an explicit `FLOWFILTER_ROOT` parameter.

For example:

```bash
-DFLOWFILTER_ROOT=/absolute/path/to/optical-flow-filter
```

**Validation**

The optical flow executable successfully linked to `libflowfilter_gpu.so` and the CUDA runtime.

The GPU optical flow node subsequently initialized and published flow frames.

### 3.5 ROS2 Dataset Compatibility

**Challenge**

The original checkerboard dataset was stored in ROS1 bag format and could not be used directly as an ordinary ROS2 bag.

**Solution**

A ROS2-compatible version of the checkerboard dataset was prepared.

The converted dataset was used to test the processing pipeline.

**Validation**

The complete ROS2 pipeline processed the event data and generated 1280 × 720 optical flow frames.

### 3.6 Independent Build Verification

**Challenge**

A successful build in the original development workspace does not necessarily demonstrate that the source files copied into a release repository are complete.

**Solution**

A separate workspace, `rt_of_release_test_ws`, was created.

The three ROS2 packages were linked from the GitHub release directory and rebuilt with the required CUDA and GPU library configuration.

**Validation**

All three packages built successfully:

- `rt_of_event_msgs`
- `davis_event_adapter`
- `rt_of_ros2`

The complete pipeline was then launched from this independent workspace and tested with real event data.

This verification reduces the risk of accidentally relying on source files that exist only in the original development workspace.

## 4. Experimental Validation

### 4.1 Test Environment

| Component | Configuration |
|---|---|
| Operating system | Xubuntu 24.04 |
| ROS distribution | ROS2 Jazzy |
| GPU | NVIDIA Quadro RTX 5000 |
| CUDA Toolkit | 12.6 |
| Dataset | Converted checkerboard ROS2 bag |
| Image resolution | 1280 × 720 |
| Dataset playback rate | 1.0× |

### 4.2 Pipeline Startup

The following four nodes were confirmed to be running:

```text
/events_accumulation_node
/denoising_filling_node
/distance_surface_original_node
/optical_flow_gpu_node
```

The expected intermediate and output topics were also present.

### 4.3 Optical Flow Output

The GPU optical flow node successfully published frames at a resolution of 1280 × 720.

The observed execution logs included frames 984–988, confirming that the pipeline continued processing beyond initialization.

### 4.4 Optical Flow Publication Frequency

The publication frequency was measured using:

```bash
ros2 topic hz /optical_flow
```

Selected measurements were:

| Measurement window | Average publication rate |
|---|---:|
| 232 messages | 57.026 FPS |
| 458 messages | 56.233 FPS |
| 692 messages | 56.737 FPS |
| 876 messages | 57.563 FPS |

The final observed cumulative-window measurement was approximately **57.56 FPS**.

This value represents the measured ROS2 topic publication frequency, not the theoretical maximum GPU computation rate.

### 4.5 GPU Optical Flow Computation Time

Selected GPU computation measurements were:

| Frame | Computation time |
|---|---:|
| 984 | 8.035518 ms |
| 985 | 7.986675 ms |
| 986 | 7.974930 ms |
| 987 | 7.881018 ms |
| 988 | 8.017087 ms |

The mean computation time across these five samples was approximately **7.98 ms**.

These measurements represent a small consecutive sample and should not be interpreted as the average performance of the entire dataset.

### 4.6 Interpretation

The experimental results demonstrate that:

1. The migrated ROS2 packages can be compiled in a separate workspace.
2. All four processing modules can run together.
3. The pipeline can process converted event-camera data.
4. The GPU optical flow module can publish 1280 × 720 frames.
5. The complete system achieves an observed publication rate of approximately 57.56 FPS in the reported test.

A controlled comparison with the original ROS1 implementation would require equivalent hardware, datasets, processing parameters, and measurement methods.

## 5. Repository Organization

The ROS2 migration is maintained in a separate Git branch:

`ros2-jazzy-migration`

The original ROS1 implementation remains at the repository root.

The ROS2 source code is organized as follows:

```text
ros2/
├── README.md
├── rt_of_event_msgs/
│   └── msg/
├── davis_event_adapter/
│   └── src/
└── rt_of_ros2/
    ├── config/
    ├── cuda/
    ├── launch/
    ├── src/
    └── test/
```

The external GPU library is built separately and is not included as a compiled binary in the repository.

Large event datasets and generated build artifacts are also excluded.

## 6. Conclusion and Future Work

The ROS1-to-ROS2 Jazzy migration has reached a functional validation milestone.

The migrated implementation preserves the four-stage optical flow processing architecture and successfully operates with GPU acceleration.

Independent compilation, complete pipeline startup, dataset playback, and optical flow publication have been verified.

Future work may include:

- Further performance profiling and optimization.
- Additional validation with live event-camera input.
- More extensive comparisons with the original ROS1 implementation.
- Refactoring the processing algorithms into ROS-independent components.

The final objective of the potential ROS-independent refactoring would be to separate the core algorithms from middleware-specific communication interfaces, while preserving the existing processing behavior.

This future refactoring is not part of the currently validated ROS2 migration.