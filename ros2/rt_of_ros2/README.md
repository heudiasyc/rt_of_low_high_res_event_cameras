# ROS2 Jazzy migration — stage 1 ONLY

This is a small independently written ROS2 adapter based on the original project's event accumulation and CPU denoising/filling behavior. It is **not** the completed migration. It does not decode Prophesee ROS1 bags, implement undistortion, reproduce the original replay timing exactly, implement distance surfaces, CUDA, optical flow or visualization. Preserve the original CeCILL license when reusing original code.

## Prerequisites
Ubuntu 24.04 with ROS2 Jazzy, ros-jazzy-cv-bridge, libopencv-dev, python3-colcon-common-extensions.

## Build
```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select rt_of_ros2
source install/setup.bash
```

## Run synthetic sample
```bash
ros2 run rt_of_ros2 events_txt_node --ros-args \
  -p input_file:=$HOME/ros2_ws/src/rt_of_ros2/test/sample_events.txt \
  -p width:=240 -p height:=180 -p accumulation_window_ms:=25
```
In another terminal (source ROS2 environment):
```bash
ros2 topic list
ros2 topic hz /rt_of_low_high_res_event_cameras/edge_image
```
The tiny sample finishes after 3 timer ticks, so `topic hz` may not have enough samples; use `ros2 topic echo /rt_of_low_high_res_event_cameras/edge_image --once --field encoding` before running or make a longer sample.

## Critical limitations
- TXT input format: `timestamp_in_seconds x y polarity`.
- This uses non-overlapping windows beginning at each next event. The original ROS1 replay code has its own window boundary semantics; compare before claiming equivalence.
- The sample's `header.stamp` uses ROS2 wall time, not the event timestamp.
- All events are currently loaded in memory; use streaming for big datasets.
- Prophesee EventArray schema is **not known** from rosbag info alone; inspect with `rosmsg show prophesee_event_msgs/EventArray` inside the working ROS1 container.
