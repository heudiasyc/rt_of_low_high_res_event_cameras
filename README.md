# Real-Time Optical Flow for Vehicular Perception with Low- and High-Resolution Event Cameras

![Our pipeline](https://www.hds.utc.fr/~vbrebion/dokuwiki/_media/fr/pipeline.png?)

This repository holds the code associated to the "Real-Time Optical Flow for Vehicular Perception with Low- and High-Resolution Event Cameras" article. If you use this code as part of your work, please cite:

```BibTeX
@article{Brebion2022RealTimeOF,
  title={Real-Time Optical Flow for Vehicular Perception With Low- and High-Resolution Event Cameras},
  author={Vincent Brebion and Julien Moreau and Franck Davoine},
  journal={IEEE Transactions on Intelligent Transportation Systems},
  year={2022},
  volume={23},
  number={9},
  pages={15066-15078}
}
```

## Downloading our High-Speed High-Definition Event-Based Indoor dataset

Our dataset can be downloaded from the following webpage: <https://datasets.hds.utc.fr/share/er2aA4R0QMJzMyO>

## Overview

In this work, we propose an optimized framework for computing optical flow in real-time, with both low- and high-resolution event cameras, by computing dense image-like representations from the events through the use of our "inverse exponential distance surface".

## Installing and compiling the code

This code has originally been developed and tested with Ubuntu 20.04, ROS Noetic, CUDA 11.5+ and OpenCV 4.2.\
It is also compatible with Ubuntu 18.04, ROS Melodic, CUDA 11.5+ and OpenCV 3.2.\
While it *should* be compatible with older versions of Ubuntu/ROS/CUDA/OpenCV, we cannot ensure that the code will work flawlessly with them.

To use our code, two main dependencies have to be installed:

- the correct [CUDA](https://developer.nvidia.com/cuda-downloads) version for your system;
- the real-time optical flow computation library, called [Optical-flow-filter](https://github.com/jadarve/optical-flow-filter) (installation instructions can be found on their GitHub page).

Once done, you will need an initialized catkin workspace. If you don't already have one, follow [this tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

You also need to install the `rpg_dvs_ros` package, which contains the ROS message definitions for receiving the events from a live camera or reading them from a rosbag. Please follow the instructions detailed on their [GitHub repository](https://github.com/uzh-rpg/rpg_dvs_ros).

You can finally download and compile our code:

```txt
cd ~/catkin_ws/src
git clone https://github.com/vbrebion/rt_of_low_high_res_event_cameras.git
catkin build rt_of_low_high_res_event_cameras
```

## Testing

In order to quickly and easily test our code, sample roslaunch files are given. To use any of them, simply type the following command after having compiled the code:

```txt
roslaunch rt_of_low_high_res_event_cameras [launchfile].launch
```

where `[launchfile].launch` is the file you want to use.

Provided launchfiles are:

- `davis240_live.launch`: simply plug in your DAVIS240 camera, and use this file to visualize the optical flow results produced with it
- `davis346_replay.launch`: allows to compute optical flow from events from a DAVIS346 camera, recorded in a rosbag (this launchfile can be used for instance to compute optical flow for the MVSEC dataset)
- `prophesee_gen4_live.launch`: simply plug in your Prophesee Gen4 camera, and use this file to visualize the optical flow results produced with it
- `prophesee_gen4_replay.launch`: allows to compute optical flow from events from a Prophesee Gen4 camera, recorded in a rosbag (for instance, for our High-Speed High-Definition Event-Based Indoor dataset) or in a .dat file (for instance, for the 1 Megapixel Automotive Detection dataset)

Do not hesitate to modify these launchfiles to your needs, or to use them as templates for creating new ones for other cameras for instance (an explanation for each of the parameters is given in the corresponding node `..._node.cpp` file).

## Adding support for reading Prophesee's .dat files

By default, support for replaying Prophesee's .dat files is disabled, as they require the Metavision SDK, which is not openly available.

If you need to use such files with our code (for instance, for using Prophesee's [1 Megapixel Automotive Detection dataset](https://www.prophesee.ai/2020/11/24/automotive-megapixel-event-based-dataset/)), you first need to request an access to the SDK on [their website](https://www.prophesee.ai/metavision-intelligence/).

Once done, simply modify the `METAVISION_SDK_AVAILABLE` variable from `false` to `true` in the [include/rt_of_low_high_res_event_cameras/defines.hpp](./include/rt_of_low_high_res_event_cameras/defines.hpp) file, and recompile the code to make this change effective:

```txt
catkin build rt_of_low_high_res_event_cameras
```

## Code details

The code was developed so as to be as modular as possible. Each of the four modules of our pipeline architecture (edges image formation, denoising & filling, distance transform, optical flow computation) was implemented as an independent ROS node. A fifth node was also added, to visualize the optical flow field as images. While this choice of separate ROS nodes may not be as optimized as using nodelets for instance (or simply not using ROS), it allows for modularity and for making our code easier to use and modify in a wide variety of situations.

Each node was implemented using the following format:

- the core of the node, responsible for initialization, receiving and publishing messages, ... is located in the corresponding `..._node.cpp` file
  - the node responsible for receiving the events and creating the edge images slightly deviates from this pattern, to be able to handle both the reception of events from a live camera and from a file: two distinct nodes are therefore available, `edges_node_live` and `edges_node_replay`
- the function responsible for computing the output of the node (edge image, denoised & filled edge image, distance surface, optical flow matrix, optical flow visualization), on CPU and/or GPU, is located in the corresponding `..._cpu.cpp` or `..._gpu.cpp` file
- a `utils.cpp` file is also present, and contains utility functions that may be used throughout all nodes

Each file and each function was properly documented, so do not hesitate to take a look at them!
