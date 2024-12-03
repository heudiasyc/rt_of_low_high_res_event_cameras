# We use a Ubuntu 20.04 / CUDA 12.6 image as a base
FROM nvidia/cuda:12.6.2-devel-ubuntu20.04

# We change the default shell from sh to bash
SHELL ["/bin/bash", "-c"]

# We install some required packages
RUN apt update
RUN DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC apt install -y build-essential git nano software-properties-common tzdata wget

# We setup ROS by following the wiki instructions (https://wiki.ros.org/noetic/Installation/Ubuntu)
RUN echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros-latest.list
RUN wget --no-check-certificate -qO - https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN apt update
RUN apt install -y ros-noetic-ros-base ros-noetic-catkin ros-noetic-rqt-image-view python3-catkin-tools
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN source ~/.bashrc

# We setup the optical-flow-filter library (https://github.com/jadarve/optical-flow-filter)
RUN git clone https://github.com/jadarve/optical-flow-filter.git
WORKDIR /optical-flow-filter
RUN mkdir build
WORKDIR /optical-flow-filter/build
RUN cmake ..
RUN make install

# We setup the catkin workspace (https://wiki.ros.org/catkin/Tutorials/create_a_workspace)
WORKDIR /
RUN mkdir -p catkin_ws/src
WORKDIR /catkin_ws
RUN catkin config --init --mkdirs --extend /opt/ros/noetic --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release

# We setup the rpg_dvs_ros package (https://github.com/uzh-rpg/rpg_dvs_ros)
RUN add-apt-repository ppa:inivation-ppa/inivation
RUN apt install -y libcaer-dev ros-noetic-cv-bridge ros-noetic-image-common
WORKDIR /catkin_ws/src
RUN git clone https://github.com/catkin/catkin_simple.git
RUN git clone https://github.com/uzh-rpg/rpg_dvs_ros.git

# We setup our rt_of_low_high_res_event_cameras package (https://github.com/heudiasyc/rt_of_low_high_res_event_cameras)
RUN mkdir /catkin_ws/src/rt_of_low_high_res_event_cameras
WORKDIR /catkin_ws/src/rt_of_low_high_res_event_cameras
COPY ./ /catkin_ws/src/rt_of_low_high_res_event_cameras/

# We build the rt_of_low_high_res_event_cameras package
WORKDIR /catkin_ws
RUN catkin build rt_of_low_high_res_event_cameras

# Finally, when running the container, we should source the catkin workspace
RUN echo "source devel/setup.bash" >> ~/.bashrc
