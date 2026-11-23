import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    home = os.path.expanduser("~")

    flow_lib = os.path.join(
        home,
        "ros2_ws/external/optical-flow-filter/build/lib",
    )

    cuda_lib = "/usr/local/cuda-12.6/lib64"

    old_lib = os.environ.get("LD_LIBRARY_PATH", "")

    return LaunchDescription([
        DeclareLaunchArgument(
            "accumulation_window_ms",
            default_value="15",
        ),

        DeclareLaunchArgument(
            "processing_mode",
            default_value="gpu",
        ),

        DeclareLaunchArgument(
            "distance_processing_mode",
            default_value="gpu",
        ),

        SetEnvironmentVariable(
            "LD_LIBRARY_PATH",
            ":".join(
                x for x in (flow_lib, cuda_lib, old_lib) if x
            ),
        ),

        # 1. Events Accumulation
        Node(
            package="rt_of_ros2",
            executable="events_accumulation_node",
            name="events_accumulation_node",
            output="screen",
            parameters=[{
                "accumulation_window_ms":
                    LaunchConfiguration("accumulation_window_ms"),
                "input_topic":
                    "/prophesee/camera/cd_events_buffer",
                "output_topic": "/edge_image",
            }],
        ),

        # 2. Denoising & Filling
        Node(
            package="rt_of_ros2",
            executable="denoising_filling_node",
            name="denoising_filling_node",
            output="screen",
            parameters=[{
                "processing_mode":
                    LaunchConfiguration("processing_mode"),
                "denoising_min_neighbours": 2,
                "filling_min_neighbours": 3,
            }],
        ),

        # 3. Distance Transform
        Node(
            package="rt_of_ros2",
            executable="distance_surface_original_node",
            name="distance_surface_original_node",
            output="screen",
            parameters=[{
                "distance_processing_mode":
                    LaunchConfiguration(
                        "distance_processing_mode"
                    ),
                "distance_surface_formulation":
                    "exponential",
            }],
        ),

        # 4. Optical Flow
        Node(
            package="rt_of_ros2",
            executable="optical_flow_gpu_node",
            name="optical_flow_gpu_node",
            output="screen",
            remappings=[
                (
                    "/distance_surface",
                    "/distance_surface_original",
                ),
            ],
        ),
    ])
