from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument("input_file"),

        DeclareLaunchArgument(
            "replay_rate", default_value="1.0"),

        DeclareLaunchArgument(
            "max_duration_sec", default_value="0.0"),

        DeclareLaunchArgument(
            "accumulation_window_ms", default_value="100"),

        # 1. HDF5 event replay
        Node(
            package="rt_of_ros2",
            executable="events_h5_node",
            output="screen",
            parameters=[{
                "input_file": LaunchConfiguration("input_file"),
                "width": 640,
                "height": 480,
                "output_topic": "/dsec/events",
                "startup_delay_ms": 3000,
                "replay_rate": ParameterValue(
                    LaunchConfiguration("replay_rate"),
                    value_type=float),
                "max_duration_sec": ParameterValue(
                    LaunchConfiguration("max_duration_sec"),
                    value_type=float),
            }],
        ),

        # 2. Events accumulation
        Node(
            package="rt_of_ros2",
            executable="events_accumulation_node",
            output="screen",
            parameters=[{
                "input_topic": "/dsec/events",
                "output_topic": "/edge_image",
                "accumulation_window_ms": ParameterValue(
                    LaunchConfiguration("accumulation_window_ms"),
                    value_type=int),
            }],
        ),

        # 3. Denoising & filling
        Node(
            package="rt_of_ros2",
            executable="denoising_filling_node",
            output="screen",
            parameters=[{
                "processing_mode": "gpu",
                "denoising_min_neighbours": -1,
                "filling_min_neighbours": -1,
            }],
        ),

        # 4. Distance transform
        Node(
            package="rt_of_ros2",
            executable="distance_surface_original_node",
            output="screen",
            parameters=[{
                "distance_processing_mode": "gpu",
                "distance_surface_formulation": "exponential",
            }],
        ),

        # 5. Optical flow computation
        Node(
            package="rt_of_ros2",
            executable="optical_flow_gpu_node",
            output="screen",
            parameters=[{
                "pyramidal_flow_levels": 3,
                "max_flow": 15.0,
                "gammas": [50.0, 250.0, 500.0],
                "smooth_iterations": [50, 25, 5],
            }],
            remappings=[
                ("/distance_surface",
                 "/distance_surface_original"),
            ],
        ),
    ])
