import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    home = os.path.expanduser('~')
    lib = os.path.join(home, 'ros2_ws/external/optical-flow-filter/build/lib')
    cuda = '/usr/local/cuda-12.6/lib64'
    old = os.environ.get('LD_LIBRARY_PATH', '')

    return LaunchDescription([
        DeclareLaunchArgument(
            'processing_mode',
            default_value='cpu',
            description='Denoising/filling mode: cpu or gpu'
        ),
        DeclareLaunchArgument(
            'distance_processing_mode',
            default_value='cpu',
            description='Distance transform mode: cpu or gpu'
        ),

        SetEnvironmentVariable(
            'LD_LIBRARY_PATH',
            ':'.join(x for x in (lib, cuda, old) if x)
        ),

        # Module 2: Denoising & Filling (CPU/GPU selectable)
        Node(
            package='rt_of_ros2',
            executable='denoising_filling_node',
            name='denoising_filling_node',
            output='screen',
            parameters=[{
                'processing_mode': LaunchConfiguration('processing_mode'),
                'denoising_min_neighbours': 2,
                'filling_min_neighbours': 3,
            }],
        ),

        # Module 3: Distance Transform (CPU/GPU selectable)
        Node(
            package='rt_of_ros2',
            executable='distance_surface_original_node',
            name='distance_surface_original_node',
            output='screen',
            parameters=[{
                'distance_surface_formulation': 'exponential',
                'distance_processing_mode':
                    LaunchConfiguration('distance_processing_mode'),
            }],
        ),

        # Module 4: Optical Flow (GPU)
        Node(
            package='rt_of_ros2',
            executable='optical_flow_gpu_node',
            name='optical_flow_gpu_node',
            output='screen',
            remappings=[
                ('/distance_surface', '/distance_surface_original'),
            ],
        ),
    ])
