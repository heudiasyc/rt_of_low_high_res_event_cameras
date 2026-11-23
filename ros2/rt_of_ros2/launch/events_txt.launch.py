from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(get_package_share_directory('rt_of_ros2'), 'config', 'events_txt.yaml')
    return LaunchDescription([Node(package='rt_of_ros2', executable='events_txt_node', namespace='rt_of_low_high_res_event_cameras', output='screen', parameters=[config])])
