# multi_node_remap_with_params.launch.py
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    share_folder = get_package_share_directory('gps_calcul_pos')
    config_file = os.path.join(share_folder, 'config', 'gps_compute.yaml')
    
    gps_compute_node = Node(
        package='gps_calcul_pos',
        executable='gps_compute',
        name='gps_compute',
        output='screen',
        parameters=[config_file]
    )
    
    return LaunchDescription([
        gps_compute_node
    ])