import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _source_maps_dir():
    share = get_package_share_directory('scan_map')
    ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
    src = os.path.join(ws_root, 'scan_map')
    if os.path.isdir(os.path.join(src, 'maps')):
        return os.path.join(src, 'maps')
    return os.path.join(share, 'maps')


def generate_launch_description():
    map_path = os.path.join(_source_maps_dir(), 'lidar_map.pgm')

    return LaunchDescription([
        Node(
            package='localization',
            executable='lidar_localizer.py',
            name='lidar_localizer',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'base_frame': 'car_base',
                'map_frame':  'map',
                'field_size': 4.0,
                'map_path':   map_path,
            }]
        )
    ])
