from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scan_map',
            executable='lidar_mapper.py',
            name='lidar_mapper',
            output='screen',
            parameters=[{
                'scan_topic': '/scan',
                'base_frame': 'car_base',
                'map_frame':  'map',
                'field_size': 4.0,
                'resolution': 0.01,
                'margin':     0.1,
            }]
        )
    ])
