
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    scan_map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('scan_map'),
                'launch',
                'scan.launch.py'
            )
        )
    )
    return LaunchDescription([
        scan_map_launch,
        Node(
            package='my_car',
            executable='run_scan',
            name='my_car_run',
            output='screen',
        ),
    ])