import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('scan_map')
    rf2o_config = os.path.join(pkg_path, 'config', 'rf2o_odometry.yaml')
    slam_config = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')

    return LaunchDescription([
        # Odometry Provider
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[rf2o_config],
            output='screen'
        ),
        
        # Mapping Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config]
        )
    ])