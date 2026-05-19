import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('scan_map')
    rf2o_config = os.path.join(pkg_path, 'config', 'rf2o_odometry.yaml')
    loc_config = os.path.join(pkg_path, 'config', 'localization.yaml')

    lifecycle_nodes = ['map_server', 'amcl']

    return LaunchDescription([
        # 1. Odometry
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            parameters=[rf2o_config]
        ),
        
        # 2. Map Server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[loc_config]
        ),

        # 3. AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            parameters=[loc_config]
        ),

        # 4. Lifecycle Manager (Crucial for Nav2 nodes)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            parameters=[{'use_sim_time': False},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]
        )
    ])