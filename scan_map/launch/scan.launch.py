import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('scan_map')
    rf2o_config = os.path.join(pkg_path, 'config', 'rf2o_odometry.yaml')
    slam_config = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')

    print("\n" + "="*60)
    print("  SCAN_MAP LAUNCH CONFIGURATION DEBUGGER")
    print("="*60 + "\n")

    return LaunchDescription([
        # Updated to ROS 2 Humble syntax to remove the deprecation warning
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_lidar_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0.3', 
                '--roll', '0', '--pitch', '0', '--yaw', '0', 
                '--frame-id', 'car_base', '--child-frame-id', 'lidar'
            ]
        ),

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_odometry', # Changed name to prevent rosout clash
            output='screen',
            parameters=[
                rf2o_config,
                {
                    'use_sim_time': True,
                    'laser_scan_topic': '/scan',
                    'odom_topic': '/odom',
                    'base_frame_id': 'car_base',
                    'odom_frame_id': 'odom',
                }
            ]
        ),
        
        # Node(
        #     package='slam_toolbox',
        #     executable='async_slam_toolbox_node',
        #     name='slam_toolbox_node',
        #     output='screen',
        #     parameters=[
        #         slam_config,
        #         {
        #             'use_sim_time': True,
        #             'scan_topic': '/scan',
        #             'odom_frame': 'odom',
        #             'map_frame': 'map',
        #             'base_frame': 'car_base',
        #             'qos_reliability_policy': 'reliable',
        #             'scan_queue_size': 10
        #         }
        #     ]
        # )
    ])