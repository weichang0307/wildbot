import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('my_car'), 'config', 'nav2_params.yaml'
    )

    start_side = LaunchConfiguration('start_side')

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_side',
            default_value='right',
            description="Which side the car starts from: 'right' or 'left'",
        ),
        Node(
            package='my_car',
            executable='run',
            name='my_car_run',
            output='screen',
            parameters=[{'start_side': start_side}],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_planning',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['planner_server'],
            }],
        ),
    ])
