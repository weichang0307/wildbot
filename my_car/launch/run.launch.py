from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_car',
            executable='run',
            name='my_car_run',
            output='screen',
        ),
    ])