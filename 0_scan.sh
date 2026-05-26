source /ros2_ws/00_config.bash
source /ros2_ws/setup.bash

date +"%Y-%m-%d %H:%M:%S" > time.txt
ros2 launch my_car scan.launch.py