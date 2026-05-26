source /ros2_ws/00_config.bash
source /ros2_ws/setup.bash


ros2 launch my_car run.launch.py start_side:=${START_SIDE:-$start_side} runtime:=${RUNTIME:-$runtime}