source /ros2_ws/00_config.bash
source /ros2_ws/setup.bash

ros2 launch bridge bridge.launch.py start_side:=${START_SIDE:-$start_side_bridge}