source /ros2_ws/00_config.bash
source /ros2_ws/setup.bash

# Kill any leftover bridge nodes so they don't conflict on shared topics
pkill -f "bridge" 2>/dev/null; sleep 0.5

# Ensure door's 'nodes' module takes priority over bridge's on PYTHONPATH
export PYTHONPATH=/ros2_ws/install/door/lib/python3.12/site-packages:$PYTHONPATH

ros2 launch door door.launch.py door_type:=${DOOR_TYPE:-$door_type} start_side:=${START_SIDE:-$start_side_door}
