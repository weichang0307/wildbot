source /ros2_ws/00_config.bash
source /ros2_ws/setup.bash

# Kill any leftover door nodes so they don't conflict on shared topics
pkill -f "door" 2>/dev/null; sleep 0.5

# Ensure bridge's 'nodes' module takes priority over door's on PYTHONPATH
export PYTHONPATH=/ros2_ws/install/bridge/lib/python3.12/site-packages:$PYTHONPATH

ros2 launch bridge bridge.launch.py start_side:=${START_SIDE:-$start_side_bridge}