#!/usr/bin/env bash
# Manual fallback — normally map_processor.py runs automatically when lidar_mapper
# is stopped with Ctrl+C. Only needed if the auto-trigger failed.
#
# Uses the installed package path (consistent with where lidar_mapper writes maps).

source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash

MAP_DIR=$(python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('scan_map'))")/maps
PROCESSOR=$(python3 -c "from ament_index_python.packages import get_package_share_directory; print(get_package_share_directory('scan_map'))")/scripts/map_processor.py

if [ ! -f "$MAP_DIR/raw_map.pgm" ]; then
    echo "ERROR: $MAP_DIR/raw_map.pgm not found. Stop lidar_mapper first (Ctrl+C saves it)."
    exit 1
fi

echo "Generating processed maps..."
python3 "$PROCESSOR"
echo "Done — maps written to $MAP_DIR"
