# Executes right after manual driving is complete

MAP_DIR="/ros2_ws/scan_map/maps"
cd $MAP_DIR

echo "Saving raw SLAM map..."
ros2 run nav2_map_server map_saver_cli -f raw_map

echo "Generating processed maps..."
python3 /ros2_ws/scan_map/map_processor.py

echo "Final maps saved to $MAP_DIR"