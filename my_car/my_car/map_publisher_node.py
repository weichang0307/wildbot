#!/usr/bin/env python3

import csv
import os
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapPublisherNode(Node):
    def __init__(self):
        super().__init__('map_publisher')
        
        # Map parameters
        self.declare_parameter('map_width', 5.0)
        self.declare_parameter('map_height', 5.0)
        self.declare_parameter('resolution', 0.1)
        self.declare_parameter('origin_x', -2.5)
        self.declare_parameter('origin_y', -2.5)
        
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.resolution = self.get_parameter('resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        
        # Create publisher for occupancy grid
        self.map_pub = self.create_publisher(
            OccupancyGrid,
            'obstacle_map',
            10
        )
        
        # Timer to publish map periodically
        self.timer = self.create_timer(1.0, self.publish_map)
        
        # Load obstacles from CSV
        self.obstacles = self.load_obstacles()
        self.get_logger().info(f'Loaded {len(self.obstacles)} obstacles')
        
        # Create the occupancy grid
        self.occupancy_grid = self.create_occupancy_grid()
    
    def load_obstacles(self):
        """Load obstacles from map.csv file"""
        obstacles = []
        
        # Try multiple possible paths
        possible_paths = [
            os.path.join(os.path.dirname(__file__), '..', 'map.csv'),
            '/ros2_ws/my_car/map.csv',
            os.path.expanduser('~/my_car/map.csv'),
        ]
        
        csv_file = None
        for path in possible_paths:
            if os.path.exists(path):
                csv_file = path
                break
        
        if csv_file is None:
            self.get_logger().warn(f'Could not find map.csv in: {possible_paths}')
            return obstacles
        
        try:
            with open(csv_file, 'r') as f:
                reader = csv.reader(f)
                for i, row in enumerate(reader):
                    if len(row) >= 4:
                        obstacle = {
                            'type': row[0].strip(),
                            'x': float(row[1]) - 2.0,  # Adjust for map origin
                            'y': float(row[2]) - 2.0,  # Adjust for map origin
                            'yaw': float(row[3])
                        }
                        obstacles.append(obstacle)
            self.get_logger().info(f'Successfully loaded {len(obstacles)} obstacles from {csv_file}')
        except Exception as e:
            self.get_logger().error(f'Error loading CSV file: {e}')
        
        return obstacles
    
    def get_grid_index(self, x, y):
        """Convert world coordinates to grid indices"""
        grid_x = int((x - self.origin_x) / self.resolution)
        grid_y = int((y - self.origin_y) / self.resolution)
        
        width = int(self.map_width / self.resolution)
        height = int(self.map_height / self.resolution)
        
        # Check bounds
        if 0 <= grid_x < width and 0 <= grid_y < height:
            return grid_y * width + grid_x
        return None
    
    def point_in_rectangle(self, px, py, center_x, center_y, length, width, yaw_deg):
        """Check if point (px, py) is inside rotated rectangle"""
        # Convert yaw from degrees to radians
        yaw_rad = math.radians(yaw_deg)
        
        # Translate point to rectangle's local coordinates
        dx = px - center_x
        dy = py - center_y
        
        # Rotate point back to rectangle's frame
        cos_yaw = math.cos(-yaw_rad)
        sin_yaw = math.sin(-yaw_rad)
        local_x = dx * cos_yaw - dy * sin_yaw
        local_y = dx * sin_yaw + dy * cos_yaw
        
        # Check if point is within rectangle bounds
        return abs(local_x) <= length / 2.0 and abs(local_y) <= width / 2.0

    def is_outside_drive_area(self, x, y):
        """Treat everything outside the central 4x4 area as blocked."""
        return abs(x) > 2.0 or abs(y) > 2.0
    
    def create_occupancy_grid(self):
        """Create occupancy grid from obstacles"""
        width = int(self.map_width / self.resolution)
        height = int(self.map_height / self.resolution)
        
        # Initialize grid with free cells (0)
        data = [0] * (width * height)

        # Mark everything outside the drive area as occupied.
        for grid_y in range(height):
            world_y = self.origin_y + (grid_y + 0.5) * self.resolution
            row_offset = grid_y * width
            for grid_x in range(width):
                world_x = self.origin_x + (grid_x + 0.5) * self.resolution
                if self.is_outside_drive_area(world_x, world_y):
                    data[row_offset + grid_x] = 100
        
        # Mark obstacles as occupied (100)
        for obstacle in self.obstacles:
            x, y = obstacle['x'], obstacle['y']
            yaw = obstacle['yaw']
            
            # Get obstacle dimensions based on type
            if 'bridge' in obstacle['type'].lower():
                length = 2.0  # Bridge length (m)
                rect_width = 0.8  # Bridge width (m)
            elif 'pyramid' in obstacle['type'].lower():
                length = 0.6  # Pyramid length (m)
                rect_width = 0.6  # Pyramid width (m)
            else:
                length = 0.6
                rect_width = 0.6
            
            # Check all cells that could potentially be in the rectangle
            # Use a bounding box to limit iterations
            max_dim = max(length, rect_width)
            cell_range = int(max_dim / self.resolution) + 2
            
            for dx in range(-cell_range, cell_range + 1):
                for dy in range(-cell_range, cell_range + 1):
                    cell_x = x + dx * self.resolution
                    cell_y = y + dy * self.resolution
                    
                    # Check if this cell is within the rectangle
                    if self.point_in_rectangle(cell_x, cell_y, x, y, length, rect_width, yaw):
                        idx = self.get_grid_index(cell_x, cell_y)
                        if idx is not None:
                            data[idx] = 100
        
        # Create OccupancyGrid message
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = self.resolution
        grid.info.width = width
        grid.info.height = height
        
        # Origin (bottom-left corner in map frame)
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0
        
        grid.data = data
        
        return grid
    
    def publish_map(self):
        """Publish the occupancy grid"""
        self.occupancy_grid.header.stamp = self.get_clock().now().to_msg()
        self.map_pub.publish(self.occupancy_grid)
        self.get_logger().debug(f'Published occupancy grid')


def main(args=None):
    rclpy.init(args=args)
    node = MapPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
