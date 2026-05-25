#!/usr/bin/env python3

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from tf2_ros import Buffer, TransformException, TransformListener


class GoalSelectorNode(Node):
    """Decides the navigation goal from explicit poses, the bear map, or random exploration.

    Publishes the current goal on ``path_goal`` (PoseStamped) whenever it changes.
    """

    def __init__(self):
        super().__init__('goal_selector')

        self.declare_parameter('robot_frame', 'car_base')
        self.declare_parameter('bear_map_topic', '/bear_map')
        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('safety_margin', 0.32)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('goal_search_radius', 1.0)
        self.declare_parameter('random_goal_min_distance', 1.0)

        self.robot_frame = self.get_parameter('robot_frame').value
        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.map_msg = None
        self.start_cell = None
        self.goal_cell = None
        self.random_goal_active = False
        self.last_bear_map_had_points = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, 'path_goal', 10)

        self.create_subscription(OccupancyGrid, 'obstacle_map', self.map_callback, 10)
        self.create_subscription(PoseStamped, 'move_base_simple/goal', self.goal_callback, 10)
        self.create_subscription(
            PointCloud,
            self.get_parameter('bear_map_topic').value,
            self.bear_map_callback,
            10,
        )
        self.create_timer(0.2, self.update_start_from_tf)

        self.get_logger().info(
            f'Goal selector ready; robot frame: {self.robot_frame}'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def map_callback(self, msg):
        self.map_msg = msg

    def update_start_from_tf(self):
        if self.map_msg is None:
            return

        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_msg.header.frame_id or 'map',
                self.robot_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        start_cell = self.world_to_grid(
            transform.transform.translation.x,
            transform.transform.translation.y,
        )
        if start_cell is None:
            return

        self.start_cell = start_cell

        if self.random_goal_active and self.goal_cell is not None:
            if self.is_close_enough_to_goal(self.start_cell, self.goal_cell, scale=3.0):
                next_goal = self.find_random_free_cell(exclude_cell=self.goal_cell)
                if next_goal is not None:
                    self.goal_cell = next_goal
                    self.publish_goal()
                else:
                    self.get_logger().warn('Random goal reached, but no next random free goal found')

    def goal_callback(self, msg):
        if self.map_msg is None:
            return
        requested = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        if requested is None:
            self.get_logger().warn('Goal pose is outside the map bounds')
            return
        goal_cell = self.find_nearest_free_cell(requested)
        if goal_cell is None:
            self.get_logger().warn('No free goal cell found near requested goal')
            return
        self.goal_cell = goal_cell
        self.random_goal_active = False
        self.publish_goal()

    def bear_map_callback(self, msg):
        if self.map_msg is None:
            return

        if not msg.points:
            if self.last_bear_map_had_points or self.goal_cell is None:
                random_goal = self.find_random_free_cell()
                if random_goal is None:
                    self.get_logger().warn('Bear map empty and no random free goal found')
                    self.last_bear_map_had_points = False
                    return
                if random_goal != self.goal_cell:
                    self.goal_cell = random_goal
                    self.random_goal_active = True
                    self.publish_goal()
            self.last_bear_map_had_points = False
            return

        self.last_bear_map_had_points = True

        goal_frame = msg.header.frame_id or self.map_msg.header.frame_id or 'map'
        try:
            transform = self.tf_buffer.lookup_transform(
                goal_frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException:
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        closest_point = min(
            msg.points,
            key=lambda p: math.hypot(p.x - robot_x, p.y - robot_y),
        )

        requested = self.world_to_grid(closest_point.x, closest_point.y)
        if requested is None:
            self.get_logger().warn('Closest bear is outside the map bounds')
            return

        goal_cell = self.find_nearest_free_cell(requested)
        if goal_cell is None:
            self.get_logger().warn('No free goal cell found near closest bear')
            return

        if goal_cell != self.goal_cell:
            self.goal_cell = goal_cell
            self.random_goal_active = False
            self.publish_goal()

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def publish_goal(self):
        if self.goal_cell is None or self.map_msg is None:
            return
        msg = PoseStamped()
        msg.header.frame_id = self.map_msg.header.frame_id or 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y = self.grid_to_world(*self.goal_cell)
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Published path_goal at grid {self.goal_cell}')

    # ------------------------------------------------------------------
    # Map utilities
    # ------------------------------------------------------------------

    def world_to_grid(self, x, y):
        origin = self.map_msg.info.origin.position
        resolution = self.map_msg.info.resolution
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        grid_x = int(math.floor((x - origin.x) / resolution))
        grid_y = int(math.floor((y - origin.y) / resolution))
        if 0 <= grid_x < width and 0 <= grid_y < height:
            return (grid_x, grid_y)
        return None

    def grid_to_world(self, grid_x, grid_y):
        origin = self.map_msg.info.origin.position
        resolution = self.map_msg.info.resolution
        return (
            origin.x + (grid_x + 0.5) * resolution,
            origin.y + (grid_y + 0.5) * resolution,
        )

    def cell_to_index(self, cell):
        return cell[1] * self.map_msg.info.width + cell[0]

    def is_occupied(self, cell):
        cost = self.map_msg.data[self.cell_to_index(cell)]
        return cost < 0 or cost >= self.occupancy_threshold

    def inflation_radius_cells(self):
        return int(math.ceil(self.safety_margin / self.map_msg.info.resolution))

    def goal_tolerance_cells(self):
        return self.goal_tolerance / self.map_msg.info.resolution

    def goal_search_radius_cells(self):
        return int(math.ceil(
            float(self.get_parameter('goal_search_radius').value) / self.map_msg.info.resolution
        ))

    def is_cell_free(self, cell):
        if self.is_occupied(cell):
            return False

        inflation_radius = self.inflation_radius_cells()
        if inflation_radius <= 0:
            return True

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        for dx in range(-inflation_radius, inflation_radius + 1):
            for dy in range(-inflation_radius, inflation_radius + 1):
                neighbor_x = cell[0] + dx
                neighbor_y = cell[1] + dy
                if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                    return False
                if math.hypot(dx, dy) * self.map_msg.info.resolution > self.safety_margin:
                    continue
                if self.is_occupied((neighbor_x, neighbor_y)):
                    return False

        return True

    def find_nearest_free_cell(self, target_cell):
        if self.is_cell_free(target_cell):
            return target_cell

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        max_radius = max(1, self.goal_search_radius_cells())
        best_cell = None
        best_distance = float('inf')

        for radius in range(1, max_radius + 1):
            for dx in range(-radius, radius + 1):
                for dy in range(-radius, radius + 1):
                    if max(abs(dx), abs(dy)) != radius:
                        continue
                    candidate = (target_cell[0] + dx, target_cell[1] + dy)
                    if not (0 <= candidate[0] < width and 0 <= candidate[1] < height):
                        continue
                    if not self.is_cell_free(candidate):
                        continue
                    distance = math.hypot(dx, dy)
                    if distance < best_distance:
                        best_distance = distance
                        best_cell = candidate
            if best_cell is not None:
                return best_cell

        return None

    def find_random_free_cell(self, max_attempts=200, exclude_cell=None):
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        if width <= 0 or height <= 0:
            return None

        resolution = float(self.map_msg.info.resolution)
        min_distance_m = float(self.get_parameter('random_goal_min_distance').value)
        min_distance_cells = max(0.0, min_distance_m / resolution) if resolution > 0.0 else 0.0

        for _ in range(max_attempts):
            candidate = (random.randrange(width), random.randrange(height))
            if exclude_cell is not None and candidate == exclude_cell:
                continue
            if self.start_cell is not None:
                dx = candidate[0] - self.start_cell[0]
                dy = candidate[1] - self.start_cell[1]
                if math.hypot(dx, dy) < min_distance_cells:
                    continue
            if self.is_cell_free(candidate):
                return candidate

        return None

    def is_close_enough_to_goal(self, current, goal, scale=1.0):
        dx = current[0] - goal[0]
        dy = current[1] - goal[1]
        return math.hypot(dx, dy) <= self.goal_tolerance_cells() * scale


def main(args=None):
    rclpy.init(args=args)
    node = GoalSelectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
