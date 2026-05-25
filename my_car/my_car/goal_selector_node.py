#!/usr/bin/env python3

import math
import random
import time as _time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from std_msgs.msg import String
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
        self.declare_parameter('safety_margin', 0.15)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('goal_search_radius', 1.0)
        self.declare_parameter('random_goal_min_distance', 0.5)
        self.declare_parameter('random_goal_max_distance', 3.0)
        self.declare_parameter('block_zone_radius',   0.5)   # m
        self.declare_parameter('block_zone_duration', 5.0)   # s

        self.robot_frame = self.get_parameter('robot_frame').value
        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.map_msg = None
        self.explore_map_msg = None
        self._last_bear_msg = None
        self.start_cell = None
        self.goal_cell = None
        self.random_goal_active = False
        self.last_bear_map_had_points = False
        self._control_state = ''  # latest state string from /my_control/state
        self._blocked_zones = []  # list of (cx, cy, expiry_unix_s)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_pub = self.create_publisher(PoseStamped, 'path_goal', 10)

        self.create_subscription(OccupancyGrid, '/fused_obstacle_map', self.map_callback, 10)
        self.create_subscription(OccupancyGrid, '/explore_map', self._on_explore_map, 10)
        self.create_subscription(PoseStamped, 'move_base_simple/goal', self.goal_callback, 10)
        self.create_subscription(String,      '/my_control/state', self._on_control_state, 10)
        self.create_subscription(PoseStamped, '/nav/block_zone',    self._on_block_zone,    10)
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

    def _on_explore_map(self, msg):
        self.explore_map_msg = msg

    def _on_control_state(self, msg):
        self._control_state = msg.data

    def _on_block_zone(self, msg):
        cx = msg.pose.position.x
        cy = msg.pose.position.y
        duration = float(self.get_parameter('block_zone_duration').value)
        self._blocked_zones.append((cx, cy, _time.time() + duration))
        self.get_logger().info(f'Blocking zone ({cx:.2f},{cy:.2f}) for {duration:.0f}s')

    def _is_finishing(self) -> bool:
        state = self._control_state.split(':')[-1]
        return state in ('FINISH', 'FINISH_ADJUST', 'FINISH_PARK', 'FINISH_UNSTOCK', 'FINISH_STOP')

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

        if self._is_finishing():
            return

        # If the current goal has become occupied, clear it and try to pick a new one
        if self.goal_cell is not None and not self.is_cell_free(self.goal_cell):
            self.get_logger().warn(f'Goal cell {self.goal_cell} is now occupied — picking new goal')
            self.goal_cell = None
            if not self._try_set_bear_goal():
                self._pick_and_publish_random_goal()
            return

        # In random mode, if the goal has already been explored, move on
        if self.random_goal_active and self.goal_cell is not None and self._is_cell_explored(self.goal_cell):
            self.goal_cell = None
            if not self._try_set_bear_goal():
                self._pick_and_publish_random_goal()
            return

        # In random mode, keep re-checking for bears even without a goal change trigger
        if self.random_goal_active:
            self._try_set_bear_goal()

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

        self._last_bear_msg = msg

        if self._is_finishing():
            return

        if not msg.points:
            if self.last_bear_map_had_points or self.goal_cell is None:
                if not self._pick_and_publish_random_goal():
                    self.get_logger().warn('Bear map empty and no valid unexplored free goal found')
            self.last_bear_map_had_points = False
            return

        goal_frame = msg.header.frame_id or self.map_msg.header.frame_id or 'map'
        try:
            transform = self.tf_buffer.lookup_transform(
                goal_frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException:
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y

        target_bear = None
        target_dist = float('inf')
        for bear in msg.points:
            if self._in_exclusion_zone(bear.x, bear.y):
                continue
            cell = self.world_to_grid(bear.x, bear.y)
            if cell is None or not self.is_cell_free(cell):
                continue
            dist = math.hypot(bear.x - robot_x, bear.y - robot_y)
            if dist < target_dist:
                target_dist = dist
                target_bear = bear

        if target_bear is None:
            if self.last_bear_map_had_points or self.goal_cell is None:
                self._pick_and_publish_random_goal()
            self.last_bear_map_had_points = False
            return

        self.last_bear_map_had_points = True

        requested = self.world_to_grid(target_bear.x, target_bear.y)
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
    # Exclusion zones
    # ------------------------------------------------------------------

    _EXCLUSION_ZONES = [
        (-2.0, -2.0, 1.2),  # (centre_x, centre_y, side)
        ( -2.0, 2.0, 1.2),
    ]

    def _in_exclusion_zone(self, x, y) -> bool:
        for cx, cy, side in self._EXCLUSION_ZONES:
            half = side / 2.0
            if abs(x - cx) <= half and abs(y - cy) <= half:
                return True
        return False

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

        # Check temporary blocked zones
        if self._blocked_zones:
            now = _time.time()
            self._blocked_zones = [(cx, cy, exp) for cx, cy, exp in self._blocked_zones if exp > now]
            wx, wy = self.grid_to_world(*cell)
            block_r = float(self.get_parameter('block_zone_radius').value)
            for cx, cy, _ in self._blocked_zones:
                if math.hypot(wx - cx, wy - cy) < block_r:
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

    def _try_set_bear_goal(self) -> bool:
        """Set goal to the closest available bear (explored or not). Returns True on success."""
        if self._last_bear_msg is None or self.map_msg is None:
            return False
        if not self._last_bear_msg.points:
            return False

        goal_frame = self._last_bear_msg.header.frame_id or self.map_msg.header.frame_id or 'map'
        try:
            tf = self.tf_buffer.lookup_transform(goal_frame, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return False

        robot_x = tf.transform.translation.x
        robot_y = tf.transform.translation.y

        target_bear = None
        target_dist = float('inf')
        for bear in self._last_bear_msg.points:
            if self._in_exclusion_zone(bear.x, bear.y):
                continue
            cell = self.world_to_grid(bear.x, bear.y)
            if cell is None or not self.is_cell_free(cell):
                continue
            dist = math.hypot(bear.x - robot_x, bear.y - robot_y)
            if dist < target_dist:
                target_dist = dist
                target_bear = bear

        if target_bear is None:
            return False

        requested = self.world_to_grid(target_bear.x, target_bear.y)
        goal_cell = self.find_nearest_free_cell(requested)
        if goal_cell is None or goal_cell == self.goal_cell:
            return False

        self.goal_cell = goal_cell
        self.random_goal_active = False
        self.publish_goal()
        return True

    def _pick_and_publish_random_goal(self, exclude_cell=None) -> bool:
        """Find a free, unexplored cell, verify it again, then publish. Returns True on success."""
        cell = self.find_random_unexplored_free_cell(exclude_cell=exclude_cell)
        if cell is None:
            return False
        if not self.is_cell_free(cell):
            return False
        if cell == self.goal_cell:
            return False
        self.goal_cell = cell
        self.random_goal_active = True
        self.publish_goal()
        return True

    def _is_cell_explored(self, cell) -> bool:
        """Return True if the obstacle-map cell centre is marked explored in the explore map."""
        if self.explore_map_msg is None or self.map_msg is None:
            return False
        wx, wy = self.grid_to_world(*cell)
        exp = self.explore_map_msg
        exp_res = exp.info.resolution
        exp_ox  = exp.info.origin.position.x
        exp_oy  = exp.info.origin.position.y
        exp_w   = exp.info.width
        exp_h   = exp.info.height
        gx = int(math.floor((wx - exp_ox) / exp_res))
        gy = int(math.floor((wy - exp_oy) / exp_res))
        if not (0 <= gx < exp_w and 0 <= gy < exp_h):
            return False
        return exp.data[gy * exp_w + gx] == 100

    def find_random_unexplored_free_cell(self, max_attempts=200, exclude_cell=None):
        """Return the closest unexplored free cell to the robot, falling back to any free cell."""
        if self.map_msg is None:
            return None

        explore = self.explore_map_msg
        if explore is None:
            return self.find_random_free_cell(max_attempts=max_attempts, exclude_cell=exclude_cell)

        min_distance_m = float(self.get_parameter('random_goal_min_distance').value)
        max_distance_m = float(self.get_parameter('random_goal_max_distance').value)

        rx, ry = (self.grid_to_world(*self.start_cell)
                  if self.start_cell is not None else (0.0, 0.0))

        exp_w   = explore.info.width
        exp_res = explore.info.resolution
        exp_ox  = explore.info.origin.position.x
        exp_oy  = explore.info.origin.position.y

        # Build (distance, world_x, world_y) for every unexplored cell within range
        candidates = []
        for idx, val in enumerate(explore.data):
            if val == 100:
                continue
            wx = exp_ox + (idx % exp_w + 0.5) * exp_res
            wy = exp_oy + (idx // exp_w + 0.5) * exp_res
            dist = math.hypot(wx - rx, wy - ry)
            if dist < min_distance_m or dist > max_distance_m:
                continue
            candidates.append((dist, wx, wy))

        if not candidates:
            return self.find_random_free_cell(max_attempts=max_attempts, exclude_cell=exclude_cell)

        candidates.sort()  # closest first

        for _, wx, wy in candidates:
            cell = self.world_to_grid(wx, wy)
            if cell is None or cell == exclude_cell:
                continue
            if self.is_cell_free(cell):
                return cell

        return self.find_random_free_cell(max_attempts=max_attempts, exclude_cell=exclude_cell)

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
