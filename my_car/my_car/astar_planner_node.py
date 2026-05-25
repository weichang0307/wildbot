#!/usr/bin/env python3

import heapq
import math

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('astar_planner')

        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('allow_diagonal', True)
        self.declare_parameter('robot_frame', 'car_base')
        self.declare_parameter('safety_margin', 0.32)
        self.declare_parameter('goal_tolerance', 0.3)

        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.allow_diagonal = bool(self.get_parameter('allow_diagonal').value)
        self.robot_frame = self.get_parameter('robot_frame').value
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)

        self.map_msg = None
        self.start_cell = None
        self.goal_cell = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, 'astar_path', 10)
        self.create_subscription(OccupancyGrid, 'obstacle_map', self.map_callback, 10)
        self.create_subscription(PoseStamped, 'path_goal', self.path_goal_callback, 10)
        self.create_timer(0.2, self.update_start_from_tf)

        self.get_logger().info(
            f'Waiting for obstacle_map and path_goal; using TF pose from {self.robot_frame}'
        )

    def map_callback(self, msg):
        self.map_msg = msg

    def path_goal_callback(self, msg):
        if self.map_msg is None:
            return
        goal_cell = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        if goal_cell is None:
            self.get_logger().warn('Received path_goal is outside the map bounds')
            return
        self.goal_cell = goal_cell
        self.try_plan('path_goal')

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
            self.get_logger().warn('Car pose is outside the map bounds')
            return

        self.start_cell = start_cell

    def try_plan(self, reason):
        if self.map_msg is None or self.start_cell is None or self.goal_cell is None:
            return

        if not self.is_cell_free(self.start_cell):
            self.get_logger().warn('Start cell is occupied; path not published')
            return

        if not self.is_cell_free(self.goal_cell):
            self.get_logger().warn('Goal cell is occupied; path not published')
            return

        path_cells = self.a_star(self.start_cell, self.goal_cell)
        if path_cells is None:
            self.get_logger().warn(f'No A* path found after {reason}')
            return

        path_msg = self.cells_to_path(path_cells)
        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published A* path with {len(path_cells)} cells after {reason}')

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
        world_x = origin.x + (grid_x + 0.5) * resolution
        world_y = origin.y + (grid_y + 0.5) * resolution
        return world_x, world_y

    def cell_to_index(self, cell):
        return cell[1] * self.map_msg.info.width + cell[0]

    def is_occupied(self, cell):
        cost = self.map_msg.data[self.cell_to_index(cell)]
        return cost < 0 or cost >= self.occupancy_threshold

    def inflation_radius_cells(self):
        return int(math.ceil(self.safety_margin / self.map_msg.info.resolution))

    def goal_tolerance_cells(self):
        return self.goal_tolerance / self.map_msg.info.resolution

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

    # ------------------------------------------------------------------
    # A* search
    # ------------------------------------------------------------------

    def get_neighbors(self, cell):
        x, y = cell
        steps = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        if self.allow_diagonal:
            steps.extend([(-1, -1), (-1, 1), (1, -1), (1, 1)])

        width = self.map_msg.info.width
        height = self.map_msg.info.height

        for dx, dy in steps:
            neighbor = (x + dx, y + dy)
            if not (0 <= neighbor[0] < width and 0 <= neighbor[1] < height):
                continue
            if not self.is_cell_free(neighbor):
                continue
            if dx != 0 and dy != 0 and not self.diagonal_move_is_clear(cell, neighbor):
                continue
            yield neighbor, math.sqrt(2.0) if dx != 0 and dy != 0 else 1.0

    def diagonal_move_is_clear(self, cell, neighbor):
        x0, y0 = cell
        x1, y1 = neighbor
        return self.is_cell_free((x0, y1)) and self.is_cell_free((x1, y0))

    def heuristic(self, cell, goal):
        dx = abs(goal[0] - cell[0])
        dy = abs(goal[1] - cell[1])
        if self.allow_diagonal:
            return max(dx, dy)
        return dx + dy

    def is_close_enough_to_goal(self, current, goal, scale=1.0):
        dx = current[0] - goal[0]
        dy = current[1] - goal[1]
        return math.hypot(dx, dy) <= self.goal_tolerance_cells() * scale

    def a_star(self, start, goal):
        open_heap = []
        heapq.heappush(open_heap, (0.0, start))
        came_from = {}
        g_score = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if self.is_close_enough_to_goal(current, goal):
                return self.reconstruct_path(came_from, current)

            current_cost = g_score[current]
            for neighbor, step_cost in self.get_neighbors(current):
                tentative_cost = current_cost + step_cost
                if tentative_cost >= g_score.get(neighbor, float('inf')):
                    continue
                came_from[neighbor] = current
                g_score[neighbor] = tentative_cost
                priority = tentative_cost + self.heuristic(neighbor, goal)
                heapq.heappush(open_heap, (priority, neighbor))

        return None

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    def cells_to_path(self, cells):
        path = Path()
        path.header.frame_id = self.map_msg.header.frame_id or 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for grid_x, grid_y in cells:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x, pose.pose.position.y = self.grid_to_world(grid_x, grid_y)
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        return path


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
