#!/usr/bin/env python3

import math
import random

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


class RRTStarPlannerNode(Node):
    def __init__(self):
        super().__init__('rrt_star_planner')

        self.declare_parameter('occupancy_threshold', 50)
        self.declare_parameter('robot_frame', 'car_base')
        self.declare_parameter('safety_margin', 0.3)
        self.declare_parameter('rrt_max_iterations', 3000)
        self.declare_parameter('rrt_step_size', 0.3)
        self.declare_parameter('rrt_rewire_radius', 0.6)
        self.declare_parameter('rrt_goal_sample_rate', 0.15)
        self.declare_parameter('rrt_goal_tolerance', 0.2)
        self.declare_parameter('path_topic', 'rrt_star_path')

        self.occupancy_threshold = int(self.get_parameter('occupancy_threshold').value)
        self.robot_frame = self.get_parameter('robot_frame').value
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.rrt_max_iterations = int(self.get_parameter('rrt_max_iterations').value)
        self.rrt_step_size = float(self.get_parameter('rrt_step_size').value)
        self.rrt_rewire_radius = float(self.get_parameter('rrt_rewire_radius').value)
        self.rrt_goal_sample_rate = float(self.get_parameter('rrt_goal_sample_rate').value)
        self.rrt_goal_tolerance = float(self.get_parameter('rrt_goal_tolerance').value)
        self.path_topic = self.get_parameter('path_topic').value

        self.map_msg = None
        self.start_cell = None
        self.goal_cell = None
        self.last_start_cell = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)
        self.create_subscription(OccupancyGrid, 'obstacle_map', self.map_callback, 10)
        self.create_subscription(PoseStamped, 'move_base_simple/goal', self.goal_callback, 10)
        self.create_timer(0.2, self.update_start_from_tf)

        self.get_logger().info(
            f'RRT* ready. Waiting for obstacle_map and move_base_simple/goal; using TF pose from {self.robot_frame}'
        )

    def map_callback(self, msg):
        self.map_msg = msg
        self.try_plan('map update')

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
        if self.start_cell != self.last_start_cell:
            self.last_start_cell = self.start_cell
            self.try_plan('car pose update')

    def goal_callback(self, msg):
        self.goal_cell = self.world_to_grid(msg.pose.position.x, msg.pose.position.y)
        if self.goal_cell is None:
            self.get_logger().warn('Goal pose is outside the map bounds')
            return
        self.try_plan('goal pose')

    def try_plan(self, reason):
        if self.map_msg is None or self.start_cell is None or self.goal_cell is None:
            return

        if not self.is_cell_free(self.start_cell):
            self.get_logger().warn('Start cell is occupied; path not published')
            return

        if not self.is_cell_free(self.goal_cell):
            self.get_logger().warn('Goal cell is occupied; path not published')
            return
        print(f'Planning RRT* path from {self.start_cell} to {self.goal_cell} after {reason}')
        path_cells = self.rrt_star(self.start_cell, self.goal_cell)
        if path_cells is None:
            self.get_logger().warn(f'No RRT* path found after {reason}')
            return

        path_msg = self.cells_to_path(path_cells)
        self.path_pub.publish(path_msg)
        self.get_logger().info(
            f'Published RRT* path with {len(path_cells)} cells on {self.path_topic} after {reason}'
        )

    def world_to_grid(self, x, y):
        if self.map_msg is None:
            return None

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
        width = self.map_msg.info.width
        return cell[1] * width + cell[0]

    def is_occupied(self, cell):
        cost = self.map_msg.data[self.cell_to_index(cell)]
        return cost < 0 or cost >= self.occupancy_threshold

    def inflation_radius_cells(self):
        resolution = self.map_msg.info.resolution
        return int(math.ceil(self.safety_margin / resolution))

    def meters_to_cells(self, value_m):
        resolution = self.map_msg.info.resolution
        return max(1, int(math.ceil(value_m / resolution)))

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

    def distance_cells(self, a, b):
        return math.hypot(a[0] - b[0], a[1] - b[1])

    def steer(self, from_cell, to_cell, step_size_cells):
        dx = to_cell[0] - from_cell[0]
        dy = to_cell[1] - from_cell[1]
        dist = math.hypot(dx, dy)
        if dist == 0.0:
            return from_cell
        if dist <= step_size_cells:
            return to_cell
        scale = step_size_cells / dist
        new_x = int(round(from_cell[0] + dx * scale))
        new_y = int(round(from_cell[1] + dy * scale))
        return (new_x, new_y)

    def is_segment_free(self, a, b):
        width = self.map_msg.info.width
        height = self.map_msg.info.height
        steps = int(max(abs(b[0] - a[0]), abs(b[1] - a[1])))
        if steps == 0:
            return self.is_cell_free(a)

        for i in range(steps + 1):
            t = i / float(steps)
            x = int(round(a[0] + (b[0] - a[0]) * t))
            y = int(round(a[1] + (b[1] - a[1]) * t))
            if not (0 <= x < width and 0 <= y < height):
                return False
            if not self.is_cell_free((x, y)):
                return False
        return True

    def sample_free_cell(self, goal):
        if random.random() < self.rrt_goal_sample_rate:
            return goal

        width = self.map_msg.info.width
        height = self.map_msg.info.height
        for _ in range(100):
            sample = (random.randint(0, width - 1), random.randint(0, height - 1))
            if self.is_cell_free(sample):
                return sample
        return goal

    def find_near_nodes(self, nodes, new_node, radius_cells):
        near_nodes = []
        for node in nodes:
            if self.distance_cells(node, new_node) <= radius_cells:
                near_nodes.append(node)
        return near_nodes

    def rrt_star(self, start, goal):
        nodes = [start]
        parent = {start: None}
        cost = {start: 0.0}
        best_goal_cost = float('inf')

        step_size_cells = self.meters_to_cells(self.rrt_step_size)
        rewire_radius_cells = self.meters_to_cells(self.rrt_rewire_radius)
        goal_tolerance_cells = self.meters_to_cells(self.rrt_goal_tolerance)

        for _ in range(self.rrt_max_iterations):
            print(f'RRT* iteration {_+1}/{self.rrt_max_iterations}', end='\r')
            sample = self.sample_free_cell(goal)
            nearest = min(nodes, key=lambda n: self.distance_cells(n, sample))
            new_node = self.steer(nearest, sample, step_size_cells)

            if new_node == nearest or not self.is_cell_free(new_node):
                continue
            if new_node in cost:
                continue
            if not self.is_segment_free(nearest, new_node):
                continue

            near_nodes = self.find_near_nodes(nodes, new_node, rewire_radius_cells)
            best_parent = nearest
            best_new_cost = cost[nearest] + self.distance_cells(nearest, new_node)

            for near in near_nodes:
                if not self.is_segment_free(near, new_node):
                    continue
                candidate_cost = cost[near] + self.distance_cells(near, new_node)
                if candidate_cost < best_new_cost:
                    best_parent = near
                    best_new_cost = candidate_cost

            parent[new_node] = best_parent
            cost[new_node] = best_new_cost
            nodes.append(new_node)

            for near in near_nodes:
                if near == best_parent:
                    continue
                if not self.is_segment_free(new_node, near):
                    continue
                candidate_cost = cost[new_node] + self.distance_cells(new_node, near)
                if candidate_cost < cost[near]:
                    parent[near] = new_node
                    cost[near] = candidate_cost

            if self.distance_cells(new_node, goal) <= goal_tolerance_cells and self.is_segment_free(new_node, goal):
                goal_cost = cost[new_node] + self.distance_cells(new_node, goal)
                if goal_cost < best_goal_cost:
                    parent[goal] = new_node
                    cost[goal] = goal_cost
                    best_goal_cost = goal_cost

        if goal not in parent:
            return None

        return self.reconstruct_path(parent, goal)

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            if current is None:
                break
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
    node = RRTStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()