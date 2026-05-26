#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose


class AStarPlannerNode(Node):
    """Bridges path_goal → Nav2 ComputePathToPose → astar_path."""

    def __init__(self):
        super().__init__('astar_planner')

        self._action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        self._goal_handle = None
        self._last_goal: PoseStamped | None = None
        self._planning_in_progress = False
        self._consecutive_failures = 0
        self._max_failures = 5

        self.path_pub      = self.create_publisher(Path,        'astar_path',      10)
        self.block_zone_pub = self.create_publisher(PoseStamped, '/nav/block_zone', 10)
        self.create_subscription(PoseStamped, 'path_goal',        self._on_path_goal, 10)
        self.create_timer(1.0, self._replan_tick)

        self.get_logger().info('Waiting for Nav2 compute_path_to_pose action server...')

    def _send_goal(self, msg: PoseStamped):
        if not self._action_client.server_is_ready():
            self.get_logger().warn('Nav2 planner not ready; skipping replan')
            return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None

        self._planning_in_progress = True
        goal = ComputePathToPose.Goal()
        goal.goal = msg
        goal.planner_id = ''
        goal.use_start = False

        self._action_client.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_path_goal(self, msg):
        self._last_goal = msg
        self._consecutive_failures = 0
        self._send_goal(msg)

    def _replan_tick(self):
        if self._last_goal is None or self._planning_in_progress:
            return
        self._send_goal(self._last_goal)

    def _on_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 planner rejected goal')
            self._planning_in_progress = False
            return
        self._goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, future):
        self._goal_handle = None
        self._planning_in_progress = False
        path = future.result().result.path
        if not path.poses:
            self._consecutive_failures += 1
            if self._consecutive_failures >= self._max_failures:
                self.get_logger().warn(
                    f'Planning failed {self._consecutive_failures} times in a row — dropping goal'
                )
                if self._last_goal is not None:
                    self.block_zone_pub.publish(self._last_goal)
                self._last_goal = None
                self._consecutive_failures = 0
            else:
                self.get_logger().warn(
                    f'Nav2 planner returned empty path ({self._consecutive_failures}/{self._max_failures})'
                )
            return
        self._consecutive_failures = 0
        self.path_pub.publish(path)
        self.get_logger().info(f'Published path ({len(path.poses)} poses)')


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
