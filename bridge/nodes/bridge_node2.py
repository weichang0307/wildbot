#!/usr/bin/env python3
"""
Like bridge_node.py but picks whichever side of the bridge is closer to the
robot's current position as the entrance, instead of a fixed start_side.
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from geometry_msgs.msg import PoseStamped


_DEFAULT_LANDMARKS_CSV = '/ros2_ws/scan_map/maps/landmarks.csv'
_FIELD_ORIGIN = 2.0
_ROBOT_FRAME  = 'car_base'
_MAP_FRAME    = 'map'


def _load_bridge(csv_path: str):
    abs_path = os.path.realpath(csv_path)
    with open(abs_path, newline='') as f:
        for row in csv.DictReader(f):
            if row['Object_Type'].strip() == 'Bridge':
                return float(row['X']), float(row['Y']), float(row['Yaw_Degrees'])
    raise RuntimeError(f'No Bridge entry found in {abs_path}')


def _compute_side(cx, cy, entry_yaw, offset_m):
    """Return (entrance_x, entrance_y, exit_x, exit_y) for one approach direction."""
    ex    = cx - math.cos(entry_yaw) * (1 + offset_m)
    ey    = cy - math.sin(entry_yaw) * (1 + offset_m)
    exitx = cx + math.cos(entry_yaw) * (1 + offset_m)
    exity = cy + math.sin(entry_yaw) * (1 + offset_m)
    return ex, ey, exitx, exity


class BridgeNode2(Node):

    def __init__(self):
        super().__init__('bridge_node2')

        self.declare_parameter('offset_m',         0.3)
        self.declare_parameter('offset_lateral_m', -0.1)
        self.declare_parameter('frame_id',         'map')
        self.declare_parameter('landmarks_csv',    _DEFAULT_LANDMARKS_CSV)

        offset_m         = self.get_parameter('offset_m').value
        offset_lateral_m = self.get_parameter('offset_lateral_m').value
        frame_id         = self.get_parameter('frame_id').value
        landmarks_csv    = self.get_parameter('landmarks_csv').value

        bx, by, yaw_deg = _load_bridge(landmarks_csv)
        bx -= _FIELD_ORIGIN
        by -= _FIELD_ORIGIN
        yaw_rad = math.radians(yaw_deg)

        # Bridge centre (same regardless of approach direction)
        self._cx = bx + math.cos(yaw_rad) * offset_lateral_m
        self._cy = by + math.sin(yaw_rad) * offset_lateral_m

        # Two possible approach directions
        entry_yaw_a = yaw_rad + 0.5 * math.pi
        entry_yaw_b = yaw_rad - 0.5 * math.pi

        self._side_a = (*_compute_side(self._cx, self._cy, entry_yaw_a, offset_m), entry_yaw_a)
        self._side_b = (*_compute_side(self._cx, self._cy, entry_yaw_b, offset_m), entry_yaw_b)

        self._frame_id = frame_id
        self._selected = False

        self._entrance_msg: PoseStamped | None = None
        self._centre_msg:   PoseStamped | None = None
        self._exit_msg:     PoseStamped | None = None

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._pub        = self.create_publisher(PoseStamped, '/bridge/entrance', 10)
        self._centre_pub = self.create_publisher(PoseStamped, '/bridge/centre',   10)
        self._exit_pub   = self.create_publisher(PoseStamped, '/bridge/exit',     10)

        self.create_timer(1.0, self._publish)

    # ------------------------------------------------------------------

    def _robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(_MAP_FRAME, _ROBOT_FRAME, rclpy.time.Time())
            return tf.transform.translation.x, tf.transform.translation.y
        except TransformException:
            return None

    def _make_pose(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.frame_id    = self._frame_id
        msg.pose.position.x    = x
        msg.pose.position.y    = y
        msg.pose.position.z    = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    def _select_side(self, rx, ry):
        ex_a, ey_a, exitx_a, exity_a, eyaw_a = self._side_a
        ex_b, ey_b, exitx_b, exity_b, eyaw_b = self._side_b

        if math.hypot(ex_a - rx, ey_a - ry) <= math.hypot(ex_b - rx, ey_b - ry):
            ex, ey, exitx, exity, eyaw = self._side_a
            label = 'A'
        else:
            ex, ey, exitx, exity, eyaw = self._side_b
            label = 'B'

        self.get_logger().info(
            f'Selected side {label}  '
            f'entrance=({ex:.3f},{ey:.3f})  '
            f'centre=({self._cx:.3f},{self._cy:.3f})  '
            f'exit=({exitx:.3f},{exity:.3f})'
        )

        self._entrance_msg = self._make_pose(ex,    ey,           eyaw)
        self._centre_msg   = self._make_pose(self._cx, self._cy,  eyaw)
        self._exit_msg     = self._make_pose(exitx, exity,        eyaw)
        self._selected     = True

    def _publish(self):
        if not self._selected:
            robot = self._robot_xy()
            if robot is None:
                self.get_logger().warn('Waiting for robot TF to select bridge side...')
                return
            self._select_side(*robot)

        now = self.get_clock().now().to_msg()
        self._entrance_msg.header.stamp = now
        self._centre_msg.header.stamp   = now
        self._exit_msg.header.stamp     = now
        self._pub.publish(self._entrance_msg)
        self._centre_pub.publish(self._centre_msg)
        self._exit_pub.publish(self._exit_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode2()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
