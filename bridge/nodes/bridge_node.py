#!/usr/bin/env python3
"""
Publishes the bridge entrance pose to /bridge/entrance at 1 Hz so it is
visible in Foxglove.  The robot does not move.
"""

import csv
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


_DEFAULT_LANDMARKS_CSV = '/ros2_ws/scan_map/maps/landmarks.csv'


def _load_bridge(csv_path: str):
    abs_path = os.path.realpath(csv_path)
    with open(abs_path, newline='') as f:
        for row in csv.DictReader(f):
            if row['Object_Type'].strip() == 'Bridge':
                return float(row['X']), float(row['Y']), float(row['Yaw_Degrees'])
    raise RuntimeError(f'No Bridge entry found in {abs_path}')


_FIELD_ORIGIN = 2.0  # CSV coords are in [0,4] field space; map frame centres at (0,0)


def compute_entrance(bx: float, by: float, yaw_deg: float, offset_m: float = 0.4, offset_lateral_m: float = -0.1):
    """
    Convert CSV field coordinates to map-frame world coordinates, then compute
    the entrance point on the −yaw side so the robot travels in the +yaw
    direction and keeps the flat left wall on its left.
    """
    # CSV stores positions in pixel-scaled field space (0→4 m); map origin is field centre
    bx -= _FIELD_ORIGIN
    by -= _FIELD_ORIGIN

    yaw_rad = math.radians(yaw_deg)
    entry_yaw = yaw_rad + 0.5 * math.pi
    ex = bx - math.cos(entry_yaw) * (1 + offset_m) + math.cos(yaw_rad) * offset_lateral_m
    ey = by - math.sin(entry_yaw) * (1 + offset_m) + math.sin(yaw_rad) * offset_lateral_m
    return ex, ey, entry_yaw


class BridgeNode(Node):

    def __init__(self):
        super().__init__('bridge_node')

        self.declare_parameter('offset_m',    0.3)
        self.declare_parameter('frame_id',   'map')
        self.declare_parameter('landmarks_csv', _DEFAULT_LANDMARKS_CSV)

        offset_m     = self.get_parameter('offset_m').value
        frame_id     = self.get_parameter('frame_id').value
        landmarks_csv = self.get_parameter('landmarks_csv').value

        bx, by, yaw_deg = _load_bridge(landmarks_csv)
        self.declare_parameter('offset_lateral_m', -0.1)
        offset_lateral_m = self.get_parameter('offset_lateral_m').value

        ex, ey, approach_yaw = compute_entrance(bx, by, yaw_deg, offset_m, offset_lateral_m)
        yaw_rad = math.radians(yaw_deg)
        # Map-frame centre with the same lateral offset as the entrance
        cx = bx - _FIELD_ORIGIN + math.cos(yaw_rad) * offset_lateral_m
        cy = by - _FIELD_ORIGIN + math.sin(yaw_rad) * offset_lateral_m

        self.get_logger().info(
            f'Bridge centre (map)=({cx:.3f}, {cy:.3f})  yaw={yaw_deg:.1f} deg'
        )
        self.get_logger().info(
            f'Entrance (map)=({ex:.3f}, {ey:.3f})  approach_yaw={math.degrees(approach_yaw):.1f} deg'
        )

        q_z = math.sin(approach_yaw / 2.0)
        q_w = math.cos(approach_yaw / 2.0)

        self._msg = PoseStamped()
        self._msg.header.frame_id   = frame_id
        self._msg.pose.position.x   = ex
        self._msg.pose.position.y   = ey
        self._msg.pose.position.z   = 0.0
        self._msg.pose.orientation.z = q_z
        self._msg.pose.orientation.w = q_w

        # Bridge centre pose (used by control_node for ADJUST heading and PASS target)
        self._centre_msg = PoseStamped()
        self._centre_msg.header.frame_id    = frame_id
        self._centre_msg.pose.position.x    = cx
        self._centre_msg.pose.position.y    = cy
        self._centre_msg.pose.position.z    = 0.0
        self._centre_msg.pose.orientation.z = q_z
        self._centre_msg.pose.orientation.w = q_w

        # Exit point: opposite side of bridge, same lateral offset, same approach orientation
        exitx = cx + math.cos(approach_yaw) * (1 + offset_m)
        exity = cy + math.sin(approach_yaw) * (1 + offset_m)
        self.get_logger().info(f'Exit (map)=({exitx:.3f}, {exity:.3f})')
        self._exit_msg = PoseStamped()
        self._exit_msg.header.frame_id    = frame_id
        self._exit_msg.pose.position.x    = exitx
        self._exit_msg.pose.position.y    = exity
        self._exit_msg.pose.position.z    = 0.0
        self._exit_msg.pose.orientation.z = q_z
        self._exit_msg.pose.orientation.w = q_w

        self._pub        = self.create_publisher(PoseStamped, '/bridge/entrance', 10)
        self._centre_pub = self.create_publisher(PoseStamped, '/bridge/centre',   10)
        self._exit_pub   = self.create_publisher(PoseStamped, '/bridge/exit',     10)
        self.create_timer(1.0, self._publish)

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self._msg.header.stamp        = now
        self._centre_msg.header.stamp = now
        self._exit_msg.header.stamp   = now
        self._pub.publish(self._msg)
        self._centre_pub.publish(self._centre_msg)
        self._exit_pub.publish(self._exit_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
