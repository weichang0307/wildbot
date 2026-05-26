#!/usr/bin/env python3
"""
Publishes /door/ready and /door/knob poses at 1 Hz based on door_type parameter.
door_type: 'front' | 'back' | 'left' | 'right'
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# (x, y, z, qx, qy, qz, qw)
_DOOR_POSES = {
    'front': {
        'ready': (1.6, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0),
        'knob':  (2.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0),
    },
    'back': {
        'ready': (-1.6, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0),
        'knob':  (-2.0, 0.0, 0.0,  0.0, 0.0, 0.0, 1.0),
    },
    'left': {
        'ready': (0.0, 1.6, 0.0,  0.0, 0.0, 0.0, 1.0),
        'knob':  (0.0, 2.0, 0.0,  0.0, 0.0, 0.0, 1.0),
    },
    'right': {
        'ready': (0.0, -1.6, 0.0,  0.0, 0.0, 0.0, 1.0),
        'knob':  (0.0, -2.0, 0.0,  0.0, 0.0, 0.0, 1.0),
    },
}


def _make_pose(frame_id: str, x, y, z, qx, qy, qz, qw) -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id      = frame_id
    msg.pose.position.x      = x
    msg.pose.position.y      = y
    msg.pose.position.z      = z
    msg.pose.orientation.x   = qx
    msg.pose.orientation.y   = qy
    msg.pose.orientation.z   = qz
    msg.pose.orientation.w   = qw
    return msg


class DoorNode(Node):

    def __init__(self):
        super().__init__('door_node')

        self.declare_parameter('door_type', 'front')
        self.declare_parameter('frame_id',  'map')

        door_type = self.get_parameter('door_type').value
        frame_id  = self.get_parameter('frame_id').value

        if door_type not in _DOOR_POSES:
            self.get_logger().error(
                f"Unknown door_type '{door_type}'. Valid: {list(_DOOR_POSES)}. Defaulting to 'front'."
            )
            door_type = 'front'

        poses = _DOOR_POSES[door_type]
        self._ready_msg = _make_pose(frame_id, *poses['ready'])
        self._knob_msg  = _make_pose(frame_id, *poses['knob'])

        self._ready_pub = self.create_publisher(PoseStamped, '/door/ready', 10)
        self._knob_pub  = self.create_publisher(PoseStamped, '/door/knob',  10)

        self.create_timer(1.0, self._publish)
        self.get_logger().info(f"DoorNode ready — type='{door_type}' frame='{frame_id}'")

    def _publish(self):
        now = self.get_clock().now().to_msg()
        self._ready_msg.header.stamp = now
        self._knob_msg.header.stamp  = now
        self._ready_pub.publish(self._ready_msg)
        self._knob_pub.publish(self._knob_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DoorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
