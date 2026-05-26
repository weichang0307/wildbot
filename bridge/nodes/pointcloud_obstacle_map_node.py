#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class PointCloudObstacleMapNode(Node):

    def __init__(self):
        super().__init__('pointcloud_obstacle_map')

        self.declare_parameter('cloud_topic',  '/camera/depth/points')
        self.declare_parameter('output_topic', '/dynamic_obstacle_map')
        self.declare_parameter('map_frame',    'map')
        self.declare_parameter('resolution',   0.05)   # m/cell
        self.declare_parameter('width',        200)    # cells
        self.declare_parameter('height',       200)    # cells
        self.declare_parameter('origin_x',    -5.0)   # m — map lower-left corner
        self.declare_parameter('origin_y',    -5.0)   # m
        self.declare_parameter('min_z',              0.07)  # m — lower height threshold
        self.declare_parameter('max_z',              0.45)  # m — upper height threshold
        self.declare_parameter('robot_frame',        'car_base')
        self.declare_parameter('min_robot_distance', 0.5)   # m — ignore points this close to the robot

        cloud_topic  = self.get_parameter('cloud_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.map_frame          = self.get_parameter('map_frame').value
        self.resolution         = float(self.get_parameter('resolution').value)
        self.width              = int(self.get_parameter('width').value)
        self.height             = int(self.get_parameter('height').value)
        self.origin_x           = float(self.get_parameter('origin_x').value)
        self.origin_y           = float(self.get_parameter('origin_y').value)
        self.min_z              = float(self.get_parameter('min_z').value)
        self.max_z              = float(self.get_parameter('max_z').value)
        self.robot_frame        = self.get_parameter('robot_frame').value
        self.min_robot_distance = float(self.get_parameter('min_robot_distance').value)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._pass_mode = False

        self.map_pub = self.create_publisher(OccupancyGrid, output_topic, 10)
        self.create_subscription(PointCloud2, cloud_topic, self._on_cloud,  10)
        self.create_subscription(String, '/my_control/state', self._on_state, 10)

        self.get_logger().info(
            f'{cloud_topic} → {output_topic}  '
            f'{self.width}×{self.height} cells @ {self.resolution} m, z > {self.min_z} m'
        )

    def _on_state(self, msg):
        self._pass_mode = msg.data.endswith(':PASS')

    def _on_cloud(self, msg):
        if self._pass_mode:
            grid = OccupancyGrid()
            grid.header.stamp    = self.get_clock().now().to_msg()
            grid.header.frame_id = self.map_frame
            grid.info.resolution = self.resolution
            grid.info.width      = self.width
            grid.info.height     = self.height
            grid.info.origin.position.x    = self.origin_x
            grid.info.origin.position.y    = self.origin_y
            grid.info.origin.orientation.w = 1.0
            grid.data = [0] * (self.width * self.height)
            self.map_pub.publish(grid)
            return

        stamp = msg.header.stamp
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, stamp,
                timeout=rclpy.duration.Duration(seconds=0.3),
            )
        except TransformException as e:
            self.get_logger().warn(f'TF failed: {e}', throttle_duration_sec=2.0)
            return

        # Rotation matrix from quaternion
        qx = tf.transform.rotation.x
        qy = tf.transform.rotation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        R = np.array([
            [1 - 2*(qy**2 + qz**2),   2*(qx*qy - qw*qz),   2*(qx*qz + qw*qy)],
            [  2*(qx*qy + qw*qz), 1 - 2*(qx**2 + qz**2),   2*(qy*qz - qw*qx)],
            [  2*(qx*qz - qw*qy),   2*(qy*qz + qw*qx), 1 - 2*(qx**2 + qy**2)],
        ])
        t = np.array([
            tf.transform.translation.x,
            tf.transform.translation.y,
            tf.transform.translation.z,
        ])

        # Locate x/y/z byte offsets in the PointCloud2 layout
        field_offsets = {f.name: f.offset for f in msg.fields}
        if not all(k in field_offsets for k in ('x', 'y', 'z')):
            self.get_logger().warn('PointCloud2 missing x/y/z fields', throttle_duration_sec=5.0)
            return
        ox = field_offsets['x']
        oy = field_offsets['y']
        oz = field_offsets['z']
        step = msg.point_step
        n    = msg.width * msg.height

        if n == 0:
            return

        # Parse xyz as float32 using numpy — efficient even for 640×480 clouds
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)
        px = raw[:, ox:ox + 4].copy().view(np.float32).ravel()
        py = raw[:, oy:oy + 4].copy().view(np.float32).ravel()
        pz = raw[:, oz:oz + 4].copy().view(np.float32).ravel()

        # Drop NaN/Inf (typical for out-of-range depth pixels)
        valid = np.isfinite(px) & np.isfinite(py) & np.isfinite(pz)
        pts = np.stack([px[valid], py[valid], pz[valid]])  # 3×N

        # Transform to map frame
        world = R @ pts + t[:, None]  # 3×N

        # Get robot position in map frame for proximity filter
        robot_x, robot_y = 0.0, 0.0
        try:
            rt = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, stamp,
                timeout=rclpy.duration.Duration(seconds=0.3),
            )
            robot_x = rt.transform.translation.x
            robot_y = rt.transform.translation.y
        except TransformException:
            pass

        # Keep only points above the height threshold and far enough from the robot
        above = (world[2] > self.min_z) & (world[2] < self.max_z)
        wx = world[0][above]
        wy = world[1][above]
        dist_sq = (wx - robot_x) ** 2 + (wy - robot_y) ** 2
        far_enough = dist_sq >= self.min_robot_distance ** 2
        wx = wx[far_enough]
        wy = wy[far_enough]

        # Project to grid indices
        gx = np.floor((wx - self.origin_x) / self.resolution).astype(np.int32)
        gy = np.floor((wy - self.origin_y) / self.resolution).astype(np.int32)

        in_bounds = (gx >= 0) & (gx < self.width) & (gy >= 0) & (gy < self.height)

        cells = np.zeros(self.width * self.height, dtype=np.int8)
        cells[gy[in_bounds] * self.width + gx[in_bounds]] = 100

        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = self.map_frame
        grid.info.resolution = self.resolution
        grid.info.width      = self.width
        grid.info.height     = self.height
        grid.info.origin.position.x    = self.origin_x
        grid.info.origin.position.y    = self.origin_y
        grid.info.origin.orientation.w = 1.0
        grid.data = cells.tolist()
        self.map_pub.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudObstacleMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
