#!/usr/bin/env python3
"""
Lidar localizer — pure scan-to-map correlation, no RANSAC.

Every frame:
  1. Rotate scan pts by each theta candidate
  2. Count how many land on occupied cells in lidar_map.pgm (walls + pyramid outlines)
  3. Best (theta, x, y) wins

Initialization:
  - Try 4 cardinal headings × 4 corner seeds with wide (x, y) search
  - The asymmetric pyramid positions in lidar_map.pgm disambiguate the 4 headings

Tracking:
  - ±10° theta window at 1° steps, ±0.15 m (x, y) at 1 cm steps

Pose filter mirrors lidar_mapper:
  - Velocity clamp rejects physics-breaking jumps
  - Kinematic deadzone suppresses sub-1 cm quantization jitter
"""
import math, os, yaml
import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Point, Quaternion
from tf2_ros import (TransformBroadcaster, Buffer, TransformListener,
                     LookupException, ConnectivityException, ExtrapolationException)


# ── Search parameters ─────────────────────────────────────────────────────────

_SEARCH_XY    = 0.15   # m — (x, y) search radius (tracking)
_STEP_XY      = 0.01   # m — (x, y) step size
_INIT_SEARCH  = 0.5    # m — (x, y) search radius on first frame
_INIT_STEP    = 0.02   # m — (x, y) step size on first frame
_TRACK_DA     = 0.175  # rad ≈ 10° — theta search radius (tracking)
_TRACK_DT     = 0.0175 # rad ≈ 1°  — theta step (tracking)
_PLATE_OFFSET = 0.3    # m — corner seed offset from arena edge
_MIN_SCORE    = 20     # occupied-cell hits required to accept a pose update
_MAX_SCAN_PTS = 360    # downsample scan to this many points before scoring

# Dilation radius (px) applied to the score map so wall inner faces score.
# The lidar_map wall material is 10 px outside the arena boundary; a 13-px
# dilation closes that gap plus adds 3 px margin for lidar noise.
_DILATE_PX = 13


# ── Pose filter parameters (mirrors lidar_mapper) ────────────────────────────

_MAX_STEP_TRANS = 0.10    # m  per scan — velocity clamp
_MAX_STEP_ROT   = 0.15    # rad per scan — velocity clamp
_DEADZONE_TRANS = 0.01    # m — micro-jitter floor
_DEADZONE_ROT   = 0.0087  # rad — 0.5°


# ── Map loader ────────────────────────────────────────────────────────────────

def _load_map(pgm_path):
    """Load PGM + companion YAML.

    Returns (occ, score_map, w, h, res, origin_x, origin_y) where:
      occ       — raw uint8 pixel array (0=occupied, 255=free), shape (H, W)
      score_map — dilated binary uint8 (255=occupied for correlation), shape (H, W)
      res       — metres per pixel (from YAML)
      origin_x/y — world coordinates of pixel (0, H-1) bottom-left corner (from YAML)
    """
    with open(pgm_path, 'rb') as f:
        def _next():
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            return line.strip()
        if _next() != b'P5':
            raise ValueError(f'Not a binary PGM: {pgm_path}')
        w, h = map(int, _next().split())
        int(_next())
        data = np.frombuffer(f.read(), dtype=np.uint8)
    occ = data.reshape(h, w)

    yaml_path = pgm_path.replace('.pgm', '.yaml')
    with open(yaml_path) as f:
        meta = yaml.safe_load(f)
    res = float(meta['resolution'])
    ox  = float(meta['origin'][0])
    oy  = float(meta['origin'][1])

    # Dilate occupied pixels into the arena interior so that scan returns at
    # the wall inner face (world x=0, y=0, x=4, y=4) score correctly.
    occ_bin   = (occ < 128).astype(np.uint8) * 255
    k         = 2 * _DILATE_PX + 1
    score_map = cv2.dilate(occ_bin, np.ones((k, k), np.uint8))

    return occ, score_map, w, h, res, ox, oy


# ── Scan-to-map correlation search ───────────────────────────────────────────

def _local_search(scan_pts, px, py, theta_list, score_map, pixels, resolution,
                  origin_x, origin_y,
                  search_xy=_SEARCH_XY, step_xy=_STEP_XY):
    """
    For each theta in theta_list, search (x, y) in a grid around (px, py).
    Score = number of scan endpoints landing on occupied cells of score_map.
    Returns (best_pose, best_score).

    The (x, y) grid is fully vectorised via numpy broadcasting; theta_list
    is a small Python loop (4 for init mode, ~21 for tracking).

    Pixel formula (ROS-standard, matches lidar_map.yaml metadata):
      col = (world_x - origin_x) / resolution
      row = (H - 1) - (world_y - origin_y) / resolution
    """
    if len(scan_pts) > _MAX_SCAN_PTS:
        idx = np.round(np.linspace(0, len(scan_pts) - 1, _MAX_SCAN_PTS)).astype(int)
        scan_pts = scan_pts[idx]

    sx = scan_pts[:, 0]
    sy = scan_pts[:, 1]

    dx = np.arange(-search_xy, search_xy + step_xy * 0.5, step_xy)
    dy = np.arange(-search_xy, search_xy + step_xy * 0.5, step_xy)

    best_score = -1
    best_pose  = (px, py, theta_list[0])

    for theta in theta_list:
        c, s = math.cos(theta), math.sin(theta)

        # Rotate scan to map frame (no translation yet) — (N,)
        rx = c * sx - s * sy
        ry = s * sx + c * sy

        # (X, N): map x for each dx candidate and each scan point
        mx = (px + dx)[:, np.newaxis] + rx[np.newaxis, :]
        # (Y, N): map y for each dy candidate and each scan point
        my = (py + dy)[:, np.newaxis] + ry[np.newaxis, :]

        # Pixel indices using correct ROS-standard formula
        # broadcast to (X, 1, N) and (1, Y, N) → (X, Y, N)
        col = ((mx[:, np.newaxis, :] - origin_x) / resolution).astype(np.int32)
        row = ((pixels - 1) - (my[np.newaxis, :, :] - origin_y) / resolution).astype(np.int32)

        in_bounds = (col >= 0) & (col < pixels) & (row >= 0) & (row < pixels)
        scores = np.sum(
            (score_map[np.clip(row, 0, pixels - 1), np.clip(col, 0, pixels - 1)] > 0)
            & in_bounds,
            axis=2)   # (X, Y)

        xi, yi = np.unravel_index(np.argmax(scores), scores.shape)
        if scores[xi, yi] > best_score:
            best_score = int(scores[xi, yi])
            best_pose  = (float(px + dx[xi]), float(py + dy[yi]), float(theta))

    return best_pose, best_score


# ── ROS2 node ─────────────────────────────────────────────────────────────────

_COV_XY  = 0.005
_COV_YAW = 0.02


class LidarLocalizer(Node):
    def __init__(self):
        super().__init__('lidar_localizer')
        for name, val in [('scan_topic', '/scan'),
                          ('base_frame', 'car_base'),
                          ('map_frame',  'map'),
                          ('field_size', 4.0),
                          ('map_path',   '')]:
            self.declare_parameter(name, val)

        self.field      = self.get_parameter('field_size').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame  = self.get_parameter('map_frame').value

        self._pose = None
        self._tf   = TransformBroadcaster(self)

        # scan_frame → base_frame static offset (resolved on first scan)
        self.scan_to_base = None
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_path = self.get_parameter('map_path').value
        if not map_path:
            map_path = os.path.join(self._source_pkg_dir(), 'maps', 'lidar_map.pgm')

        self._occ, self._score_map, w, h, self._res, self._ox, self._oy = \
            _load_map(map_path)
        self._pixels = h
        self.get_logger().info(
            f'Map loaded: {map_path} ({w}×{h} px, res={self._res*1e3:.1f} mm, '
            f'origin=({self._ox},{self._oy}))')

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/pose', 10)

        # Publish the reference map as OccupancyGrid for Foxglove visualisation
        self._map_pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self._map_msg = self._make_map_msg()
        self.create_timer(1.0, self._pub_map)

        self.get_logger().info('LidarLocalizer ready')

    def _source_pkg_dir(self):
        share = get_package_share_directory('scan_map')
        ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
        src = os.path.join(ws_root, 'scan_map')
        if os.path.isdir(os.path.join(src, 'maps')) or \
           os.path.isfile(os.path.join(src, 'package.xml')):
            return src
        return share

    # ── Map publisher ─────────────────────────────────────────────────────────

    def _make_map_msg(self):
        """Convert lidar_map.pgm to a ROS OccupancyGrid message.

        lidar_map PGM: row 0 = top of image (high world y), pixel 0=occupied.
        OccupancyGrid: row 0 = bottom (low world y), 100=occupied, 0=free.
        → flipud before filling data.
        """
        flipped = np.flipud(self._occ)
        data = np.where(flipped < 128, 100, 0).astype(np.int8)
        msg = OccupancyGrid()
        msg.header.frame_id             = self.map_frame
        msg.info.resolution             = self._res
        msg.info.width                  = self._occ.shape[1]
        msg.info.height                 = self._occ.shape[0]
        msg.info.origin.position.x      = self._ox
        msg.info.origin.position.y      = self._oy
        msg.info.origin.orientation.w   = 1.0
        msg.data = data.flatten().tolist()
        return msg

    def _pub_map(self):
        self._map_msg.header.stamp = self.get_clock().now().to_msg()
        self._map_pub.publish(self._map_msg)

    # ── TF resolver ───────────────────────────────────────────────────────────

    def _resolve_scan_to_base(self, scan_frame):
        """Look up static transform scan_frame → base_frame. Returns False if not ready."""
        if scan_frame == self.base_frame:
            self.scan_to_base = (0.0, 0.0, 0.0)
            return True
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame, scan_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return False
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.scan_to_base = (t.x, t.y, yaw)
        self.get_logger().info(
            f'TF {scan_frame} → {self.base_frame}: '
            f'dx={t.x:.3f} dy={t.y:.3f} dyaw={yaw:.3f}')
        return True

    # ── Scan callback ─────────────────────────────────────────────────────────

    def _on_scan(self, msg):
        if self.scan_to_base is None:
            if not self._resolve_scan_to_base(msg.header.frame_id):
                return

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r      = np.array(msg.ranges, np.float64)
        ok     = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        lpts   = np.column_stack([r[ok] * np.cos(angles[ok]),
                                  r[ok] * np.sin(angles[ok])])
        if len(lpts) < 20:
            return

        # Transform scan_frame → base_frame (static lidar mount offset)
        sx, sy, syaw = self.scan_to_base
        cs, ss = math.cos(syaw), math.sin(syaw)
        pts = np.empty_like(lpts)
        pts[:, 0] = cs * lpts[:, 0] - ss * lpts[:, 1] + sx
        pts[:, 1] = ss * lpts[:, 0] + cs * lpts[:, 1] + sy

        if self._pose is None:
            self._init_pose(pts, msg.header.stamp)
            return

        self._track_pose(pts, msg.header.stamp)

    # ── Pose search ───────────────────────────────────────────────────────────

    def _search(self, pts, px, py, theta_list, search_xy, step_xy):
        return _local_search(pts, px, py, theta_list,
                             self._score_map, self._pixels, self._res,
                             self._ox, self._oy,
                             search_xy=search_xy, step_xy=step_xy)

    def _init_pose(self, pts, stamp):
        """Try 4 cardinal headings × 4 corner seeds to find initial pose."""
        p = _PLATE_OFFSET
        f = self.field
        seeds  = [(p, p), (f - p, p), (p, f - p), (f - p, f - p)]
        theta4 = [k * math.pi / 2 for k in range(4)]

        best_score, best_pose = -1, None
        for seed in seeds:
            pose, score = self._search(pts, seed[0], seed[1], theta4,
                                       _INIT_SEARCH, _INIT_STEP)
            if score > best_score:
                best_score, best_pose = score, pose

        self._pose = best_pose
        rx, ry, theta = best_pose
        self.get_logger().info(
            f'Initialised at ({rx:.2f}, {ry:.2f})  θ={math.degrees(theta):.1f}°  '
            f'score={best_score}')
        self._publish(rx, ry, theta, stamp)
        self._broadcast_tf(rx, ry, theta, stamp)

    def _track_pose(self, pts, stamp):
        """Narrow theta + xy search around current pose."""
        px, py, ptheta = self._pose
        theta_candidates = np.arange(
            ptheta - _TRACK_DA,
            ptheta + _TRACK_DA + _TRACK_DT * 0.5,
            _TRACK_DT).tolist()

        pose, score = self._search(pts, px, py, theta_candidates,
                                   _SEARCH_XY, _STEP_XY)

        if score >= _MIN_SCORE:
            new_rx, new_ry, new_theta = pose
            old_rx, old_ry, old_theta = self._pose

            dist_jump  = math.hypot(new_rx - old_rx, new_ry - old_ry)
            angle_jump = abs(math.atan2(math.sin(new_theta - old_theta),
                                        math.cos(new_theta - old_theta)))

            if dist_jump <= _MAX_STEP_TRANS and angle_jump <= _MAX_STEP_ROT:
                dx     = new_rx - old_rx
                dy     = new_ry - old_ry
                dtheta = math.atan2(math.sin(new_theta - old_theta),
                                    math.cos(new_theta - old_theta))
                if math.hypot(dx, dy) < _DEADZONE_TRANS:
                    dx, dy = 0.0, 0.0
                if abs(dtheta) < _DEADZONE_ROT:
                    dtheta = 0.0
                self._pose = (old_rx + dx, old_ry + dy, old_theta + dtheta)

        rx, ry, theta = self._pose
        self._publish(rx, ry, theta, stamp)
        self._broadcast_tf(rx, ry, theta, stamp)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish(self, rx, ry, theta, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position    = Point(x=rx, y=ry, z=0.0)
        msg.pose.pose.orientation = Quaternion(
            z=math.sin(theta / 2), w=math.cos(theta / 2))
        cov = [0.0] * 36
        cov[0]  = _COV_XY
        cov[7]  = _COV_XY
        cov[35] = _COV_YAW
        msg.pose.covariance = cov
        self.pub.publish(msg)

    def _broadcast_tf(self, rx, ry, theta, stamp):
        t = TransformStamped()
        t.header.stamp      = stamp
        t.header.frame_id   = self.map_frame
        t.child_frame_id    = self.base_frame
        t.transform.translation.x = rx
        t.transform.translation.y = ry
        t.transform.rotation.z    = math.sin(theta / 2)
        t.transform.rotation.w    = math.cos(theta / 2)
        self._tf.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(LidarLocalizer())


if __name__ == '__main__':
    main()
