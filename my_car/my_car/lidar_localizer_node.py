#!/usr/bin/env python3
"""
Lidar localizer — hybrid RANSAC heading + scan-to-map correlation.

Every frame:
  1. RANSAC wall-fit → sub-degree heading (4-fold ambiguous: 0/90/180/270°)
  2. Map correlation search over (x, y) for each of the 4 theta candidates
  3. Best-scoring (theta, x, y) wins — pyramid positions resolve the ambiguity

If RANSAC fails, falls back to a continuous theta range around the previous heading.

Pose logic mirrors lidar_mapper:
  - scan_frame → base_frame static offset resolved via tf2
  - Kinematic deadzone suppresses sub-1cm quantization jitter
  - Velocity clamp rejects physics-breaking jumps
  - Velocity clamp and kinematic deadzone stabilise _pose without lag
"""
import math, os
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Point, Quaternion
from tf2_ros import (TransformBroadcaster, Buffer, TransformListener,
                     LookupException, ConnectivityException, ExtrapolationException)


# ── RANSAC hyperparameters ────────────────────────────────────────────────────

_RANSAC_ITERS = 80
_INLIER_DIST  = 0.03
_MIN_INLIERS  = 20
_MAX_WALLS    = 4
_PLATE_OFFSET = 0.3
_WALL_ANGLES  = [0.0, math.pi / 2, math.pi, 3 * math.pi / 2]


# ── Search parameters ─────────────────────────────────────────────────────────

_SEARCH_XY    = 0.15   # m — (x, y) search radius (tracking)
_STEP_XY      = 0.01   # m — (x, y) step size
_INIT_SEARCH  = 0.5    # m — (x, y) search radius on first frame
_INIT_STEP    = 0.02   # m — (x, y) step size on first frame
_FALLBACK_DA  = 0.08   # rad — theta search radius when RANSAC fails
_FALLBACK_DT  = 0.01   # rad — theta step when RANSAC fails
_MIN_SCORE    = 20     # occupied-cell hits required to accept a pose update
_MAX_SCAN_PTS = 360    # downsample scan to this many points before scoring


# ── Pose filter parameters (mirrors lidar_mapper) ────────────────────────────

_MAX_STEP_TRANS = 0.10    # m  per scan — velocity clamp
_MAX_STEP_ROT   = 0.15    # rad per scan — velocity clamp
_DEADZONE_TRANS = 0.01    # m — micro-jitter floor
_DEADZONE_ROT   = 0.0087  # rad — 0.5°


# ── RANSAC geometry ───────────────────────────────────────────────────────────

def _adiff(a, b):
    d = (a - b) % (2 * math.pi)
    return d - 2 * math.pi if d > math.pi else d


def _fit_line(pts, n_iter, thresh, min_pts):
    rng = np.random.default_rng(0)
    best = np.zeros(len(pts), bool)
    for _ in range(n_iter):
        i, j = rng.choice(len(pts), 2, replace=False)
        d = pts[j] - pts[i]
        nn = np.linalg.norm(d)
        if nn < 1e-6:
            continue
        a, b = -d[1] / nn, d[0] / nn
        c = -(a * pts[i, 0] + b * pts[i, 1])
        mask = np.abs(a * pts[:, 0] + b * pts[:, 1] + c) < thresh
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < min_pts:
        return None, None
    inp = pts[best]
    p = inp.mean(0)
    _, _, Vt = np.linalg.svd(inp - p)
    a, b = -Vt[0, 1], Vt[0, 0]
    nn = math.hypot(a, b)
    a /= nn; b /= nn
    c = -(a * p[0] + b * p[1])
    return (a, b, c), best


def _extract_walls(pts):
    remaining, walls = pts.copy(), []
    for _ in range(_MAX_WALLS):
        if len(remaining) < _MIN_INLIERS:
            break
        line, mask = _fit_line(remaining, _RANSAC_ITERS, _INLIER_DIST, _MIN_INLIERS)
        if line is None:
            break
        walls.append(line)
        remaining = remaining[~mask]
    return walls


def _estimate_pose(walls, field):
    """Returns (rx, ry, theta) or None. theta is 4-fold ambiguous modulo π/2."""
    if len(walls) < 2:
        return None
    alphas = [math.atan2(w[1], w[0]) % (2 * math.pi) for w in walls]
    dists  = [abs(w[2]) for w in walls]

    best_theta, best_score = 0.0, float('inf')
    for ref in _WALL_ANGLES:
        for alpha in alphas:
            theta = _adiff(ref, alpha)
            score = sum(
                min(_adiff((a + theta) % (2 * math.pi), n) ** 2 for n in _WALL_ANGLES)
                for a in alphas
            )
            if score < best_score:
                best_score = score
                best_theta = theta

    if best_score > 0.3:
        return None

    rxs, rys = [], []
    for (a, b, _), d in zip(walls, dists):
        map_angle = (math.atan2(b, a) + best_theta) % (2 * math.pi)
        nearest   = min(_WALL_ANGLES, key=lambda n: abs(_adiff(map_angle, n)))
        if abs(_adiff(map_angle, nearest)) > 0.3:
            continue
        if   abs(_adiff(nearest, 0.0))             < 0.1: rxs.append(field - d)
        elif abs(_adiff(nearest, math.pi))          < 0.1: rxs.append(d)
        elif abs(_adiff(nearest, math.pi / 2))     < 0.1: rys.append(field - d)
        elif abs(_adiff(nearest, 3 * math.pi / 2)) < 0.1: rys.append(d)

    if not rxs or not rys:
        return None
    return float(np.mean(rxs)), float(np.mean(rys)), best_theta


# ── PGM loader ────────────────────────────────────────────────────────────────

def _load_pgm(path):
    with open(path, 'rb') as f:
        def _next():
            line = f.readline()
            while line.startswith(b'#'):
                line = f.readline()
            return line.strip()
        if _next() != b'P5':
            raise ValueError(f'Not a binary PGM: {path}')
        w, h = map(int, _next().split())
        int(_next())
        data = np.frombuffer(f.read(), dtype=np.uint8)
    return data.reshape(h, w), w, h


# ── Scan-to-map correlation search ───────────────────────────────────────────

def _local_search(scan_pts, px, py, theta_list, occ_map, pixels, resolution,
                  search_xy=_SEARCH_XY, step_xy=_STEP_XY):
    """
    For each theta in theta_list, search (x, y) in a grid around (px, py).
    Score = number of scan endpoints landing on occupied cells (pixel < 128).
    Returns (best_pose, best_score).

    The (x, y) grid is fully vectorized via numpy broadcasting; theta_list
    is a small Python loop (4 for RANSAC mode, ~17 for fallback).
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

        # Pixel indices — broadcast to (X, 1, N) and (1, Y, N) → (X, Y, N)
        col = (mx[:, np.newaxis, :] / resolution).astype(np.int32)
        row = (pixels - my[np.newaxis, :, :] / resolution).astype(np.int32)

        in_bounds = (col >= 0) & (col < pixels) & (row >= 0) & (row < pixels)
        scores = np.sum(
            (occ_map[np.clip(row, 0, pixels - 1), np.clip(col, 0, pixels - 1)] < 128)
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

        self._occ, w, h = _load_pgm(map_path)
        self._pixels    = h
        self._res       = self.field / self._pixels
        self.get_logger().info(
            f'Map loaded: {map_path} ({w}×{h} px, {self._res * 1e3:.1f} mm/cell)')

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/pose', 10)
        self.get_logger().info('LidarLocalizer ready')

    def _source_pkg_dir(self):
        share = get_package_share_directory('scan_map')
        ws_root = os.path.abspath(os.path.join(share, '..', '..', '..', '..'))
        src = os.path.join(ws_root, 'scan_map')
        if os.path.isdir(os.path.join(src, 'maps')) or \
           os.path.isfile(os.path.join(src, 'package.xml')):
            return src
        return share

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

    def _on_scan(self, msg):
        if self.scan_to_base is None:
            if not self._resolve_scan_to_base(msg.header.frame_id):
                return

        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r      = np.array(msg.ranges, np.float64)
        ok     = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        lpts   = np.column_stack([r[ok] * np.cos(angles[ok]),
                                  r[ok] * np.sin(angles[ok])])
        if len(lpts) < _MIN_INLIERS:
            return

        # Transform scan_frame → base_frame (static lidar mount offset)
        sx, sy, syaw = self.scan_to_base
        cs, ss = math.cos(syaw), math.sin(syaw)
        pts = np.empty_like(lpts)
        pts[:, 0] = cs * lpts[:, 0] - ss * lpts[:, 1] + sx
        pts[:, 1] = ss * lpts[:, 0] + cs * lpts[:, 1] + sy

        ransac = _estimate_pose(_extract_walls(pts), self.field)

        if self._pose is None:
            if ransac is None:
                return   # can't init without a heading estimate
            rx, ry, theta = ransac
            p       = _PLATE_OFFSET
            corners = [(p, p), (self.field - p, p),
                       (p, self.field - p), (self.field - p, self.field - p)]
            rx, ry  = min(corners, key=lambda c: (c[0] - rx) ** 2 + (c[1] - ry) ** 2)
            theta4  = [theta + k * math.pi / 2 for k in range(4)]
            pose, _ = _local_search(pts, rx, ry, theta4, self._occ,
                                    self._pixels, self._res,
                                    search_xy=_INIT_SEARCH, step_xy=_INIT_STEP)
            self._pose = pose
            rx, ry, theta = pose
            self.get_logger().info(
                f'Initialised at ({rx:.2f}, {ry:.2f})  θ={math.degrees(theta):.1f}°')
            self._publish(rx, ry, theta, msg.header.stamp)
            self._broadcast_tf(rx, ry, theta, msg.header.stamp)
            return

        px, py, ptheta = self._pose
        if ransac is not None:
            theta4 = [ransac[2] + k * math.pi / 2 for k in range(4)]
        else:
            # Fallback: continuous theta range around previous heading
            theta4 = np.arange(ptheta - _FALLBACK_DA,
                                ptheta + _FALLBACK_DA + _FALLBACK_DT * 0.5,
                                _FALLBACK_DT).tolist()

        pose, score = _local_search(pts, px, py, theta4,
                                    self._occ, self._pixels, self._res)

        if score >= _MIN_SCORE:
            new_rx, new_ry, new_theta = pose
            old_rx, old_ry, old_theta = self._pose

            # Velocity clamp — reject physics-breaking jumps
            dist_jump  = math.hypot(new_rx - old_rx, new_ry - old_ry)
            angle_jump = abs(math.atan2(math.sin(new_theta - old_theta),
                                        math.cos(new_theta - old_theta)))

            if dist_jump <= _MAX_STEP_TRANS and angle_jump <= _MAX_STEP_ROT:
                # Kinematic deadzone — suppress grid-step quantization jitter
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
        self._publish(rx, ry, theta, msg.header.stamp)
        self._broadcast_tf(rx, ry, theta, msg.header.stamp)

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
