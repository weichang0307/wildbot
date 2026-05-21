#!/usr/bin/env python3
"""
Lidar mapper for a rectangular arena.

Per-frame RANSAC wall-line extraction → absolute pose from known arena geometry.
No odometry dependency, no drift accumulation.
"""
import math, os, signal, subprocess, sys, yaml
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster


# ── RANSAC hyperparameters ────────────────────────────────────────────────────

_RANSAC_ITERS = 80
_INLIER_DIST  = 0.03   # m — point-to-line threshold
_MIN_INLIERS  = 20
_MAX_WALLS    = 4
_PLATE_OFFSET = 0.3    # m — robot center from starting corner

# Wall normal directions in map frame (right, top, left, bottom)
_WALL_ANGLES  = [0.0, math.pi/2, math.pi, 3*math.pi/2]


# ── Pure geometry helpers ─────────────────────────────────────────────────────

def _adiff(a, b):
    """Signed angular difference a−b, wrapped to (−π, π]."""
    d = (a - b) % (2*math.pi)
    return d - 2*math.pi if d > math.pi else d


def _fit_line(pts, n_iter, thresh, min_pts):
    """
    RANSAC line fit. Returns ((a,b,c), inlier_mask) where ax+by+c=0
    and (a,b) is a unit normal. Returns (None, None) on failure.
    """
    rng = np.random.default_rng(0)
    best = np.zeros(len(pts), bool)
    for _ in range(n_iter):
        i, j = rng.choice(len(pts), 2, replace=False)
        d  = pts[j] - pts[i]
        nn = np.linalg.norm(d)
        if nn < 1e-6:
            continue
        a, b = -d[1]/nn, d[0]/nn
        c    = -(a*pts[i,0] + b*pts[i,1])
        mask = np.abs(a*pts[:,0] + b*pts[:,1] + c) < thresh
        if mask.sum() > best.sum():
            best = mask
    if best.sum() < min_pts:
        return None, None
    # Refit on all inliers for accuracy
    inp = pts[best]
    p   = inp.mean(0)
    _, _, Vt = np.linalg.svd(inp - p)
    a, b = -Vt[0,1], Vt[0,0]
    nn   = math.hypot(a, b)
    a   /= nn; b /= nn
    c    = -(a*p[0] + b*p[1])
    return (a, b, c), best


def _extract_walls(pts):
    """Iteratively remove dominant lines up to _MAX_WALLS."""
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
    """
    Compute absolute robot pose (rx, ry, theta) from wall lines detected in
    robot frame. Returns None if walls don't align clearly to a rectangle.

    Strategy:
      1. Find robot heading theta by minimising angular residuals when mapping
         each detected wall normal to the nearest cardinal direction.
      2. With theta known, assign each wall to a map-side and read robot coords
         from the point-to-line distance.
    """
    if len(walls) < 2:
        return None

    alphas = [math.atan2(w[1], w[0]) % (2*math.pi) for w in walls]
    dists  = [abs(w[2]) for w in walls]   # distance from robot to each wall

    # Grid search over (reference wall normal, detected wall) pairings for theta
    best_theta, best_score = 0.0, float('inf')
    for ref in _WALL_ANGLES:
        for alpha in alphas:
            theta = _adiff(ref, alpha)
            score = sum(
                min(_adiff((a + theta) % (2*math.pi), n)**2 for n in _WALL_ANGLES)
                for a in alphas
            )
            if score < best_score:
                best_score = score
                best_theta = theta

    if best_score > 0.3:   # walls don't cleanly align to a rectangle
        return None

    # Assign each wall to a map side and accumulate coordinate estimates
    rxs, rys = [], []
    for (a, b, _), d in zip(walls, dists):
        map_angle = (math.atan2(b, a) + best_theta) % (2*math.pi)
        nearest   = min(_WALL_ANGLES, key=lambda n: abs(_adiff(map_angle, n)))
        if abs(_adiff(map_angle, nearest)) > 0.3:
            continue                                      # poor match, skip
        if   abs(_adiff(nearest, 0.0))          < 0.1:  rxs.append(field - d)  # right wall x=field
        elif abs(_adiff(nearest, math.pi))       < 0.1:  rxs.append(d)          # left wall  x=0
        elif abs(_adiff(nearest, math.pi/2))     < 0.1:  rys.append(field - d)  # top wall   y=field
        elif abs(_adiff(nearest, 3*math.pi/2))   < 0.1:  rys.append(d)          # bottom wall y=0

    if not rxs or not rys:
        return None
    return float(np.mean(rxs)), float(np.mean(rys)), best_theta


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper')
        for name, val in [('scan_topic', '/scan'), ('base_frame', 'car_base'),
                          ('map_frame',  'map'),   ('field_size', 4.0),
                          ('resolution', 0.005),   ('margin',     0.1)]:
            self.declare_parameter(name, val)

        self.field      = self.get_parameter('field_size').value
        self.res        = self.get_parameter('resolution').value
        self.margin     = self.get_parameter('margin').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame  = self.get_parameter('map_frame').value

        sz         = int(round((self.field + 2*self.margin) / self.res))
        self.size  = sz
        self.hits    = np.zeros((sz, sz), np.int32)   # occupied ray endpoints
        self.visited = np.zeros((sz, sz), np.int32)   # free ray cells

        self.pose    = None    # (rx, ry, theta) in map frame
        self._first  = True
        self._tf     = TransformBroadcaster(self)

        self.create_subscription(
            LaserScan, self.get_parameter('scan_topic').value, self._on_scan, 10)
        self.pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.create_service(Trigger, '/save_map', self._save_cb)
        self.create_timer(0.5, self._publish_map)

        signal.signal(signal.SIGINT, lambda *_: (self._save(), self._run_processor(), rclpy.shutdown()))
        self.get_logger().info(
            f'LidarMapper ready — {sz}² grid @ {self.res*1e3:.0f} mm/cell')

    # ── scan callback ─────────────────────────────────────────────────────────

    def _on_scan(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r      = np.array(msg.ranges, np.float64)
        ok     = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        pts    = np.column_stack([r[ok]*np.cos(angles[ok]), r[ok]*np.sin(angles[ok])])
        if len(pts) < _MIN_INLIERS:
            return

        result = _estimate_pose(_extract_walls(pts), self.field)

        if result is None:
            if self.pose is None:
                return                  # can't place robot yet
            rx, ry, theta = self.pose  # hold last known pose
        else:
            rx, ry, theta = result
            if self._first:
                # Snap initial position to the nearest expected corner
                p = _PLATE_OFFSET
                corners = [(p, p), (self.field-p, p), (p, self.field-p), (self.field-p, self.field-p)]
                rx, ry  = min(corners, key=lambda c: (c[0]-rx)**2 + (c[1]-ry)**2)
                self._first = False
            self.pose = (rx, ry, theta)

        self._update_grid(pts, rx, ry, theta)
        self._broadcast_tf(rx, ry, theta, msg.header.stamp)

    # ── occupancy grid update ─────────────────────────────────────────────────

    def _update_grid(self, pts, rx, ry, theta):
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        off = self.margin
        mx = cos_t*pts[:,0] - sin_t*pts[:,1] + rx + off
        my = sin_t*pts[:,0] + cos_t*pts[:,1] + ry + off

        gx  = (mx / self.res).astype(int)
        gy  = (my / self.res).astype(int)
        grx = int((rx + off) / self.res)
        gry = int((ry + off) / self.res)

        # Mark occupied endpoints
        valid = (gx >= 0) & (gx < self.size) & (gy >= 0) & (gy < self.size)
        np.add.at(self.hits, (gy[valid], gx[valid]), 1)

        # Raytrace free cells along each ray (Bresenham via linspace)
        for i in np.where(valid)[0]:
            steps = max(abs(int(gx[i]) - grx), abs(int(gy[i]) - gry))
            if steps < 2:
                continue
            xs = np.linspace(grx, int(gx[i]), steps, endpoint=False).astype(int)
            ys = np.linspace(gry, int(gy[i]), steps, endpoint=False).astype(int)
            ok = (xs >= 0) & (xs < self.size) & (ys >= 0) & (ys < self.size)
            np.add.at(self.visited, (ys[ok], xs[ok]), 1)

    # ── TF broadcast ─────────────────────────────────────────────────────────

    def _broadcast_tf(self, rx, ry, theta, stamp):
        t = TransformStamped()
        t.header.stamp      = stamp
        t.header.frame_id   = self.map_frame
        t.child_frame_id    = self.base_frame
        t.transform.translation.x = rx
        t.transform.translation.y = ry
        t.transform.rotation.z    = math.sin(theta/2)
        t.transform.rotation.w    = math.cos(theta/2)
        self._tf.sendTransform(t)

    # ── map publishing ────────────────────────────────────────────────────────

    def _publish_map(self):
        if self.pose is None:
            return
        occ = np.full((self.size, self.size), -1, np.int8)
        occ[self.visited > 0] = 0
        occ[self.hits    > 0] = 100
        msg = OccupancyGrid()
        msg.header.stamp         = self.get_clock().now().to_msg()
        msg.header.frame_id      = self.map_frame
        msg.info.resolution      = self.res
        msg.info.width           = self.size
        msg.info.height          = self.size
        msg.info.origin.position.x  = -self.margin
        msg.info.origin.position.y  = -self.margin
        msg.info.origin.orientation.w = 1.0
        msg.data = occ.flatten().tolist()
        self.pub.publish(msg)

    # ── map saving ────────────────────────────────────────────────────────────

    def _pkg_share(self):
        return get_package_share_directory('scan_map')

    def _save(self):
        maps_dir = os.path.join(self._pkg_share(), 'maps')
        os.makedirs(maps_dir, exist_ok=True)

        # PGM: 0=occupied (black), 255=free (white), 205=unknown (grey)
        pgm = np.full((self.size, self.size), 205, np.uint8)
        pgm[self.visited > 0] = 255
        pgm[self.hits    > 0] = 0
        pgm = np.flipud(pgm)   # row 0 = y_min in ROS convention

        pgm_path = os.path.join(maps_dir, 'raw_map.pgm')
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{self.size} {self.size}\n255\n'.encode())
            f.write(pgm.tobytes())

        with open(os.path.join(maps_dir, 'raw_map.yaml'), 'w') as f:
            yaml.dump({'image': 'raw_map.pgm', 'resolution': self.res,
                       'origin': [-self.margin, -self.margin, 0.0],
                       'occupied_thresh': 0.65, 'free_thresh': 0.25, 'negate': 0}, f)

        self.get_logger().info(f'Map saved → {pgm_path}')

    def _run_processor(self):
        script = os.path.join(self._pkg_share(), 'scripts', 'map_processor.py')
        self.get_logger().info('Running map_processor.py...')
        result = subprocess.run([sys.executable, script])
        if result.returncode != 0:
            self.get_logger().error('map_processor.py failed')

    def _save_cb(self, req, resp):
        self._save()
        resp.success = True
        resp.message = 'saved'
        return resp


def main():
    rclpy.init()
    rclpy.spin(LidarMapper())


if __name__ == '__main__':
    main()
