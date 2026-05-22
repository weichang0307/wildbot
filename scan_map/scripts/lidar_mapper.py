#!/usr/bin/env python3
"""
Lidar mapper for a rectangular arena using Scan-to-Map ICP SLAM.

Features:
- Voxel-downsampled global map for loop closure.
- Kinematic Deadzone filtering to prevent flat-wall jitter.
- Graceful shutdown for guaranteed map saving.
"""
import math, os, subprocess, sys, yaml
import numpy as np
from scipy.spatial import cKDTree
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

# ── Configuration ────────────────────────────────────────────────────────────

_PLATE_OFFSET = 0.3    # m — starting distance from corner
_MAX_ICP_ITER = 25
_ICP_TOL      = 1e-4
_VOXEL_SIZE   = 1.0    # m — map resolution
_MAX_CORR_DST = 0.15   # m — max distance to accept point pair


def _voxel_downsample(points, voxel_size=_VOXEL_SIZE):
    if len(points) == 0:
        return points
    voxels = np.round(points / voxel_size).astype(int)
    _, unique_indices = np.unique(voxels, axis=0, return_index=True)
    return points[unique_indices]


def _icp_2d_scan_to_map(local_pts, global_map, current_pose, max_iters=_MAX_ICP_ITER, tolerance=_ICP_TOL):
    rx, ry, theta = current_pose
    
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    R_pose = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
    t_pose = np.array([rx, ry])
    
    src_curr = (R_pose @ local_pts.T).T + t_pose
    
    total_R = np.eye(2)
    total_t = np.zeros(2)
    tree = cKDTree(global_map)

    for _ in range(max_iters):
        dists, nn_idx = tree.query(src_curr)

        valid = dists < _MAX_CORR_DST
        if np.sum(valid) < 15:
            break 

        src_v = src_curr[valid]
        dst_v = global_map[nn_idx[valid]]

        mu_s = np.mean(src_v, axis=0)
        mu_d = np.mean(dst_v, axis=0)

        src_c = src_v - mu_s
        dst_c = dst_v - mu_d

        H = src_c.T @ dst_c
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T

        if np.linalg.det(R) < 0:
            Vt[1, :] *= -1
            R = Vt.T @ U.T

        t = mu_d - R @ mu_s

        src_curr = (R @ src_curr.T).T + t
        total_R = R @ total_R
        total_t = R @ total_t + t

        if np.mean(np.abs(t)) < tolerance and np.abs(np.arccos(np.clip(total_R[0,0], -1.0, 1.0))) < tolerance:
            break
            
    new_R = total_R @ R_pose
    new_t = total_R @ t_pose + total_t
    
    new_theta = math.atan2(new_R[1, 0], new_R[0, 0])
    new_rx, new_ry = new_t[0], new_t[1]

    # Velocity Clamp (Reject massive physics-breaking jumps)
    dist_jump = math.hypot(new_rx - rx, new_ry - ry)
    angle_jump = abs(math.atan2(math.sin(new_theta - theta), math.cos(new_theta - theta)))
    
    if dist_jump > 0.20 or angle_jump > 0.20:
        return current_pose 
    
    return new_rx, new_ry, new_theta


# ── ROS2 node ─────────────────────────────────────────────────────────────────

class LidarMapper(Node):
    def __init__(self):
        super().__init__('lidar_mapper')
        for name, val in [('scan_topic', '/scan'), ('base_frame', 'car_base'),
                          ('map_frame',  'map'),   ('field_size', 4.0),
                          ('resolution', 0.01),   ('margin',     0.1)]:
            self.declare_parameter(name, val)

        self.field      = self.get_parameter('field_size').value
        self.res        = self.get_parameter('resolution').value
        self.margin     = self.get_parameter('margin').value
        self.base_frame = self.get_parameter('base_frame').value
        self.map_frame  = self.get_parameter('map_frame').value

        sz           = int(round((self.field + 2*self.margin) / self.res))
        self.size    = sz
        self.hits    = np.zeros((sz, sz), np.int32)   
        self.visited = np.zeros((sz, sz), np.int32)   

        self.pose           = (_PLATE_OFFSET, _PLATE_OFFSET, 0.0) 
        self.global_map_pts = None   
        self.keyframe_pose  = self.pose
        
        self._tf = TransformBroadcaster(self)

        self.create_subscription(
            LaserScan,
            self.get_parameter('scan_topic').value,
            self._on_scan,
            qos_profile_sensor_data,
        )
        self.pub = self.create_publisher(OccupancyGrid, '/map', 1)
        self.create_service(Trigger, '/save_map', self._save_cb)
        self.create_timer(0.2, self._publish_map)

        self.get_logger().info(f'LidarMapper ready — {sz}² grid @ {self.res*1e3:.0f} mm/cell')

    # ── scan callback ─────────────────────────────────────────────────────────

    def _on_scan(self, msg):
        angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
        r      = np.array(msg.ranges, np.float64)
        ok     = np.isfinite(r) & (r > msg.range_min) & (r < msg.range_max)
        pts    = np.column_stack([r[ok]*np.cos(angles[ok]), r[ok]*np.sin(angles[ok])])
        
        if len(pts) < 15:
            return

        if self.global_map_pts is None:
            rx, ry, theta = self.pose
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            global_pts = np.copy(pts)
            global_pts[:,0] = cos_t*pts[:,0] - sin_t*pts[:,1] + rx
            global_pts[:,1] = sin_t*pts[:,0] + cos_t*pts[:,1] + ry
            self.global_map_pts = _voxel_downsample(global_pts)
            return

        # 1. Get raw ICP suggestion
        raw_rx, raw_ry, raw_theta = _icp_2d_scan_to_map(pts, self.global_map_pts, self.pose)
        
        # 2. Apply Kinematic Deadzone (Kill micro-jittering)
        old_rx, old_ry, old_theta = self.pose
        
        dx = raw_rx - old_rx
        dy = raw_ry - old_ry
        dtheta = math.atan2(math.sin(raw_theta - old_theta), math.cos(raw_theta - old_theta))
        
        # If movement is < 1cm, zero it out (stop sliding)
        if math.hypot(dx, dy) < 0.01:
            dx, dy = 0.0, 0.0
            
        # If rotation is < 0.5 degrees, zero it out (stop spinning)
        if abs(dtheta) < 0.0087:
            dtheta = 0.0
            
        # 3. Apply Low-Pass Filter (Smooth intentional movement)
        alpha = 0.6  # Trust ICP 60% per frame
        final_rx = old_rx + dx * alpha
        final_ry = old_ry + dy * alpha
        final_theta = old_theta + dtheta * alpha
        
        self.pose = (final_rx, final_ry, final_theta)
        
        # 4. Map Expansion
        kx, ky, ktheta = self.keyframe_pose
        dist_moved = math.hypot(final_rx - kx, final_ry - ky)
        angle_moved = abs(math.atan2(math.sin(final_theta - ktheta), math.cos(final_theta - ktheta)))
        
        if dist_moved > 0.15 or angle_moved > 0.15:
            cos_t, sin_t = math.cos(final_theta), math.sin(final_theta)
            aligned_pts = np.copy(pts)
            aligned_pts[:,0] = cos_t*pts[:,0] - sin_t*pts[:,1] + final_rx
            aligned_pts[:,1] = sin_t*pts[:,0] + cos_t*pts[:,1] + final_ry
            
            combined = np.vstack((self.global_map_pts, aligned_pts))
            self.global_map_pts = _voxel_downsample(combined)
            self.keyframe_pose = self.pose

        self._update_grid(pts, final_rx, final_ry, final_theta)
        self._broadcast_tf(final_rx, final_ry, final_theta, msg.header.stamp)

    # ── grid update & tf ──────────────────────────────────────────────────────
    
    def _update_grid(self, pts, rx, ry, theta):
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        off = self.margin
        mx = cos_t*pts[:,0] - sin_t*pts[:,1] + rx + off
        my = sin_t*pts[:,0] + cos_t*pts[:,1] + ry + off

        gx  = (mx / self.res).astype(int)
        gy  = (my / self.res).astype(int)
        grx = int((rx + off) / self.res)
        gry = int((ry + off) / self.res)

        valid = (gx >= 0) & (gx < self.size) & (gy >= 0) & (gy < self.size)
        np.add.at(self.hits, (gy[valid], gx[valid]), 1)

        for i in np.where(valid)[0]:
            steps = max(abs(int(gx[i]) - grx), abs(int(gy[i]) - gry))
            if steps < 2: continue
            xs = np.linspace(grx, int(gx[i]), steps, endpoint=False).astype(int)
            ys = np.linspace(gry, int(gy[i]), steps, endpoint=False).astype(int)
            ok = (xs >= 0) & (xs < self.size) & (ys >= 0) & (ys < self.size)
            np.add.at(self.visited, (ys[ok], xs[ok]), 1)

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

    def _publish_map(self):
        if self.pose is None: return
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

        pgm = np.full((self.size, self.size), 205, np.uint8)
        pgm[self.visited > 0] = 255
        pgm[self.hits    > 0] = 0
        pgm = np.flipud(pgm)   

        pgm_path = os.path.join(maps_dir, 'raw_map.pgm')
        with open(pgm_path, 'wb') as f:
            f.write(f'P5\n{self.size} {self.size}\n255\n'.encode())
            f.write(pgm.tobytes())

        with open(os.path.join(maps_dir, 'raw_map.yaml'), 'w') as f:
            yaml.dump({'image': 'raw_map.pgm', 'resolution': self.res,
                       'origin': [-self.margin, -self.margin, 0.0],
                       'occupied_thresh': 0.65, 'free_thresh': 0.25, 'negate': 0}, f)

        # Print loudly so it's easy to find in the terminal after Ctrl+C
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info(f'MAP SAVED SUCCESSFULLY TO:')
        self.get_logger().info(f'{pgm_path}')
        self.get_logger().info('='*50 + '\n')

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


def main(args=None):
    rclpy.init(args=args)
    mapper = LidarMapper()
    
    # Graceful Shutdown Block
    try:
        rclpy.spin(mapper)
    except KeyboardInterrupt:
        mapper.get_logger().info('Ctrl+C detected. Shutting down and saving map...')
    finally:
        # Guarantee saving happens before the node is destroyed
        mapper._save()
        mapper._run_processor()
        mapper.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()