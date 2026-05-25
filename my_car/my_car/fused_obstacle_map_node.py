#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException


class FusedObstacleMapNode(Node):

    def __init__(self):
        super().__init__('fused_obstacle_map')

        self.declare_parameter('static_topic',    '/obstacle_map')
        self.declare_parameter('dynamic_topic',   '/dynamic_obstacle_map')
        self.declare_parameter('output_topic',    '/fused_obstacle_map')
        self.declare_parameter('robot_frame',     'car_base')
        self.declare_parameter('map_frame',       'map')
        self.declare_parameter('camera_fov_deg',   60.0)  # full horizontal FOV
        self.declare_parameter('camera_min_range',  0.3)  # m — near clipping
        self.declare_parameter('camera_max_range',  1.0)  # m — far clipping
        self.declare_parameter('score_increment', 3.0)    # added when cell seen occupied
        self.declare_parameter('score_decrement', 1.0)    # subtracted when cell seen free in FOV
        self.declare_parameter('score_threshold', 2.0)    # score >= threshold → occupied
        self.declare_parameter('score_max',         15.0)  # clamp ceiling
        self.declare_parameter('explore_resolution',   0.4)    # m/cell for explore map
        self.declare_parameter('robot_explore_radius', 0.5)   # m — proximity explore radius
        self.declare_parameter('start_side',           'right')  # 'right'→zone1(2,-2) skipped, 'left'→zone0(-2,-2) skipped

        static_topic         = self.get_parameter('static_topic').value
        dynamic_topic        = self.get_parameter('dynamic_topic').value
        output_topic         = self.get_parameter('output_topic').value
        self.robot_frame     = self.get_parameter('robot_frame').value
        self.map_frame       = self.get_parameter('map_frame').value
        self.camera_fov_deg   = float(self.get_parameter('camera_fov_deg').value)
        self.camera_min_range = float(self.get_parameter('camera_min_range').value)
        self.camera_max_range = float(self.get_parameter('camera_max_range').value)
        self.score_increment = float(self.get_parameter('score_increment').value)
        self.score_decrement = float(self.get_parameter('score_decrement').value)
        self.score_threshold = float(self.get_parameter('score_threshold').value)
        self.score_max        = float(self.get_parameter('score_max').value)
        self.explore_res          = float(self.get_parameter('explore_resolution').value)
        self.robot_explore_radius = float(self.get_parameter('robot_explore_radius').value)
        start_side = self.get_parameter('start_side').value
        # Index of the zone to SKIP (same side as start) — zone 0 is left (-2,-2), zone 1 is right (2,-2)
        self._skip_zone_idx = 1 if start_side == 'right' else 0

        self.static_map = None
        self._score: np.ndarray | None = None   # float32 at static map resolution
        self._score_shape = 0

        # Explore map — maintained at explore_resolution (coarser)
        self._explore: np.ndarray | None = None  # bool
        self._exp_w = 0
        self._exp_h = 0
        self._exp_ox = 0.0
        self._exp_oy = 0.0
        self._exp_static_key = (0, 0, 0.0)  # (width, height, resolution) of last static map

        # Per-zone flag: True once the robot has left that zone for the first time
        self._zone_exited = [False] * len(self._EXCLUSION_ZONES)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.create_subscription(OccupancyGrid, static_topic,  self._on_static,  10)
        self.create_subscription(OccupancyGrid, dynamic_topic, self._on_dynamic, 10)
        self.fused_pub   = self.create_publisher(OccupancyGrid, output_topic,    10)
        self.explore_pub = self.create_publisher(OccupancyGrid, '/explore_map',  10)

        self.get_logger().info(
            f'Fusing {static_topic} + {dynamic_topic} → {output_topic}'
        )

    # ------------------------------------------------------------------

    def _on_static(self, msg):
        self.static_map = msg
        n = msg.info.width * msg.info.height
        if n != self._score_shape:
            self._score = np.zeros(n, dtype=np.float32)
            self._score_shape = n

        self._rebuild_explore_grid(msg)

        # Mark static walls as explored in the coarse explore grid
        exp_static = self._resample_to_explore(msg)
        self._explore[exp_static == 100] = True

        self._publish_fused()

    def _on_dynamic(self, msg):
        # self.get_logger().info('Received dynamic obstacle map update')
        if self.static_map is None:
            return

        ref = self.static_map
        rw, rh = ref.info.width, ref.info.height
        n = rw * rh

        if self._score is None or self._score_shape != n:
            self._score = np.zeros(n, dtype=np.float32)
            self._score_shape = n

        # Align dynamic map onto static grid
        dyn = msg
        if (dyn.info.width  == rw and
            dyn.info.height == rh and
            abs(dyn.info.resolution - ref.info.resolution) < 1e-6):
            dynamic_arr = np.array(dyn.data, dtype=np.int8)
        else:
            dynamic_arr = self._resample(dyn, ref)

        occupied = dynamic_arr == 100

        # Compute camera FOV mask for the static grid
        fov_mask = self._camera_fov_mask(ref)

        # Increase score where observed occupied
        self._score[occupied] += self.score_increment
        np.clip(self._score, 0.0, self.score_max, out=self._score)

        # Mark cells close to the robot as explored
        prox = self._robot_proximity_mask_explore()
        if prox is not None:
            self._explore[prox] = True

        # Decrease score where observed free AND inside camera FOV; mark FOV as explored
        if fov_mask is not None:
            exp_fov = self._camera_fov_mask_explore()
            if exp_fov is not None:
                self._explore[exp_fov] = True
            free_in_fov = (~occupied) & fov_mask
            self._score[free_in_fov] -= self.score_decrement
            np.clip(self._score, 0.0, self.score_max, out=self._score)
        else:
            self.get_logger().warn('Camera FOV mask unavailable, skipping free-space score decrement')
        self._publish_fused()

    def _publish_fused(self):
        # self.get_logger().info('Publishing fused obstacle map')

        if self.static_map is None or self._score is None:
            return

        ref = self.static_map
        static_arr = np.array(ref.data, dtype=np.int8)

        dynamic_bool = self._score >= self.score_threshold

        fused = static_arr.copy()
        fused[(static_arr == 100) | dynamic_bool] = 100

        # Clear the outermost cells so the inflation layer never reads past the map edge
        rw = ref.info.width
        rh = ref.info.height
        fused2d = fused.reshape(rh, rw)

        # Paint bear exclusion zones before border clearing so they never sit on the edge
        self._paint_exclusion_zones(fused2d, ref)

        # Clear the outermost cells so the inflation layer never reads past the map edge
        fused2d[0, :]  = 0
        fused2d[-1, :] = 0
        fused2d[:, 0]  = 0
        fused2d[:, -1] = 0

        fused = fused2d.ravel()

        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = ref.header.frame_id
        grid.info            = ref.info
        grid.data            = fused.tolist()
        self.fused_pub.publish(grid)
        self._maybe_reset_explore()
        self._publish_explore(ref)

    def _maybe_reset_explore(self):
        if self._explore is None or self._explore.size == 0:
            return
        if self._explore.sum() / self._explore.size > 0.7:
            self.get_logger().info('Explore map >70% covered — resetting to unexplored')
            self._explore[:] = False
            # Re-mark static walls so they stay explored
            if self.static_map is not None:
                exp_static = self._resample_to_explore(self.static_map)
                self._explore[exp_static == 100] = True

    def _publish_explore(self, ref):
        if self._explore is None or self._exp_w == 0:
            return
        from nav_msgs.msg import MapMetaData
        explore_data = np.where(self._explore, np.int8(100), np.int8(0))
        info = MapMetaData()
        info.resolution = self.explore_res
        info.width      = self._exp_w
        info.height     = self._exp_h
        info.origin.position.x    = self._exp_ox
        info.origin.position.y    = self._exp_oy
        info.origin.orientation.w = 1.0
        grid = OccupancyGrid()
        grid.header.stamp    = self.get_clock().now().to_msg()
        grid.header.frame_id = ref.header.frame_id
        grid.info            = info
        grid.data            = explore_data.tolist()
        self.explore_pub.publish(grid)

    # ------------------------------------------------------------------
    # FOV helpers
    # ------------------------------------------------------------------

    def _camera_fov_mask(self, ref) -> np.ndarray | None:
        """Bool array (n_cells,) — True where cell is inside the FOV sector (no max-range limit)."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException:
            self.get_logger().warn('TF lookup failed for FOV mask', throttle_duration_sec=2.0)
            return None

        cx = tf.transform.translation.x
        cy = tf.transform.translation.y

        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)
        fx = math.cos(yaw)
        fy = math.sin(yaw)

        dw = ref.info.width
        dh = ref.info.height
        dr = ref.info.resolution
        dox = ref.info.origin.position.x
        doy = ref.info.origin.position.y

        col = np.arange(dw, dtype=np.float32)
        row = np.arange(dh, dtype=np.float32)
        wx = dox + (col + 0.5) * dr
        wy = doy + (row + 0.5) * dr
        WX, WY = np.meshgrid(wx, wy)

        dx = WX.ravel() - cx
        dy = WY.ravel() - cy
        dist = np.sqrt(dx * dx + dy * dy)

        cos_angle    = (dx * fx + dy * fy) / (dist + 1e-9)
        cos_half_fov = math.cos(math.radians(self.camera_fov_deg / 2.0))

        # min range only — no max range, so obstacles anywhere in the FOV cone get cleaned
        return (cos_angle >= cos_half_fov) & (dist >= self.camera_min_range)

    # ------------------------------------------------------------------
    # Explore grid helpers
    # ------------------------------------------------------------------

    def _rebuild_explore_grid(self, ref):
        key = (ref.info.width, ref.info.height, ref.info.resolution)
        if key == self._exp_static_key:
            return
        self._exp_static_key = key
        w_m = ref.info.width  * ref.info.resolution
        h_m = ref.info.height * ref.info.resolution
        self._exp_w  = max(1, int(math.ceil(w_m / self.explore_res)))
        self._exp_h  = max(1, int(math.ceil(h_m / self.explore_res)))
        self._exp_ox = ref.info.origin.position.x
        self._exp_oy = ref.info.origin.position.y
        self._explore = np.zeros(self._exp_w * self._exp_h, dtype=bool)

    def _resample_to_explore(self, src) -> np.ndarray:
        """Resample any OccupancyGrid onto the coarse explore grid."""
        sr  = src.info.resolution
        sox = src.info.origin.position.x
        soy = src.info.origin.position.y
        sw  = src.info.width
        sh  = src.info.height

        col = np.arange(self._exp_w, dtype=np.float32)
        row = np.arange(self._exp_h, dtype=np.float32)
        wx = self._exp_ox + (col + 0.5) * self.explore_res
        wy = self._exp_oy + (row + 0.5) * self.explore_res
        WX, WY = np.meshgrid(wx, wy)

        sx = np.floor((WX - sox) / sr).astype(np.int32)
        sy = np.floor((WY - soy) / sr).astype(np.int32)
        in_bounds = (sx >= 0) & (sx < sw) & (sy >= 0) & (sy < sh)
        idx = sy * sw + sx

        src_arr = np.array(src.data, dtype=np.int8)
        out = np.zeros(self._exp_h * self._exp_w, dtype=np.int8)
        flat_mask = in_bounds.ravel()
        flat_idx  = idx.ravel()
        out[flat_mask] = src_arr[flat_idx[flat_mask]]
        return out

    def _camera_fov_mask_explore(self) -> np.ndarray | None:
        """Bool array (exp_h * exp_w,) for the coarse explore grid."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException:
            return None

        cx = tf.transform.translation.x
        cy = tf.transform.translation.y
        qz = tf.transform.rotation.z
        qw = tf.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)
        fx = math.cos(yaw)
        fy = math.sin(yaw)

        col = np.arange(self._exp_w, dtype=np.float32)
        row = np.arange(self._exp_h, dtype=np.float32)
        wx = self._exp_ox + (col + 0.5) * self.explore_res
        wy = self._exp_oy + (row + 0.5) * self.explore_res
        WX, WY = np.meshgrid(wx, wy)

        dx = WX.ravel() - cx
        dy = WY.ravel() - cy
        dist = np.sqrt(dx * dx + dy * dy)
        cos_angle    = (dx * fx + dy * fy) / (dist + 1e-9)
        cos_half_fov = math.cos(math.radians(self.camera_fov_deg / 2.0))

        return (cos_angle >= cos_half_fov) & (dist >= self.camera_min_range) & (dist <= self.camera_max_range)

    def _robot_proximity_mask_explore(self) -> np.ndarray | None:
        """Bool array (exp_h * exp_w,) — True for cells within robot_explore_radius of the robot."""
        if self._exp_w == 0:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rclpy.time.Time()
            )
        except TransformException:
            return None

        cx = tf.transform.translation.x
        cy = tf.transform.translation.y

        col = np.arange(self._exp_w, dtype=np.float32)
        row = np.arange(self._exp_h, dtype=np.float32)
        wx = self._exp_ox + (col + 0.5) * self.explore_res
        wy = self._exp_oy + (row + 0.5) * self.explore_res
        WX, WY = np.meshgrid(wx, wy)

        dx = WX.ravel() - cx
        dy = WY.ravel() - cy
        dist = np.sqrt(dx * dx + dy * dy)
        return dist <= self.robot_explore_radius

    # ------------------------------------------------------------------
    # Bear exclusion zones
    # ------------------------------------------------------------------

    _EXCLUSION_ZONES = [
        (-2.0, -2.0, 1.2),  # (centre_x, centre_y, side)
        ( -2.0, 2.0, 1.2),
    ]

    def _paint_exclusion_zones(self, grid2d, ref):
        """Mark each exclusion square as occupied once the robot has left it."""
        # Get robot world position (best-effort; skip update if TF unavailable)
        robot_x, robot_y = None, None
        try:
            tf = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rclpy.time.Time())
            robot_x = tf.transform.translation.x
            robot_y = tf.transform.translation.y
        except TransformException:
            pass

        res = ref.info.resolution
        ox  = ref.info.origin.position.x
        oy  = ref.info.origin.position.y
        rh, rw = grid2d.shape

        for i, (cx, cy, side) in enumerate(self._EXCLUSION_ZONES):
            # Never paint the zone on the same side as the car's start position
            if i == self._skip_zone_idx:
                continue

            half = side / 2.0

            # Update exited flag when we know the robot position
            if robot_x is not None and not self._zone_exited[i]:
                if not (abs(robot_x - cx) <= half * 1.5 and abs(robot_y - cy) <= half * 1.5):
                    self._zone_exited[i] = True

            if not self._zone_exited[i]:
                continue  # robot still inside this zone — don't paint yet

            col_lo = max(0,  int(math.floor((cx - half - ox) / res)))
            col_hi = min(rw, int(math.ceil( (cx + half - ox) / res)))
            row_lo = max(0,  int(math.floor((cy - half - oy) / res)))
            row_hi = min(rh, int(math.ceil( (cy + half - oy) / res)))
            grid2d[row_lo:row_hi, col_lo:col_hi] = 100

    # ------------------------------------------------------------------
    # Grid resampling
    # ------------------------------------------------------------------

    def _resample(self, src, dst):
        """Project src OccupancyGrid onto dst grid by nearest-neighbour lookup."""
        dw = dst.info.width
        dh = dst.info.height
        dr = dst.info.resolution
        dox = dst.info.origin.position.x
        doy = dst.info.origin.position.y

        sr  = src.info.resolution
        sox = src.info.origin.position.x
        soy = src.info.origin.position.y
        sw  = src.info.width
        sh  = src.info.height

        col = np.arange(dw, dtype=np.float32)
        row = np.arange(dh, dtype=np.float32)
        wx = dox + (col + 0.5) * dr
        wy = doy + (row + 0.5) * dr
        WX, WY = np.meshgrid(wx, wy)

        sx = np.floor((WX - sox) / sr).astype(np.int32)
        sy = np.floor((WY - soy) / sr).astype(np.int32)

        in_bounds = (sx >= 0) & (sx < sw) & (sy >= 0) & (sy < sh)
        idx = sy * sw + sx

        src_arr = np.array(src.data, dtype=np.int8)
        out = np.zeros(dh * dw, dtype=np.int8)
        flat_mask = in_bounds.ravel()
        flat_idx  = idx.ravel()
        out[flat_mask] = src_arr[flat_idx[flat_mask]]
        return out


def main(args=None):
    rclpy.init(args=args)
    node = FusedObstacleMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
