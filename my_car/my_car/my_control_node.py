import math
import os
import cv2
from datetime import datetime

import rclpy
from rclpy.node import Node
from pynput import keyboard
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformException, TransformListener

from enum import Enum
from std_msgs.msg import String, Empty, Int32
from sensor_msgs.msg import Image, PointCloud
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path, OccupancyGrid


class State(Enum):
    FINDING  = 0
    ADJUST   = 1
    APPROACH = 2
    CLAMP    = 3
    LIFT     = 4
    RELEASE  = 5


class MyControlNode(Node):

    # ------------------------------------------------------------------ timings
    _CLAMP_HOLD_NS      = 120_000_000    # 120 ms
    _LIFT_HOLD_NS       = 1_000_000_000  # 1 s
    _RELEASE_HOLD_NS    = 500_000_000    # 500 ms
    _ADJUST_TIMEOUT_NS  = 5_000_000_000  # 5 s
    _ALIGN_STEADY_NS    = 1_000_000_000  # 1 s heading steady before APPROACH
    _ARM_TRAJ_NS        = 100_000_000    # 100 ms arm trajectory duration
    # ------------------------------------------------------------------ distances
    _CLAMP_TRIGGER_M    = 0.07   # IR reading that triggers clamp

    def __init__(self):
        super().__init__('my_control_node')

        # Publishers
        self.wheel_pub       = self.create_publisher(TwistStamped,   '/base_controller/cmd_vel',          10)
        self.arm_pub         = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory',  10)
        self.state_pub       = self.create_publisher(String,          '/my_control/state',                 10)
        self.bear_clamped_pub = self.create_publisher(Empty,          '/bear_map/remove_clamped',          10)

        # Subscribers
        self.create_subscription(Int32,        '/sensor/laser',            self._on_ir_distance,    10)
        self.create_subscription(Path,         '/astar_path',              self._on_path,           10)
        self.create_subscription(OccupancyGrid,'/obstacle_map',            self._on_obstacle_map,   10)
        self.create_subscription(PointCloud,   '/bear_map',                self._on_bear_map,       10)
        self.create_subscription(Image,        '/camera/color/image_raw',  self._on_image,          10)

        # Navigation tuning
        self.lin_vel_scale              = 0.35
        self.ang_vel_scale              = 1.0
        self.arm_speed                  = 5.0   # deg per timer tick
        self.path_waypoint_tolerance    = 0.3   # m
        self.path_goal_tolerance        = 0.3   # m
        self.path_heading_gain          = 3.0
        self.path_linear_speed          = 0.3   # m/s
        self.path_max_angular_speed     = 1.2   # rad/s
        self.path_long_distance_threshold = 0.15  # m
        self.adjust_heading_tolerance   = 0.08  # rad
        self.approach_max_target_distance = 10.0  # m — abort if bear too far
        self.approach_min_target_distance = 0.07  # m — clamp if bear this close
        self.approach_front_obstacle_distance = 0.42   # m
        self.approach_front_check_half_width  = 0.15   # m
        self.approach_map_occupancy_threshold = 50
        self.robot_frame = 'car_base'

        # Joint definitions  [shoulder, elbow, gripper]
        self.joint_limits = [
            {"length": 0.08089007, "min_angle": -180, "max_angle":  0,  "init": -180, "offset":  270, "dir": -1.0},
            {"length": 0.11,       "min_angle": -240, "max_angle":  0,  "init":    0, "offset": -120, "dir": -1.0},
            {"length": 0.00,       "min_angle":   20, "max_angle": 90,  "init":   90, "offset":  0.0, "dir":  1.0},
        ]
        self.joint_limits_physical = [
            {"min_angle":  30, "max_angle": 210, "init":  90},
            {"min_angle":   0, "max_angle": 240, "init":  90},
            {"min_angle": 130, "max_angle": 240, "init": 200},
        ]

        # Arm poses (radians → converted to degrees for the physical interface)
        _poses_rad = {
            'ready':   [3.32, 0.79, 3.40],
            'clamp':   [3.32, 0.79, 2.90],
            'lift':    [0.79, 1.40, 2.90],
            'release': [0.79, 1.40, 3.57],
        }
        self.ready_arm_pose_physical   = [math.degrees(a) for a in _poses_rad['ready']]
        self.clamp_arm_pose_physical   = [math.degrees(a) for a in _poses_rad['clamp']]
        self.lift_arm_pose_physical    = [math.degrees(a) for a in _poses_rad['lift']]
        self.release_arm_pose_physical = [math.degrees(a) for a in _poses_rad['release']]

        # Runtime state
        self.state        = State.FINDING
        self.auto_drive   = False
        self.lin_vel      = 0.0
        self.ang_vel      = 0.0
        self.keys         = set()
        self.joint_angles_physical    = [j["init"] for j in self.joint_limits_physical]
        self._last_arm_command        = None

        self.enterstate_time   = self.get_clock().now()
        self.steady_time       = self.get_clock().now()
        self.adjust_start_time = self.get_clock().now()

        self.current_path      = []
        self.path_frame_id     = ''
        self.obstacle_map_msg  = None
        self.bear_map_points   = []
        self.bear_map_frame_id = ''
        self.latest_image_msg  = None

        self.bridge       = CvBridge()
        self.capture_dir  = os.path.join(os.path.dirname(__file__), '..', 'captures')
        os.makedirs(self.capture_dir, exist_ok=True)
        self.tf_buffer    = Buffer()
        self.tf_listener  = TransformListener(self.tf_buffer, self)

        self.kb_listener = keyboard.Listener(on_press=self._on_key_press, on_release=self._on_key_release)
        self.kb_listener.start()

        self.create_timer(0.1, self._control_tick)

        self.get_logger().info("Node started — MANUAL mode. 'q' toggles AUTO, WASD to drive.")

    # ------------------------------------------------------------------
    # Keyboard
    # ------------------------------------------------------------------

    def _on_key_press(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return
        self.keys.add(k)
        if k == 'q':
            self.auto_drive = not self.auto_drive
            self.state = State.FINDING
            self.get_logger().info(f"Mode → {'AUTO' if self.auto_drive else 'MANUAL'}")
        elif k == 'c':
            self._save_image()

    def _on_key_release(self, key):
        try:
            k = key.char.lower()
        except AttributeError:
            return
        self.keys.discard(k)

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------

    def _on_path(self, msg):
        self.path_frame_id = msg.header.frame_id
        self.current_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(
            f"Path received: {len(self.current_path)} waypoints in '{self.path_frame_id}'"
        )

    def _on_obstacle_map(self, msg):
        self.obstacle_map_msg = msg

    def _on_bear_map(self, msg):
        self.bear_map_frame_id = msg.header.frame_id
        self.bear_map_points = [(pt.x, pt.y) for pt in msg.points]

    def _on_image(self, msg):
        self.latest_image_msg = msg

    def _on_ir_distance(self, msg):
        if not self.auto_drive:
            return

        distance_m = msg.data / 1000.0
        now = self.get_clock().now()

        if self.state == State.APPROACH:
            self._handle_approach(distance_m, now)
        elif self.state == State.CLAMP:
            self._handle_clamp(now)
        elif self.state == State.LIFT:
            self._handle_lift(now)
        elif self.state == State.RELEASE:
            self._handle_release(now)

    # ------------------------------------------------------------------
    # State handlers (called from _on_ir_distance)
    # ------------------------------------------------------------------

    def _handle_approach(self, distance_m, now):
        self.lin_vel = 0.0
        self.ang_vel = 0.0

        if self.is_obstacle_close_in_front_on_map():
            self._enter_state(State.CLAMP, now)
            self._publish_clamped_bear()
            return

        bear_dist = self._closest_bear_distance()
        if bear_dist is not None:
            if bear_dist > self.approach_max_target_distance:
                print(f'Bear too far ({bear_dist:.2f} m) — returning to FINDING')
                self._enter_finding(now)
                return
            if bear_dist < self.approach_min_target_distance:
                print(f'Bear too close ({bear_dist:.2f} m) — clamping')
                self._publish_clamped_bear()
                self._enter_finding(now)
                return

        if distance_m > self._CLAMP_TRIGGER_M:
            self.lin_vel = self.lin_vel_scale
        else:
            self._publish_clamped_bear()
            self._enter_state(State.CLAMP, now)

        self.joint_angles_physical = self.ready_arm_pose_physical.copy()

    def _handle_clamp(self, now):
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.joint_angles_physical = self.clamp_arm_pose_physical.copy()
        if (now - self.enterstate_time).nanoseconds > self._CLAMP_HOLD_NS:
            self._enter_state(State.LIFT, now)

    def _handle_lift(self, now):
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.joint_angles_physical = self.lift_arm_pose_physical.copy()
        if (now - self.enterstate_time).nanoseconds > self._LIFT_HOLD_NS:
            self._enter_state(State.RELEASE, now)

    def _handle_release(self, now):
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.joint_angles_physical = self.release_arm_pose_physical.copy()
        if (now - self.enterstate_time).nanoseconds > self._RELEASE_HOLD_NS:
            self._enter_finding(now)

    # ------------------------------------------------------------------
    # Main control tick (10 Hz timer)
    # ------------------------------------------------------------------

    def _control_tick(self):
        if self.auto_drive:
            self._auto_drive_tick()
        else:
            self._manual_drive_tick()

        state_msg = String()
        state_msg.data = f'{"AUTO" if self.auto_drive else "MANUAL"}:{self.state.name}'
        self.state_pub.publish(state_msg)

    def _auto_drive_tick(self):
        if self.state == State.FINDING:
            if self._is_path_long():
                self._follow_path()
            else:
                self.lin_vel = 0.0
                self.ang_vel = 0.0
                self.adjust_start_time = self.get_clock().now()
                self.steady_time       = self.get_clock().now()
                self.state = State.ADJUST
        elif self.state == State.ADJUST:
            self._adjust_to_bear()

        if self.state in (State.FINDING, State.ADJUST):
            self.joint_angles_physical = self.ready_arm_pose_physical.copy()

        self._publish_wheel(self.lin_vel, self.ang_vel)
        self._publish_arm(self.joint_angles_physical)

    def _manual_drive_tick(self):
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        if 'w' in self.keys: self.lin_vel =  self.lin_vel_scale
        if 's' in self.keys: self.lin_vel = -self.lin_vel_scale
        if 'a' in self.keys: self.ang_vel =  self.ang_vel_scale
        if 'd' in self.keys: self.ang_vel = -self.ang_vel_scale
        self._publish_wheel(self.lin_vel, self.ang_vel)

        if 'n' in self.keys:
            self.joint_angles_physical = [j["init"] for j in self.joint_limits_physical]
        arm_keys = [('u', 0, +1), ('j', 0, -1), ('i', 1, +1), ('k', 1, -1), ('o', 2, +1), ('l', 2, -1)]
        for key, joint_idx, sign in arm_keys:
            if key in self.keys:
                self.joint_angles_physical[joint_idx] += sign * self.arm_speed

        for i, jlim in enumerate(self.joint_limits_physical):
            self.joint_angles_physical[i] = max(jlim["min_angle"], min(jlim["max_angle"], self.joint_angles_physical[i]))

        self._publish_arm(self.joint_angles_physical)

    # ------------------------------------------------------------------
    # Navigation helpers
    # ------------------------------------------------------------------

    def _follow_path(self):
        robot_pose = self._robot_pose_in(self.path_frame_id)
        if robot_pose is None:
            self.lin_vel = self.ang_vel = 0.0
            return

        rx, ry, ryaw = robot_pose

        while self.current_path:
            wx, wy = self.current_path[0]
            tol = self.path_goal_tolerance if len(self.current_path) == 1 else self.path_waypoint_tolerance
            if math.hypot(wx - rx, wy - ry) > tol:
                break
            self.current_path.pop(0)

        if not self.current_path:
            self.lin_vel = self.ang_vel = 0.0
            self.get_logger().info('Reached end of path')
            return

        tx, ty = self.current_path[0]
        heading_err = self._normalize_angle(math.atan2(ty - ry, tx - rx) - ryaw)
        dist_err    = math.hypot(tx - rx, ty - ry)

        self.ang_vel = max(-self.path_max_angular_speed,
                           min( self.path_max_angular_speed, self.path_heading_gain * heading_err))
        self.lin_vel = 0.0 if abs(heading_err) > 0.2 else min(self.path_linear_speed, dist_err)

    def _is_path_long(self):
        if not self.current_path or not self.path_frame_id:
            return False
        robot_pose = self._robot_pose_in(self.path_frame_id)
        if robot_pose is None:
            return False
        rx, ry, _ = robot_pose
        gx, gy = self.current_path[-1]
        return math.hypot(gx - rx, gy - ry) > self.path_long_distance_threshold

    def _adjust_to_bear(self):
        now = self.get_clock().now()
        if (now - self.adjust_start_time).nanoseconds > self._ADJUST_TIMEOUT_NS:
            self._enter_finding(now)
            self.lin_vel = self.ang_vel = 0.0
            return

        robot_pose = self._robot_pose_in(self.bear_map_frame_id)
        if robot_pose is None or not self.bear_map_points:
            self.lin_vel = self.ang_vel = 0.0
            return

        rx, ry, ryaw = robot_pose
        tx, ty = min(self.bear_map_points, key=lambda p: math.hypot(p[0] - rx, p[1] - ry))

        heading_err = self._normalize_angle(math.atan2(ty - ry, tx - rx) - ryaw)
        self.lin_vel = 0.0
        self.ang_vel = max(-self.path_max_angular_speed,
                           min( self.path_max_angular_speed, self.path_heading_gain * heading_err))

        if abs(heading_err) <= self.adjust_heading_tolerance:
            if (now - self.steady_time).nanoseconds > self._ALIGN_STEADY_NS:
                self._enter_state(State.APPROACH, now)
        else:
            self.steady_time = now

    def _closest_bear_distance(self):
        if not self.bear_map_points or not self.bear_map_frame_id:
            return None
        robot_pose = self._robot_pose_in(self.bear_map_frame_id)
        if robot_pose is None:
            return None
        rx, ry, _ = robot_pose
        return min(math.hypot(p[0] - rx, p[1] - ry) for p in self.bear_map_points)

    def is_obstacle_close_in_front_on_map(self):
        if self.obstacle_map_msg is None:
            return False
        robot_pose = self._robot_pose_in(self.obstacle_map_msg.header.frame_id)
        if robot_pose is None:
            return False

        rx, ry, ryaw = robot_pose
        resolution   = float(self.obstacle_map_msg.info.resolution)
        step         = max(0.5 * resolution, 0.02)
        lateral_step = max(0.5 * resolution, 0.03)

        forward = step
        while forward <= self.approach_front_obstacle_distance:
            lateral = -self.approach_front_check_half_width
            while lateral <= self.approach_front_check_half_width:
                wx = rx + math.cos(ryaw) * forward - math.sin(ryaw) * lateral
                wy = ry + math.sin(ryaw) * forward + math.cos(ryaw) * lateral
                cell = self._world_to_grid(self.obstacle_map_msg, wx, wy)
                if cell is None:
                    return True
                idx  = cell[1] * self.obstacle_map_msg.info.width + cell[0]
                cost = self.obstacle_map_msg.data[idx]
                if cost < 0 or cost >= self.approach_map_occupancy_threshold:
                    return True
                lateral += lateral_step
            forward += step

        return False

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _robot_pose_in(self, frame_id):
        if not frame_id:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(frame_id, self.robot_frame, rclpy.time.Time())
        except TransformException:
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return x, y, yaw

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _world_to_grid(map_msg, x, y):
        origin = map_msg.info.origin.position
        res    = map_msg.info.resolution
        gx = int(math.floor((x - origin.x) / res))
        gy = int(math.floor((y - origin.y) / res))
        if 0 <= gx < map_msg.info.width and 0 <= gy < map_msg.info.height:
            return gx, gy
        return None

    def _enter_state(self, new_state, now=None):
        if now is None:
            now = self.get_clock().now()
        self.state = new_state
        self.enterstate_time = now

    def _enter_finding(self, now=None):
        if now is None:
            now = self.get_clock().now()
        self.state = State.FINDING
        self.enterstate_time = now
        self.steady_time     = now

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------

    def _publish_wheel(self, lin_vel, ang_vel):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = lin_vel
        msg.twist.angular.z = ang_vel
        self.wheel_pub.publish(msg)

    def _publish_clamped_bear(self):
        self.bear_clamped_pub.publish(Empty())

    def _publish_arm(self, angles_deg):
        clamped = [
            float(max(jlim["min_angle"], min(jlim["max_angle"], a)))
            for a, jlim in zip(angles_deg, self.joint_limits_physical)
        ]

        if self._last_arm_command is not None:
            if all(abs(clamped[i] - self._last_arm_command[i]) <= 0.1 for i in range(len(clamped))):
                return

        traj = JointTrajectory()
        traj.header.stamp  = self.get_clock().now().to_msg()
        traj.joint_names   = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']
        pt = JointTrajectoryPoint()
        pt.positions       = [math.radians(a) for a in clamped]
        pt.time_from_start.nanosec = self._ARM_TRAJ_NS
        traj.points.append(pt)
        self.arm_pub.publish(traj)
        self._last_arm_command = tuple(clamped)

    def _save_image(self):
        if self.latest_image_msg is None:
            self.get_logger().warn('No image received yet')
            return
        try:
            cv_image  = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            path      = os.path.join(self.capture_dir, f'capture_{timestamp}.png')
            if not cv2.imwrite(path, cv_image):
                self.get_logger().error(f'Failed to write {path}')
                return
            self.get_logger().info(f'Saved {path}')
        except Exception as exc:
            self.get_logger().error(f'Error saving image: {exc}')

    def destroy_node(self):
        self.kb_listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MyControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
