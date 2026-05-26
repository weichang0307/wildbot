import math
import os
import time as _time
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
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path, OccupancyGrid


class State(Enum):
    FINDING  = 0
    ADJUST   = 1
    APPROACH = 2
    CLAMP    = 3
    LIFT     = 4
    RELEASE  = 5
    FINISH        = 6
    FINISH_ADJUST = 7
    FINISH_PARK   = 8
    FINISH_UNSTOCK = 9
    FINISH_STOP   = 10


class MyControlNode(Node):

    # ------------------------------------------------------------------ timings
    _CLAMP_HOLD_NS      = 120_000_000    # 120 ms
    _LIFT_HOLD_NS       = 1_000_000_000  # 1 s
    _RELEASE_HOLD_NS    = 500_000_000    # 500 ms
    _ADJUST_TIMEOUT_NS  = 5_000_000_000  # 5 s
    _ALIGN_STEADY_NS    = 1_000_000_000  # 1 s heading steady before APPROACH
    _ARM_TRAJ_NS        = 100_000_000    # 100 ms arm trajectory duration
    _FINISH_UNSTOCK_NS         = 5_000_000_000   # total duration of FINISH_UNSTOCK
    _FINISH_UNSTOCK_HALF_NS    =   250_000_000   # half-period at 2 Hz (forward / backward)
    _FINISH_UNSTOCK_DELAY_NS   = 1_000_000_000   # wait before shaking starts
    # ------------------------------------------------------------------ distances
    _CLAMP_TRIGGER_M    = 0.07   # IR reading that triggers clamp
    _FINISH_PARK_DIST_M = 0.35    # back up until this close to the face-away zone

    def __init__(self):
        super().__init__('my_control_node')

        self.declare_parameter('start_side', 'right')  # 'right' → (-1.65,-1.4), 'left' → (1.65,-1.4)
        self.declare_parameter('runtime', 600.0)        # seconds before auto-FINISH

        # Publishers
        self.wheel_pub        = self.create_publisher(TwistStamped,   '/base_controller/cmd_vel',          10)
        self.arm_pub          = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory',  10)
        self.state_pub        = self.create_publisher(String,          '/my_control/state',                 10)
        self.bear_clamped_pub = self.create_publisher(Empty,          '/bear_map/remove_clamped',          10)
        self.goal_pub         = self.create_publisher(PoseStamped,    'move_base_simple/goal',             10)
        self.block_zone_pub   = self.create_publisher(PoseStamped,    '/nav/block_zone',                   10)
        self.servo_pos_pub    = self.create_publisher(Int32,           'actuator/servo_pos',                10)

        # Subscribers
        self.create_subscription(Int32,        '/sensor/laser',            self._on_ir_distance,    10)
        self.create_subscription(Path,         '/astar_path',              self._on_path,           10)
        self.create_subscription(OccupancyGrid,'/fused_obstacle_map',       self._on_obstacle_map,   10)
        self.create_subscription(PointCloud,   '/bear_map',                self._on_bear_map,       10)
        self.create_subscription(Image,        '/camera/color/image_raw',  self._on_image,          10)

        # Navigation tuning
        self.lin_vel_scale              = 0.35
        self.ang_vel_scale              = 1.0
        self.approach_vel               = 0.1
        self.arm_speed                  = 5.0   # deg per timer tick
        self.path_waypoint_tolerance    = 0.3   # m
        self.path_goal_tolerance        = 0.3   # m
        self.path_heading_gain          = 3.0
        self.path_linear_speed          = 0.3   # m/s
        self.path_max_angular_speed     = 1.2   # rad/s
        self.path_long_distance_threshold = 0.5  # m
        self.adjust_heading_tolerance   = 0.08  # rad
        self.adjust_max_bear_distance     = 0.7   # m — only enter ADJUST if bear is this close
        self.approach_max_target_distance = 10.0  # m — abort if bear too far
        self.approach_min_target_distance = 0.07  # m — clamp if bear this close
        self.approach_front_obstacle_distance = 0.45   # m
        self.approach_front_check_half_width  = 0.13 # m — robot half-width
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
            'finding': [3.32, 0.79, 3.60],
        }
        self.ready_arm_pose_physical   = [math.degrees(a) for a in _poses_rad['ready']]
        self.clamp_arm_pose_physical   = [math.degrees(a) for a in _poses_rad['clamp']]
        self.lift_arm_pose_physical    = [math.degrees(a) for a in _poses_rad['lift']]
        self.release_arm_pose_physical = [math.degrees(a) for a in _poses_rad['release']]
        self.finding_arm_pose_physical = [math.degrees(a) for a in _poses_rad['finding']]
        start_side = self.get_parameter('start_side').value
        self._runtime_s = float(self.get_parameter('runtime').value)
        self._finish_goal        = (-1.4,  -1.4) if start_side == 'right' else (-1.4, 1.4)
        self._finish_face_away   = (-1.7,  -1.7) if start_side == 'right' else (-1.7, 1.7)

        self.servo_angle = [90, 40]

        self._pending_block_pos = None   # (x, y) — published to /nav/block_zone once car moves away
        _BLOCK_ARM_DIST         = 1.0    # m — minimum distance before block becomes active
        self._block_arm_dist    = _BLOCK_ARM_DIST

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

        self._time_file = '/ros2_ws/time.txt'

        self.create_timer(0.1, self._control_tick)
        self.create_timer(5.0, self._check_time_limit)  # 0.2 Hz
        self.create_timer(0.1, self._pub_servo_pos)

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
            # self.state = State.APPROACH
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
            self.get_logger().info
            self.get_logger().warn('Obstacle detected in front on map during APPROACH — clamping')
            self._enter_state(State.CLAMP, now)
            self.lin_vel = 0.0
            self.ang_vel = 0.0
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
            self.lin_vel = self.approach_vel
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

        if self.state == State.FINISH:
            self._finish_drive_tick()
        elif self.state == State.FINISH_ADJUST:
            self._finish_adjust_tick()
        elif self.state == State.FINISH_PARK:
            self._finish_park_tick()
        elif self.state == State.FINISH_UNSTOCK:
            self._finish_unstock_tick()
        elif self.state == State.FINISH_STOP:
            self._publish_wheel(0.0, 0.0)
        elif self.auto_drive:
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
            elif self.bear_map_points and (
                (d := self._closest_bear_distance()) is not None
                and d <= self.adjust_max_bear_distance
            ):
                self.lin_vel = 0.0
                self.ang_vel = 0.0
                self.adjust_start_time = self.get_clock().now()
                self.steady_time       = self.get_clock().now()
                self.state = State.ADJUST
            else:
                self._follow_path()  # rotates to search when current_path is empty
        elif self.state == State.ADJUST:
            self._adjust_to_bear()

        if self.state in (State.FINDING, State.ADJUST):
            self.joint_angles_physical = self.finding_arm_pose_physical.copy()

        self._publish_wheel(self.lin_vel, self.ang_vel)
        self._publish_arm(self.joint_angles_physical)

    def _manual_drive_tick(self):
        if self.is_obstacle_close_in_front_on_map():
            self.get_logger().warn('Obstacle detected in front on map — stopping')

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

        # # No local plan yet — rotate slowly to search
        # if not self.current_path:
        #     self.lin_vel = 0.0
        #     self.ang_vel = self.path_max_angular_speed * 0.4
        #     return

        # Advance past waypoints already within tolerance
        while self.current_path:
            wx, wy = self.current_path[0]
            tol = self.path_goal_tolerance if len(self.current_path) == 1 else self.path_waypoint_tolerance
            if math.hypot(wx - rx, wy - ry) > tol:
                break
            self.current_path.pop(0)

        if not self.current_path:
            self.lin_vel = self.ang_vel = 0.0
            return

        # Use local_path[1] for heading (direction of travel), fall back to [0]
        hx, hy = self.current_path[min(1, len(self.current_path) - 1)]
        heading_err = self._normalize_angle(math.atan2(hy - ry, hx - rx) - ryaw)

        tx, ty = self.current_path[0]
        if abs(heading_err) > 0.15:
            self.lin_vel = 0.0
            self.ang_vel = math.copysign(
                min(self.path_max_angular_speed, self.path_heading_gain * abs(heading_err)),
                heading_err,
            )
        else:
            self.lin_vel = min(self.path_linear_speed, math.hypot(tx - rx, ty - ry))
            self.ang_vel = 0.0

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

        if not self.bear_map_points:
            self._enter_finding(now)
            self.lin_vel = self.ang_vel = 0.0
            return

        robot_pose = self._robot_pose_in(self.bear_map_frame_id)
        if robot_pose is None:
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
        frame = self.obstacle_map_msg.header.frame_id or 'map'
        robot_pose = self._robot_pose_in(frame)
        if robot_pose is None:
            self.get_logger().warn(
                f'is_obstacle_close_in_front_on_map: TF lookup failed for frame "{frame}"',
                throttle_duration_sec=2.0,
            )
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
    # Finish navigation
    # ------------------------------------------------------------------

    def _enter_finish(self):
        if self.state in (State.FINISH, State.FINISH_ADJUST,
                          State.FINISH_PARK, State.FINISH_UNSTOCK, State.FINISH_STOP):
            return
        self._enter_state(State.FINISH)
        fx, fy = self._finish_goal
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = fx
        msg.pose.position.y = fy
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(f'FINISH: navigating to ({fx}, {fy})')

    _FINISH_MIN_NS = 2_000_000_000  # must spend at least 2 s in FINISH before switching

    def _finish_drive_tick(self):
        now = self.get_clock().now()
        if (now - self.enterstate_time).nanoseconds < self._FINISH_MIN_NS:
            self._follow_path()
            self._publish_wheel(self.lin_vel, self.ang_vel)
            return
        robot_pose = self._robot_pose_in('map')
        if robot_pose is not None:
            rx, ry, _ = robot_pose
            fx, fy = self._finish_goal
            if not self._is_path_long() and math.hypot(fx - rx, fy - ry) < self.path_goal_tolerance:
                self._enter_state(State.FINISH_ADJUST)
                self.steady_time = self.get_clock().now()
                self._publish_wheel(0.0, 0.0)
                self.get_logger().info('FINISH goal reached — entering FINISH_ADJUST')
                return
        self._follow_path()
        self._publish_wheel(self.lin_vel, self.ang_vel)

    def _finish_adjust_tick(self):
        robot_pose = self._robot_pose_in('map')
        if robot_pose is None:
            self._publish_wheel(0.0, 0.0)
            return
        rx, ry, ryaw = robot_pose
        ax, ay = self._finish_face_away
        away_yaw = math.atan2(ry - ay, rx - ax)
        heading_err = self._normalize_angle(away_yaw - ryaw)
        now = self.get_clock().now()
        if abs(heading_err) < self.adjust_heading_tolerance:
            self._publish_wheel(0.0, 0.0)
            if (now - self.steady_time).nanoseconds > self._ALIGN_STEADY_NS:
                self._enter_state(State.FINISH_PARK)
                self.get_logger().info('FINISH_ADJUST steady — entering FINISH_PARK')
        else:
            self.steady_time = now
            ang_vel = math.copysign(
                min(self.path_max_angular_speed, self.path_heading_gain * 3 * abs(heading_err)),
                heading_err,
            )
            self._publish_wheel(0.0, ang_vel)

    def _finish_park_tick(self):
        robot_pose = self._robot_pose_in('map')
        if robot_pose is None:
            self._publish_wheel(0.0, 0.0)
            return
        rx, ry, _ = robot_pose
        ax, ay = self._finish_face_away
        dist = math.hypot(rx - ax, ry - ay)
        if dist <= self._FINISH_PARK_DIST_M:
            self._enter_state(State.FINISH_UNSTOCK)
            self._publish_wheel(0.0, 0.0)
            self.get_logger().info(f'FINISH_PARK close enough ({dist:.2f} m) — entering FINISH_UNSTOCK')
        else:
            self._publish_wheel(-self.approach_vel, 0.0)  # reverse

    def _finish_unstock_tick(self):
        now = self.get_clock().now()
        elapsed_ns = (now - self.enterstate_time).nanoseconds
        if elapsed_ns >= self._FINISH_UNSTOCK_NS:
            self._enter_state(State.FINISH_STOP)
            self._publish_wheel(0.0, 0.0)
            self.get_logger().info('FINISH_UNSTOCK done — entering FINISH_STOP')
            return
        if elapsed_ns < self._FINISH_UNSTOCK_DELAY_NS:
            self._publish_wheel(0.0, 0.0)
            return
        # Oscillate at 2 Hz: forward first half-period, backward second
        shake_ns = elapsed_ns - self._FINISH_UNSTOCK_DELAY_NS
        phase = shake_ns % (2 * self._FINISH_UNSTOCK_HALF_NS)
        vel = self.approach_vel if phase < self._FINISH_UNSTOCK_HALF_NS else -self.approach_vel
        self._publish_wheel(vel, 0.0)

    # ------------------------------------------------------------------
    # Time-limit check (0.2 Hz)
    # ------------------------------------------------------------------

    def _pub_servo_pos(self):
        angle = self.servo_angle[1] if self.state == State.FINISH_UNSTOCK else self.servo_angle[0]
        self.servo_pos_pub.publish(Int32(data=angle))

    def _check_time_limit(self):
        self.get_logger().info(f'Checking time limit... current state: {self.state.name}')
        if self.state == State.FINISH:
            return
        try:
            with open(self._time_file, 'r') as f:
                start_dt = datetime.strptime(f.read().strip(), '%Y-%m-%d %H:%M:%S')
        except Exception as exc:
            self.get_logger().error(f'Error reading time file: {exc}')
            return
        elapsed = _time.time() - start_dt.timestamp()
        if elapsed > self._runtime_s:
            self.get_logger().info(f'Time limit reached ({elapsed:.1f} s) — entering FINISH')
            self._enter_finish()
        else:
            self.get_logger().info(f'Time elapsed: {elapsed:.1f} s')

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
        robot_pose = self._robot_pose_in('map')
        if robot_pose is not None:
            rx, ry, _ = robot_pose
            self._pending_block_pos = (rx, ry)

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
