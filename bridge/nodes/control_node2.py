#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from pynput import keyboard
from tf2_ros import Buffer, TransformException, TransformListener

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Empty


class State(Enum):
    GET_ENTRY = 0   # follow A* path to the bridge entrance
    ADJUST    = 1   # rotate in place to face the bridge centre
    PASS      = 2   # drive straight through the bridge
    RETURN    = 3   # navigate back to start zone after crossing
    DONE      = 4


class ControlNode2(Node):

    # Tuning
    _ENTRY_GOAL_TOL   = 0.15          # m — how close to entrance before switching to ADJUST
    _ADJUST_HDG_TOL   = 0.08          # rad — heading error considered "aligned"
    _ADJUST_STEADY_NS = 800_000_000   # 0.8 s heading must be steady before PASS
    _PASS_DISTANCE    = 1.5           # m — how far to drive through the bridge
    _PASS_SPEED       = 0.25          # m/s

    def __init__(self):
        super().__init__('bridge_control_node2')

        # Navigation tuning
        self.lin_vel_scale               = 0.35
        self.ang_vel_scale               = 1.0
        self.path_waypoint_tolerance     = 0.2
        self.path_goal_tolerance         = 0.15
        self.path_heading_gain           = 1.0
        self.path_linear_speed           = 0.5
        self.path_max_angular_speed      = 1.2
        self.robot_frame                 = 'car_base'

        self.auto_drive          = False
        self.lin_vel             = 0.0
        self.ang_vel             = 0.0
        self.keys                = set()
        self.state               = State.GET_ENTRY
        self.current_path        = []
        self.path_frame_id       = ''
        self._path_ever_received = False
        self._entrance_pose: PoseStamped | None = None
        self._centre_pose:   PoseStamped | None = None

        self._steady_time     = None
        self._pass_start_pose = None

        self.declare_parameter('start_side', 'right')
        start_side = self.get_parameter('start_side').value
        self._return_goal = (-1.75, -1.75) if start_side == 'right' else (-1.75, 1.75)

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wheel_pub  = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        self.state_pub  = self.create_publisher(String,       '/my_control/state',       10)
        self.goal_pub   = self.create_publisher(PoseStamped,  '/path_goal',              10)
        self.reinit_pub = self.create_publisher(Empty,        '/lidar/reinit',           10)

        self.create_subscription(Path,        '/astar_path',      self._on_path,     10)
        self.create_subscription(PoseStamped, '/bridge/entrance', self._on_entrance, 10)
        self.create_subscription(PoseStamped, '/bridge/centre',   self._on_centre,   10)

        self.kb_listener = keyboard.Listener(
            on_press=self._on_key_press, on_release=self._on_key_release
        )
        self.kb_listener.start()

        self.create_timer(0.1, self._control_tick)
        self.get_logger().info("bridge_control_node2 — MANUAL. 'q' toggles AUTO, WASD to drive.")

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
            if self.auto_drive:
                self.state = State.GET_ENTRY
            self.get_logger().info(f"Mode → {'AUTO' if self.auto_drive else 'MANUAL'}")

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
        self._path_ever_received = True
        self.get_logger().info(
            f"Path received: {len(self.current_path)} waypoints in '{self.path_frame_id}'"
        )

    def _on_entrance(self, msg):
        self._entrance_pose = msg
        if self.state == State.GET_ENTRY:
            self.goal_pub.publish(msg)

    def _on_centre(self, msg):
        self._centre_pose = msg

    # ------------------------------------------------------------------
    # Main control tick (10 Hz)
    # ------------------------------------------------------------------

    def _control_tick(self):
        if self.auto_drive:
            self._auto_drive_tick()
        else:
            self._manual_drive_tick()

        msg = String()
        msg.data = f'{"AUTO" if self.auto_drive else "MANUAL"}:{self.state.name}'
        self.state_pub.publish(msg)

    def _auto_drive_tick(self):
        if self.state == State.GET_ENTRY:
            self._get_entry_tick()
        elif self.state == State.ADJUST:
            self._adjust_tick()
        elif self.state == State.PASS:
            self._pass_tick()
        elif self.state == State.RETURN:
            self._return_tick()
        elif self.state == State.DONE:
            self._publish_wheel(0.0, 0.0)

    def _manual_drive_tick(self):
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        if 'w' in self.keys: self.lin_vel =  self.lin_vel_scale
        if 's' in self.keys: self.lin_vel = -self.lin_vel_scale
        if 'a' in self.keys: self.ang_vel =  self.ang_vel_scale
        if 'd' in self.keys: self.ang_vel = -self.ang_vel_scale
        self._publish_wheel(self.lin_vel, self.ang_vel)

    # ------------------------------------------------------------------
    # State handlers
    # ------------------------------------------------------------------

    def _get_entry_tick(self):
        if self._entrance_pose is not None:
            robot_pose = self._robot_pose_in('map')
            if robot_pose is not None:
                rx, ry, _ = robot_pose
                gx = self._entrance_pose.pose.position.x
                gy = self._entrance_pose.pose.position.y
                if math.hypot(gx - rx, gy - ry) < self._ENTRY_GOAL_TOL:
                    self._publish_wheel(0.0, 0.0)
                    self.state = State.ADJUST
                    self._steady_time = self.get_clock().now()
                    self.get_logger().info('Reached entrance — ADJUST')
                    return

        self._follow_path()
        self._publish_wheel(self.lin_vel, self.ang_vel)

    def _adjust_tick(self):
        if self._centre_pose is None:
            self._publish_wheel(0.0, 0.0)
            return

        robot_pose = self._robot_pose_in('map')
        if robot_pose is None:
            self._publish_wheel(0.0, 0.0)
            return

        rx, ry, ryaw = robot_pose
        cx = self._centre_pose.pose.position.x
        cy = self._centre_pose.pose.position.y
        heading_err = self._normalize_angle(math.atan2(cy - ry, cx - rx) - ryaw)

        now = self.get_clock().now()
        if abs(heading_err) < self._ADJUST_HDG_TOL:
            if (now - self._steady_time).nanoseconds > self._ADJUST_STEADY_NS:
                self._publish_wheel(0.0, 0.0)
                self.state = State.PASS
                self._pass_start_pose = (rx, ry)
                self.get_logger().info('Aligned — PASS (straight, no HC)')
                return
        else:
            self._steady_time = now

        ang_vel = math.copysign(
            min(self.path_max_angular_speed, self.path_heading_gain * abs(heading_err)),
            heading_err,
        )
        self._publish_wheel(0.0, ang_vel)

    def _pass_tick(self):
        robot_pose = self._robot_pose_in('map')
        if robot_pose is None:
            self._publish_wheel(self._PASS_SPEED, 0.0)
            return

        rx, ry, _ = robot_pose
        sx, sy = self._pass_start_pose
        dist = math.hypot(rx - sx, ry - sy)
        self.get_logger().info(f'PASS  dist={dist:.2f}/{self._PASS_DISTANCE}')

        if dist >= self._PASS_DISTANCE:
            self._publish_wheel(0.0, 0.0)
            self.reinit_pub.publish(Empty())
            self._enter_return()
            return

        self._publish_wheel(self._PASS_SPEED, 0.0)

    def _enter_return(self):
        self.state = State.RETURN
        self.current_path = []
        self._path_ever_received = False
        gx, gy = self._return_goal
        msg = PoseStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = gx
        msg.pose.position.y = gy
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(f'Bridge crossed — RETURN to ({gx}, {gy})')

    def _return_tick(self):
        robot_pose = self._robot_pose_in('map')
        if robot_pose is not None:
            rx, ry, _ = robot_pose
            gx, gy = self._return_goal
            if math.hypot(gx - rx, gy - ry) < self.path_goal_tolerance:
                self._publish_wheel(0.0, 0.0)
                self.state = State.DONE
                self.get_logger().info('Return goal reached — DONE')
                return

        if not self.current_path:
            self._publish_wheel(0.0, self.path_max_angular_speed * 0.5)
            return

        self._follow_path()
        self._publish_wheel(self.lin_vel, self.ang_vel)

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
            return

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

    def _publish_wheel(self, lin_vel, ang_vel):
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = lin_vel
        msg.twist.angular.z = ang_vel
        self.wheel_pub.publish(msg)

    def destroy_node(self):
        self.kb_listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
