#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from pynput import keyboard
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Int32


class State(Enum):
    GET_ENTRY = 0   # follow A* path to the door entrance
    ADJUST    = 1   # rotate in place to face the door knob
    APPROACH  = 2   # drive straight towards the door until laser detects we're close
    READY     = 3   # close the claw and lift the arm
    UNLOCK    = 4   # press the handle
    OPEN      = 5   # drive forward to push the door open


class ControlNode(Node):

    _ENTRY_GOAL_TOL    = 0.15
    _ADJUST_HDG_TOL    = 0.05
    _ADJUST_STEADY_NS  = 800_000_000
    _APPROACH_SPEED    = 0.05
    _APPROACH_LASER_MM = 200
    _READY_CLAMP_NS    = 500_000_000
    _READY_LIFT_NS     = 1_000_000_000
    _UNLOCK_SPEED      = 0.08
    _UNLOCK_DIST_M     = 0.05
    _OPEN_SPEED        = 0.15
    _OPEN_DIST_M       = 0.40
    _ARM_TRAJ_NS       = 100_000_000

    def __init__(self):
        super().__init__('door_control_node')

        self.path_waypoint_tolerance = 0.2
        self.path_goal_tolerance     = 0.15
        self.path_heading_gain       = 3.0
        self.path_linear_speed       = 0.3
        self.path_max_angular_speed  = 1.2
        self.robot_frame             = 'car_base'

        self.auto_drive = False
        self.keys       = set()
        self.state      = State.GET_ENTRY
        self.current_path   = []
        self.path_frame_id  = ''

        self._entrance_pose: PoseStamped | None = None
        self._centre_pose:   PoseStamped | None = None
        self._steady_time      = None
        self._state_start_pose = None
        self._phase_start_ns   = None
        self._laser_mm: int | None = None
        self._last_arm_command = None

        self.tf_buffer   = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.wheel_pub = self.create_publisher(TwistStamped,    '/base_controller/cmd_vel',         10)
        self.state_pub = self.create_publisher(String,          '/my_control/state',                10)
        self.goal_pub  = self.create_publisher(PoseStamped,     '/path_goal',                       10)
        self.arm_pub   = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.create_subscription(Path,        '/astar_path',   self._on_path,     10)
        self.create_subscription(PoseStamped, '/door/ready',   self._on_entrance, 10)
        self.create_subscription(PoseStamped, '/door/knob',    self._on_centre,   10)
        self.create_subscription(Int32,       '/sensor/laser', self._on_laser,    10)

        _poses_rad = {
            'ready':   [3.32, 0.79, 3.80],
            'lift':    [0.79, 1.40, 2.90],
            'clamp':  [3.32, 0.79, 2.90],
            'press':   [3.32, 0.79, 2.90],
        }
        self.ready_arm_pose_physical   = [math.degrees(a) for a in _poses_rad['ready']]
        self.press_arm_pose_physical   = [math.degrees(a) for a in _poses_rad['press']]
        self.lift_arm_pose_physical    = [math.degrees(a) for a in _poses_rad['lift']]
        self.clamp_arm_pose_physical   = [math.degrees(a) for a in _poses_rad['clamp']]
        


        self.kb_listener = keyboard.Listener(
            on_press=self._on_key_press, on_release=self._on_key_release)
        self.kb_listener.start()

        self.create_timer(0.1, self._control_tick)
        self.get_logger().info("door_control_node — MANUAL. 'q' toggles AUTO, WASD to drive.")

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

    def _on_path(self, msg):
        self.path_frame_id = msg.header.frame_id
        self.current_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.get_logger().info(f"Path received: {len(self.current_path)} waypoints in '{self.path_frame_id}'")

    def _on_entrance(self, msg):
        self._entrance_pose = msg
        if self.auto_drive and self.state == State.GET_ENTRY:
            self.goal_pub.publish(msg)

    def _on_centre(self, msg):
        self._centre_pose = msg

    def _on_laser(self, msg):
        self._laser_mm = msg.data

    def _control_tick(self):
        if self.auto_drive:
            self._auto_drive_tick()
        else:
            lin_vel = ang_vel = 0.0
            if 'w' in self.keys: lin_vel =  0.35
            if 's' in self.keys: lin_vel = -0.35
            if 'a' in self.keys: ang_vel =  1.0
            if 'd' in self.keys: ang_vel = -1.0
            self._publish_wheel(lin_vel, ang_vel)

        self.state_pub.publish(String(data=f'{"AUTO" if self.auto_drive else "MANUAL"}:{self.state.name}'))

    def _auto_drive_tick(self):
        if   self.state == State.GET_ENTRY: self._get_entry_tick()
        elif self.state == State.ADJUST:    self._adjust_tick()
        elif self.state == State.APPROACH:  self._approach_tick()
        elif self.state == State.READY:     self._ready_tick()
        elif self.state == State.UNLOCK:    self._unlock_tick()
        elif self.state == State.OPEN:      self._open_tick()

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
                    self.get_logger().info('Reached /door/ready — ADJUST')
                    return

        lin_vel, ang_vel = self._follow_path()
        self._publish_wheel(lin_vel, ang_vel)

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
                self.state = State.APPROACH
                self.get_logger().info('Aligned — APPROACH')
                return
        else:
            self._steady_time = now

        self._publish_wheel(0.0, math.copysign(
            min(self.path_max_angular_speed, self.path_heading_gain * abs(heading_err)),
            heading_err,
        ))
        self._publish_arm(self.ready_arm_pose_physical)

    def _approach_tick(self):
        if self._laser_mm is not None and self._laser_mm < self._APPROACH_LASER_MM:
            self._publish_wheel(0.0, 0.0)
            self.state = State.READY
            self._phase_start_ns = self.get_clock().now().nanoseconds
            self.get_logger().info(f'Door at {self._laser_mm} mm — READY')
            return
        self._publish_arm(self.ready_arm_pose_physical)
        self._publish_wheel(self._APPROACH_SPEED, 0.0)

    def _ready_tick(self):
        elapsed = self.get_clock().now().nanoseconds - self._phase_start_ns

        if elapsed < self._READY_CLAMP_NS:
            self._publish_arm(self.clamp_arm_pose_physical)
        elif elapsed < self._READY_LIFT_NS:
            self._publish_arm(self.lift_arm_pose_physical)
        else:
            self._publish_arm(self.lift_arm_pose_physical)
            robot_pose = self._robot_pose_in('map')
            self._state_start_pose = (robot_pose[0], robot_pose[1]) if robot_pose else (0.0, 0.0)
            self.state = State.UNLOCK
            self.get_logger().info('Arm lifted — UNLOCK')

        self._publish_wheel(0.0, 0.0)

    def _unlock_tick(self):
        robot_pose = self._robot_pose_in('map')
        if robot_pose is not None and self._state_start_pose is not None:
            rx, ry, _ = robot_pose
            sx, sy = self._state_start_pose
            if math.hypot(rx - sx, ry - sy) >= self._UNLOCK_DIST_M:
                self._publish_wheel(0.0, 0.0)
                self._state_start_pose = (rx, ry)
                self.state = State.OPEN
                self.get_logger().info('Handle pressed — OPEN')
                return

        self._publish_wheel(self._UNLOCK_SPEED, 0.0)
        self._publish_arm(self.press_arm_pose_physical)

    def _open_tick(self):
        robot_pose = self._robot_pose_in('map')
        if robot_pose is not None and self._state_start_pose is not None:
            rx, ry, _ = robot_pose
            sx, sy = self._state_start_pose
            if math.hypot(rx - sx, ry - sy) >= self._OPEN_DIST_M:
                self._publish_wheel(0.0, 0.0)
                self.auto_drive = False
                self.get_logger().info('Door opened — DONE')
                return

        self._publish_wheel(self._OPEN_SPEED, 0.0)

    def _follow_path(self):
        robot_pose = self._robot_pose_in(self.path_frame_id)
        if robot_pose is None:
            return 0.0, 0.0

        rx, ry, ryaw = robot_pose

        while self.current_path:
            wx, wy = self.current_path[0]
            tol = self.path_goal_tolerance if len(self.current_path) == 1 else self.path_waypoint_tolerance
            if math.hypot(wx - rx, wy - ry) > tol:
                break
            self.current_path.pop(0)

        if not self.current_path:
            return 0.0, 0.0

        hx, hy = self.current_path[min(1, len(self.current_path) - 1)]
        heading_err = self._normalize_angle(math.atan2(hy - ry, hx - rx) - ryaw)
        tx, ty = self.current_path[0]

        if abs(heading_err) > 0.15:
            return 0.0, math.copysign(
                min(self.path_max_angular_speed, self.path_heading_gain * abs(heading_err)),
                heading_err,
            )
        return min(self.path_linear_speed, math.hypot(tx - rx, ty - ry)), 0.0

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
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
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

    def _publish_arm(self, angles_deg: list):
        if self._last_arm_command is not None:
            if all(abs(angles_deg[i] - self._last_arm_command[i]) <= 0.1 for i in range(len(angles_deg))):
                return
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names  = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']
        pt = JointTrajectoryPoint()
        pt.positions       = [math.radians(a) for a in angles_deg]
        pt.time_from_start.nanosec = self._ARM_TRAJ_NS
        traj.points.append(pt)
        self.arm_pub.publish(traj)
        self._last_arm_command = list(angles_deg)

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
