import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Empty, Int32
from sensor_msgs.msg import Image, PointCloud
from vision_msgs.msg import Detection2DArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Path, OccupancyGrid
from pynput import keyboard # Handles true key press and release
import threading
import math
import os
from datetime import datetime
import numpy as np
from cv_bridge import CvBridge
from tf2_ros import Buffer, TransformException, TransformListener

from enum import Enum

class STATE(Enum):
    APPROACH = 0
    CLAMP = 1
    LIFT = 2
    ADJUST = 3
    RELEASE = 4
    FINDING = 5

class MyControlNode(Node):
    def __init__(self):
        super().__init__('my_control_node')
        
        self.front_wheel_pub = self.create_publisher(Float32MultiArray, '/car_C_front_wheel', 10)
        self.rear_wheel_pub = self.create_publisher(Float32MultiArray, '/car_C_rear_wheel', 10)
        self.wheel_physical_pub = self.create_publisher(TwistStamped, '/base_controller/cmd_vel', 10)
        self.arm_pub = self.create_publisher(JointTrajectoryPoint, '/robot_arm', 10)
        self.arm_physical_pub = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self.state_pub = self.create_publisher(String, '/my_control/state', 10)
        self.clamped_bear_pub = self.create_publisher(Empty, '/bear_map/remove_clamped', 10)

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections_data',
            self.detection_callback,
            10)
        self.ir_distance_sub = self.create_subscription(
            Int32,
            '/sensor/laser',
            self.ir_distance_callback,
            10)
        self.path_sub = self.create_subscription(
            Path,
            '/astar_path',
            self.path_callback,
            10)
        self.obstacle_map_sub = self.create_subscription(
            OccupancyGrid,
            '/obstacle_map',
            self.obstacle_map_callback,
            10)
        self.bear_map_sub = self.create_subscription(
            PointCloud,
            '/bear_map',
            self.bear_map_callback,
            10)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)

        # State Variables
        self.keys = set() # Track currently pressed keys
        self.auto_drive = False
        self.base_speed = 500.0
        self.rotate_speed = 700.0
        self.lin_vel_scale = 0.35
        self.ang_vel_scale = 1.0
        # self.lin_vel_scale = 1.2
        # self.ang_vel_scale = 2.0
        self.lin_vel = 0.0
        self.ang_vel = 0.0
        self.arm_speed = 5.0
        self.path_waypoint_tolerance = 0.3
        self.path_goal_tolerance = 0.3
        self.path_heading_gain = 3.0
        self.path_linear_speed = 0.3
        self.path_max_angular_speed = 1.2
        self.path_long_distance_threshold = 0.15
        self.adjust_heading_tolerance = 0.08
        self.adjust_max_target_distance = 0.8
        self.approach_max_target_distance = 10.0
        self.approach_min_target_distance = 0.07
        self.approach_front_obstacle_distance = 0.42
        self.approach_front_check_half_width = 0.15
        self.approach_map_occupancy_threshold = 50
        self.robot_frame = 'car_base'
        self.joint_limits = [
            {"length": 0.08089007, "min_angle": -180, "max_angle": 0, "init": -180, "offset": 270, "dir": -1.0},  # Joint 0 (Shoulder)
            {"length": 0.11, "min_angle": -240, "max_angle": 0, "init": -0,   "offset": -120, "dir": -1.0},  # Joint 1 (Elbow)
            {"length": 0.00, "min_angle": 20, "max_angle": 90,  "init": 90,  "offset": 0.0, "dir": 1.0},  # Joint 2 (Gripper)
        ]
        self.joint_limits_physical = [
            {"min_angle": 30, "max_angle": 210, "init": 90},
            {"min_angle": 0, "max_angle": 240, "init": 90},
            {"min_angle": 130, "max_angle": 240, "init": 200},
        ]

        self.ready_arm_pose = [3.32, 0.79, 3.40]
        self.clamp_arm_pose = [3.32, 0.79, 2.9]
        self.lift_arm_pose = [0.79, 1.40, 2.9]
        self.release_arm_pose = [0.79, 1.40, 3.57]

        self.ready_arm_pose_physical = [np.rad2deg(pose) for pose in self.ready_arm_pose]
        self.clamp_arm_pose_physical = [np.rad2deg(pose) for pose in self.clamp_arm_pose]
        self.lift_arm_pose_physical = [np.rad2deg(pose) for pose in self.lift_arm_pose]
        self.release_arm_pose_physical = [np.rad2deg(pose) for pose in self.release_arm_pose]

        self.enterstate_time = self.get_clock().now()
        self.steady_time = self.get_clock().now()
        self.adjust_start_time = self.get_clock().now()
        self.current_path = []
        self.path_frame_id = ''
        self.obstacle_map_msg = None
        self.bear_map_points = []
        self.bear_map_frame_id = ''
        self.latest_image_msg = None
        self.bridge = CvBridge()
        self.capture_dir = os.path.join(os.path.dirname(__file__), '..', 'captures')
        os.makedirs(self.capture_dir, exist_ok=True)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.state = STATE.FINDING

        self.joint_angles = [joint["init"] for joint in self.joint_limits]
        self.joint_angles_physical = [joint["init"] for joint in self.joint_limits_physical]
        self._last_arm_command_physical = None

        # Start the keyboard listener
        # on_press runs when key is hit, on_release runs when key is let go
        self.listener = keyboard.Listener(
            on_press=self.on_press,
            on_release=self.on_release)
        self.listener.start()

        self.timer = self.create_timer(0.1, self.publish_control_commands)
        
        self.get_logger().info("Node started. Mode: MANUAL. Press 'q' to toggle Auto.")
        self.get_logger().info("WASD to move (stops on release).")

    def on_press(self, key):
        try:
            # Handle alphanumeric keys
            k = key.char.lower()
        except AttributeError:
            # Handle special keys (arrows, etc)
            return

        self.keys.add(k)
        if k == 'q': # Toggle Auto-Drive mode
            self.auto_drive = not self.auto_drive
            self.state = STATE.FINDING
            # self.state = STATE.APPROACH
            mode = "AUTO" if self.auto_drive else "MANUAL"
            self.get_logger().info(f"Mode switched to: {mode}")
        elif k == 'c':
            self.save_current_image()
            

    def on_release(self, key):
        try:
            # Handle alphanumeric keys
            k = key.char.lower()
        except AttributeError:
            # Handle special keys (arrows, etc)
            return
        if k in self.keys:
            self.keys.remove(k)

    def detection_callback(self, msg):
        # ADJUST now uses bear-map heading instead of image-space bbox centering.
        return


    def ir_distance_callback(self, msg):
        # if self.is_obstacle_close_in_front_on_map():
        #     print('Obstacle detected in front on map, stopping')
        # print(f"IR Distance: {msg.data} mm")


        if not self.auto_drive:
            return

        distance = msg.data / 1000.0
        threshold = 0.07  # Set a threshold for obstacle avoidance
        # print(f"IR Distance: {distance:.2f} m")

        

        
        if self.state == STATE.APPROACH:
            self.lin_vel = 0.0
            self.ang_vel = 0.0

            if self.is_obstacle_close_in_front_on_map():
                self.state = STATE.FINDING
                self.state = STATE.CLAMP
                self.publish_clamped_bear_event()
                self.enterstate_time = self.get_clock().now()

            bear_distance = self.get_closest_bear_distance()
            if bear_distance is not None and bear_distance > self.approach_max_target_distance:
                self.state = STATE.FINDING
                now = self.get_clock().now()
                self.enterstate_time = now
                self.steady_time = now
                print(f'Bear is too far away (distance={bear_distance:.2f} m), stopping approach and going back to finding')
                return
            if bear_distance is not None and bear_distance < self.approach_min_target_distance:
                self.state = STATE.FINDING
                now = self.get_clock().now()
                self.publish_clamped_bear_event()
                self.enterstate_time = now
                self.steady_time = now
                print(f'Bear is too close (distance={bear_distance:.2f} m), stopping approach aqqsssssssssssssssssssssnd going back to finding')
                return

            now = self.get_clock().now()
            if (now - self.enterstate_time).nanoseconds > 0e9: # Wait 2 seconds before starting to approach
                if distance > threshold:
                    self.lin_vel = 1.0 * self.lin_vel_scale
                    self.ang_vel = 0.0
                else:
                    self.state = STATE.CLAMP
                    self.publish_clamped_bear_event()
                    self.enterstate_time = self.get_clock().now()
            self.joint_angles_physical = self.ready_arm_pose_physical.copy()
        elif self.state == STATE.CLAMP:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            self.joint_angles_physical = self.clamp_arm_pose_physical.copy()
            now = self.get_clock().now()
            if (now - self.enterstate_time).nanoseconds > 1.2e8: # Wait 1 second before lifting
                self.state = STATE.LIFT
                self.enterstate_time = now
        elif self.state == STATE.LIFT:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            self.joint_angles_physical = self.lift_arm_pose_physical.copy()
            now = self.get_clock().now()
            if (now - self.enterstate_time).nanoseconds > 1e9: # Wait 1 second before going back to approach
                self.state = STATE.RELEASE
                self.enterstate_time = now
        elif self.state == STATE.RELEASE:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            self.joint_angles_physical = self.release_arm_pose_physical.copy()
            now = self.get_clock().now()
            if (now - self.enterstate_time).nanoseconds > 5e8: # Wait 2 seconds before going back to approach
                self.state = STATE.FINDING
                self.enterstate_time = now
                self.steady_time = now

    def path_callback(self, msg):
        self.path_frame_id = msg.header.frame_id
        self.current_path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]

        self.get_logger().info(
            f"Received A* path with {len(self.current_path)} waypoints in frame {self.path_frame_id}"
        )

    def image_callback(self, msg):
        self.latest_image_msg = msg

    def obstacle_map_callback(self, msg):
        self.obstacle_map_msg = msg

    def bear_map_callback(self, msg):
        self.bear_map_frame_id = msg.header.frame_id
        self.bear_map_points = [
            (point.x, point.y)
            for point in msg.points
        ]

    def get_closest_bear_distance(self):
        if not self.bear_map_points or not self.bear_map_frame_id:
            return None

        robot_pose = self.get_robot_pose_in_frame(self.bear_map_frame_id)
        if robot_pose is None:
            return None

        robot_x, robot_y, _ = robot_pose
        return min(
            math.hypot(point[0] - robot_x, point[1] - robot_y)
            for point in self.bear_map_points
        )

    def is_obstacle_close_in_front_on_map(self):
        if self.obstacle_map_msg is None:
            return False

        frame_id = self.obstacle_map_msg.header.frame_id
        robot_pose = self.get_robot_pose_in_frame(frame_id)
        if robot_pose is None:
            return False

        robot_x, robot_y, robot_yaw = robot_pose
        max_distance = self.approach_front_obstacle_distance
        half_width = self.approach_front_check_half_width
        threshold = self.approach_map_occupancy_threshold

        resolution = float(self.obstacle_map_msg.info.resolution)
        step = max(0.5 * resolution, 0.02)
        lateral_step = max(0.5 * resolution, 0.03)

        forward = step
        while forward <= max_distance:
            lateral = -half_width
            while lateral <= half_width:
                world_x = robot_x + math.cos(robot_yaw) * forward - math.sin(robot_yaw) * lateral
                world_y = robot_y + math.sin(robot_yaw) * forward + math.cos(robot_yaw) * lateral
                cell = self.world_to_grid_in_map(self.obstacle_map_msg, world_x, world_y)
                if cell is None:
                    return True
                index = cell[1] * self.obstacle_map_msg.info.width + cell[0]
                cost = self.obstacle_map_msg.data[index]
                if cost < 0 or cost >= threshold:
                    return True
                lateral += lateral_step
            forward += step

        return False

    @staticmethod
    def world_to_grid_in_map(map_msg, x, y):
        origin = map_msg.info.origin.position
        resolution = map_msg.info.resolution
        width = map_msg.info.width
        height = map_msg.info.height

        grid_x = int(math.floor((x - origin.x) / resolution))
        grid_y = int(math.floor((y - origin.y) / resolution))
        if 0 <= grid_x < width and 0 <= grid_y < height:
            return (grid_x, grid_y)
        return None

    def save_current_image(self):
        if self.latest_image_msg is None:
            self.get_logger().warn('No image received yet; cannot save frame')
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
            file_path = os.path.join(self.capture_dir, f'capture_{timestamp}.png')

            import cv2
            if not cv2.imwrite(file_path, cv_image):
                self.get_logger().error(f'Failed to save image to {file_path}')
                return

            self.get_logger().info(f'Saved image: {file_path}')
        except Exception as exc:
            self.get_logger().error(f'Error saving image: {exc}')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_robot_pose_in_path_frame(self):
        if not self.path_frame_id:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                self.path_frame_id,
                self.robot_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return x, y, yaw

    def get_robot_pose_in_frame(self, frame_id):
        if not frame_id:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                frame_id,
                self.robot_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        return x, y, yaw

    def follow_current_path(self):
        if not self.current_path or not self.path_frame_id:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            return

        robot_pose = self.get_robot_pose_in_path_frame()
        if robot_pose is None:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            return

        robot_x, robot_y, robot_yaw = robot_pose

        while self.current_path:
            waypoint_x, waypoint_y = self.current_path[0]
            distance = math.hypot(waypoint_x - robot_x, waypoint_y - robot_y)
            tolerance = self.path_goal_tolerance if len(self.current_path) == 1 else self.path_waypoint_tolerance
            if distance > tolerance:
                break
            self.current_path.pop(0)

        if not self.current_path:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            self.get_logger().info('Reached end of A* path')
            return

        target_x, target_y = self.current_path[0]
        target_heading = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = self.normalize_angle(target_heading - robot_yaw)
        distance_error = math.hypot(target_x - robot_x, target_y - robot_y)

        self.ang_vel = max(
            -self.path_max_angular_speed,
            min(self.path_max_angular_speed, self.path_heading_gain * heading_error),
        )

        if abs(heading_error) > 0.2:
            self.lin_vel = 0.0
        else:
            self.lin_vel = min(self.path_linear_speed, distance_error)

    def is_path_still_long(self):
        if not self.current_path or not self.path_frame_id:
            return False

        robot_pose = self.get_robot_pose_in_path_frame()
        if robot_pose is None:
            return False

        robot_x, robot_y, _ = robot_pose
        goal_x, goal_y = self.current_path[-1]
        remaining_distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
        return remaining_distance > self.path_long_distance_threshold

    def adjust_to_closest_bear_on_map(self):
        now = self.get_clock().now()
        if (now - self.adjust_start_time).nanoseconds > 5e9:
            self.state = STATE.FINDING
            self.enterstate_time = now
            self.steady_time = now
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            return

        if not self.bear_map_points or not self.bear_map_frame_id:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            return

        robot_pose = self.get_robot_pose_in_frame(self.bear_map_frame_id)
        if robot_pose is None:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            return

        robot_x, robot_y, robot_yaw = robot_pose
        target_x, target_y = min(
            self.bear_map_points,
            key=lambda point: math.hypot(point[0] - robot_x, point[1] - robot_y),
        )


        target_heading = math.atan2(target_y - robot_y, target_x - robot_x)
        heading_error = self.normalize_angle(target_heading - robot_yaw)
        self.lin_vel = 0.0
        self.ang_vel = max(
            -self.path_max_angular_speed,
            min(self.path_max_angular_speed, self.path_heading_gain * heading_error),
        )

        if abs(heading_error) <= self.adjust_heading_tolerance:
            if (now - self.steady_time).nanoseconds > 1e9:
                self.state = STATE.APPROACH
                self.enterstate_time = now
        else:
            self.steady_time = now
            
            
            

    def publish_control_commands(self):
        # print(f"State: {self.state.name}, Lin Vel: {self.lin_vel:.2f}, Ang Vel: {self.ang_vel:.2f}, Joint Angles: {[round(angle, 1) for angle in self.joint_angles_physical]}, Ready Pose: {[round(angle, 1) for angle in self.ready_arm_pose_physical]}, Clamp Pose: {[round(angle, 1) for angle in self.clamp_arm_pose_physical]}, Lift Pose: {[round(angle, 1) for angle in self.lift_arm_pose_physical]}")

        if self.auto_drive:
            if self.state == STATE.FINDING:
                if self.is_path_still_long():
                    self.follow_current_path()
                else:
                    self.lin_vel = 0.0
                    self.ang_vel = 0.0
                    self.state = STATE.ADJUST
                    self.adjust_start_time = self.get_clock().now()
                    self.steady_time = self.get_clock().now()
            elif self.state == STATE.ADJUST:
                self.adjust_to_closest_bear_on_map()

            if self.state in (STATE.FINDING, STATE.ADJUST):
                self.joint_angles_physical = self.ready_arm_pose_physical.copy()

            self.publish_wheel_speed_physical(self.lin_vel, self.ang_vel)
            self.publish_robot_arm_angle_physical(self.joint_angles_physical)
        else:
            self.lin_vel = 0.0
            self.ang_vel = 0.0
            if 'w' in self.keys:
                self.lin_vel = self.lin_vel_scale
            if 's' in self.keys:
                self.lin_vel = -self.lin_vel_scale
            if 'a' in self.keys:
                self.ang_vel = self.ang_vel_scale
            if 'd' in self.keys:
                self.ang_vel = -self.ang_vel_scale
            self.publish_wheel_speed_physical(self.lin_vel, self.ang_vel)

            # Manual control of robot arm 
            if 'n' in self.keys:
                self.joint_angles_physical = [joint["init"] for joint in self.joint_limits_physical]
            if 'u' in self.keys:
                self.joint_angles_physical[0] += self.arm_speed
            if 'j' in self.keys:
                self.joint_angles_physical[0] -= self.arm_speed
            if 'i' in self.keys:
                self.joint_angles_physical[1] += self.arm_speed
            if 'k' in self.keys:
                self.joint_angles_physical[1] -= self.arm_speed
            if 'o' in self.keys:
                self.joint_angles_physical[2] += self.arm_speed
            if 'l' in self.keys:
                self.joint_angles_physical[2] -= self.arm_speed

            for i, joint in enumerate(self.joint_limits_physical):
                self.joint_angles_physical[i] = max(joint["min_angle"], min(joint["max_angle"], self.joint_angles_physical[i]))
                
            self.publish_robot_arm_angle_physical(self.joint_angles_physical)

        state_msg = String()
        mode = 'AUTO' if self.auto_drive else 'MANUAL'
        state_msg.data = f'{mode}:{self.state.name}'
        self.state_pub.publish(state_msg)

    def publish_wheel_speed_physical(self, lin_vel, ang_vel):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = 'base_link'
        twist_msg.twist.linear.x = lin_vel
        twist_msg.twist.angular.z = ang_vel
        self.wheel_physical_pub.publish(twist_msg)

    def publish_clamped_bear_event(self):
        self.clamped_bear_pub.publish(Empty())


    def publish_robot_arm_angle_physical(self, angles): # angle is a list of 3 joint angles in degrees
        for i, joint in enumerate(self.joint_limits_physical):
            angles[i] = max(joint["min_angle"], min(joint["max_angle"], angles[i]))

        # Clamp angles to joint limits first
        angles_clamped = [float(angles[i]) for i in range(len(angles))]

        # If last command exists, compare with a small tolerance (degrees)
        tol_deg = 0.1
        if self._last_arm_command_physical is not None:
            unchanged = all(
                abs(angles_clamped[i] - self._last_arm_command_physical[i]) <= tol_deg
                for i in range(len(angles_clamped))
            )
            if unchanged:
                return

        joint_pos_radians = [math.radians(angle) for angle in angles_clamped]
        joint_trajectory = JointTrajectory()
        joint_trajectory.header.stamp = self.get_clock().now().to_msg()
        joint_trajectory.joint_names = ['arm_1_joint', 'arm_2_joint', 'gripper_joint']
        point = JointTrajectoryPoint()
        point.positions = joint_pos_radians
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100000000
        joint_trajectory.points.append(point)
        self.arm_physical_pub.publish(joint_trajectory)
        # Record last published angles in degrees
        self._last_arm_command_physical = tuple(angles_clamped)




    def destroy_node(self):
        self.listener.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MyControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()