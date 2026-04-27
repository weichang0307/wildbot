import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from vision_msgs.msg import Detection2DArray
from trajectory_msgs.msg import JointTrajectoryPoint
from pynput import keyboard # Handles true key press and release
import threading
import math

class MyControlNode(Node):
    def __init__(self):
        super().__init__('my_control_node')
        
        self.front_wheel_pub = self.create_publisher(Float32MultiArray, '/car_C_front_wheel', 10)
        self.rear_wheel_pub = self.create_publisher(Float32MultiArray, '/car_C_rear_wheel', 10)
        self.arm_pub = self.create_publisher(JointTrajectoryPoint, '/robot_arm', 10)

        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/yolo/detections_data',
            self.detection_callback,
            10)

        # State Variables
        self.keys = set() # Track currently pressed keys
        self.velocity = [0.0, 0.0, 0.0, 0.0]
        self.auto_drive = False
        self.base_speed = 500.0
        self.rotate_speed = 700.0
        self.arm_speed = 5.0
        self.joint_limits = [
            {"length": 0.08089007, "min_angle": -180, "max_angle": 0, "init": -180, "offset": 270, "dir": -1.0},  # Joint 0 (Shoulder)
            {"length": 0.11, "min_angle": -240, "max_angle": 0, "init": -0,   "offset": -120, "dir": -1.0},  # Joint 1 (Elbow)
            {"length": 0.00, "min_angle": 20, "max_angle": 90,  "init": 90,  "offset": 0.0, "dir": 1.0},  # Joint 2 (Gripper)
        ]
        self.joint_angles = [joint["init"] for joint in self.joint_limits]

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
            mode = "AUTO" if self.auto_drive else "MANUAL"
            self.get_logger().info(f"Mode switched to: {mode}")
            

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
        if not self.auto_drive:
            return

        if msg.detections:
            x = msg.detections[0].bbox.center.position.x
            ang_factor = x - 320.0
            base = 300.0
            ratio = 2.0
            
            v = [base] * 4
            shift = base * (abs(ang_factor) / 320.0) * ratio
            
            if ang_factor > 0: # Turn Right
                v[0]+=shift; v[1]-=shift; v[2]+=shift; v[3]-=shift
            else: # Turn Left
                v[0]-=shift; v[1]+=shift; v[2]-=shift; v[3]+=shift
            self.velocity = v
        else:
            self.velocity = [0.0, 0.0, 0.0, 0.0]

    def publish_control_commands(self):
        if self.auto_drive:
            self.publish_wheel_speed(self.velocity[:2], self.velocity[2:])
        else:
            # Manual Control based on keys
            f_speed = [0.0, 0.0]
            r_speed = [0.0, 0.0]
            if 'w' in self.keys:
                f_speed = [self.base_speed, self.base_speed]
                r_speed = [self.base_speed, self.base_speed]
            if 's' in self.keys:
                f_speed = [-self.base_speed, -self.base_speed]
                r_speed = [-self.base_speed, -self.base_speed]
            if 'a' in self.keys:
                f_speed[0] -= self.rotate_speed
                r_speed[0] -= self.rotate_speed
                f_speed[1] += self.rotate_speed
                r_speed[1] += self.rotate_speed
            if 'd' in self.keys:
                f_speed[1] -= self.rotate_speed
                r_speed[1] -= self.rotate_speed
                f_speed[0] += self.rotate_speed
                r_speed[0] += self.rotate_speed
            self.publish_wheel_speed(f_speed, r_speed)

            # Manual control of robot arm 
            if 'n' in self.keys:
                self.joint_angles = [joint["init"] for joint in self.joint_limits]
            if 'u' in self.keys:
                self.joint_angles[0] += self.arm_speed * self.joint_limits[0]["dir"]
            if 'j' in self.keys:
                self.joint_angles[0] -= self.arm_speed * self.joint_limits[0]["dir"]
            if 'i' in self.keys:
                self.joint_angles[1] += self.arm_speed * self.joint_limits[1]["dir"]
            if 'k' in self.keys:
                self.joint_angles[1] -= self.arm_speed * self.joint_limits[1]["dir"]
            if 'o' in self.keys:
                self.joint_angles[2] += self.arm_speed * self.joint_limits[2]["dir"]
            if 'l' in self.keys:
                self.joint_angles[2] -= self.arm_speed * self.joint_limits[2]["dir"]

            for i, joint in enumerate(self.joint_limits):
                self.joint_angles[i] = max(joint["min_angle"], min(joint["max_angle"], self.joint_angles[i]))
                
            self.publish_robot_arm_angle(self.joint_angles)

    def publish_wheel_speed(self, front_speed, rear_speed):
        f_msg = Float32MultiArray()
        r_msg = Float32MultiArray()
        f_msg.data = front_speed
        r_msg.data = rear_speed
        self.front_wheel_pub.publish(f_msg)
        self.rear_wheel_pub.publish(r_msg)

    def publish_robot_arm_angle(self, angle): # angle is a list of 3 joint angles in degrees
        joint_pos_radians = [
            math.radians(float(self.joint_angles[i]) * self.joint_limits[i].get("dir", 1.0)) 
            for i in range(len(self.joint_angles))
        ]
        joint_trajectory_point = JointTrajectoryPoint()
        joint_trajectory_point.positions = joint_pos_radians
        joint_trajectory_point.velocities = [0.0] * len(joint_pos_radians)
        self.arm_pub.publish(joint_trajectory_point)

    def destroy_node(self):
        self.listener.stop()
        super().destroy_node()

