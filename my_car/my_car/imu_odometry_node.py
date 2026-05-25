import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class ImuOdometryNode(Node):
    """
    Dead-reckoning odometry from IMU.

    Yaw  : integrate angular_velocity.z (gyroscope — accurate short-term).
    Position: rotate body-frame linear_acceleration to world frame, then
              double-integrate (drifts over time — expected for raw IMU).

    Bias calibration: the first `bias_samples` messages are averaged while
    the robot is assumed stationary.  Removes sensor offset before integration.

    Publishes:
      imu_odom  (nav_msgs/Odometry)
    Broadcasts TF:  odom → car_base  (if publish_tf param is True)
    """

    def __init__(self):
        super().__init__('imu_odometry')

        self.declare_parameter('initial_x',   -1.6)
        self.declare_parameter('initial_y',   -1.6)
        self.declare_parameter('initial_yaw',  math.pi / 2.0)  # facing +Y
        self.declare_parameter('publish_tf',      False)
        self.declare_parameter('bias_samples',    50)    # ~1 s at 50 Hz
        self.declare_parameter('velocity_decay',  2.0)   # time constant (s); lower = faster decay

        self._x   = self.get_parameter('initial_x').value
        self._y   = self.get_parameter('initial_y').value
        self._yaw = self.get_parameter('initial_yaw').value
        self._publish_tf    = self.get_parameter('publish_tf').value
        bias_n              = self.get_parameter('bias_samples').value
        self._vel_decay_tau = self.get_parameter('velocity_decay').value

        self._vx = 0.0
        self._vy = 0.0
        self._last_t: float | None = None

        # Bias accumulators
        self._bias_ax = 0.0
        self._bias_ay = 0.0
        self._bias_gz = 0.0
        self._bias_count = 0
        self._bias_n = bias_n
        self._calibrated = False

        self._odom_pub = self.create_publisher(Odometry, 'imu_odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Imu, 'sensor/imu', self._imu_cb, 10)
        self.get_logger().info('IMU odometry node started — calibrating bias...')

    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        # print('IMU callback  ax={:.2f} ay={:.2f} gz={:.2f}'.format(
        #     msg.linear_acceleration.x,
        #     msg.linear_acceleration.y,
        #     math.degrees(msg.angular_velocity.z),
        # ))
        # get current time 
        stamp = self.get_clock().now().to_msg()
        t = stamp.sec + stamp.nanosec * 1e-9

        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        gz = msg.angular_velocity.z

        # # ---- bias calibration (robot must be stationary at startup) ----
        # if not self._calibrated:
        #     self._bias_ax += ax
        #     self._bias_ay += ay
        #     self._bias_gz += gz
        #     self._bias_count += 1
        #     if self._bias_count >= self._bias_n:
        #         self._bias_ax /= self._bias_n
        #         self._bias_ay /= self._bias_n
        #         self._bias_gz /= self._bias_n
        #         self._calibrated = True
        #         self.get_logger().info(
        #             f'Bias calibrated  ax={self._bias_ax:.4f}  '
        #             f'ay={self._bias_ay:.4f}  gz={self._bias_gz:.4f}'
        #         )
        #     self._last_t = t
        #     return

        # ---- dt guard ----
        if self._last_t is None:
            self._last_t = t
            return
        dt = t - self._last_t
        if dt <= 0.0 or dt > 0.5:   # skip stale / backward-time messages
            self._last_t = t
            print(f'IMU odom  warning: dt={dt:.3f} s — skipping update')
            return
        self._last_t = t

        

        # ---- remove bias ----
        ax -= self._bias_ax
        ay -= self._bias_ay
        gz -= self._bias_gz

        # ---- integrate yaw from gyroscope ----
        self._yaw += gz * dt
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))  # wrap

        # ---- rotate body acceleration to world frame ----
        c, s = math.cos(self._yaw), math.sin(self._yaw)
        ax_w = ax * c - ay * s
        ay_w = ax * s + ay * c

        # ---- double-integrate for position ----
        self._vx += ax_w * dt
        self._vy += ay_w * dt
        self._x  += self._vx * dt
        self._y  += self._vy * dt

        # ---- exponential velocity decay (prevents unbounded drift) ----
        decay = math.exp(-dt / self._vel_decay_tau)
        self._vx *= decay
        self._vy *= decay

        # print(f'IMU odom  x={self._x:.2f} y={self._y:.2f} yaw={math.degrees(self._yaw):.1f}°  '
        #       f'vx={self._vx:.2f} vy={self._vy:.2f}  ax={ax:.2f} ay={ay:.2f} gz={math.degrees(gz):.1f}°/s')

        self._publish(stamp)

    # ------------------------------------------------------------------
    def _publish(self, stamp):
        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)

        odom = Odometry()
        odom.header.stamp    = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'car_base'

        odom.pose.pose.position.x    = self._x
        odom.pose.pose.position.y    = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x  = self._vx
        odom.twist.twist.linear.y  = self._vy
        odom.twist.twist.angular.z = 0.0  # already integrated into yaw

        self._odom_pub.publish(odom)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = 'odom'
            tf.child_frame_id  = 'car_base'
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = ImuOdometryNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
