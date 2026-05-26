import os
import math
import xml.etree.ElementTree as ET

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def _parse_xyz_rpy(text_value: str, default):
    if not text_value:
        return default
    parts = text_value.strip().split()
    if len(parts) != 3:
        return default
    try:
        return [float(parts[0]), float(parts[1]), float(parts[2])]
    except ValueError:
        return default


def _quaternion_from_rpy(roll: float, pitch: float, yaw: float):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q_w = cr * cp * cy + sr * sp * sy
    q_x = sr * cp * cy - cr * sp * sy
    q_y = cr * sp * cy + sr * cp * sy
    q_z = cr * cp * sy - sr * sp * cy
    return q_x, q_y, q_z, q_w


class UrdfPublisherNode(Node):
    def __init__(self):
        super().__init__('urdf_publisher_node')

        self.declare_parameter('urdf_path', '')
        self.declare_parameter('topic_name', '/robot_description')

        urdf_path = self.get_parameter('urdf_path').get_parameter_value().string_value
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        if not urdf_path:
            share_dir = get_package_share_directory('door')
            urdf_path = os.path.join(share_dir, 'wildbot.urdf')

        qos_profile = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(String, topic_name, qos_profile)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.dynamic_transforms = []
        self.tf_timer = None

        try:
            with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
                urdf_content = urdf_file.read()
        except OSError as exc:
            self.get_logger().error(f'Failed to load URDF from {urdf_path}: {exc}')
            return

        self.publisher.publish(String(data=urdf_content))
        self.get_logger().info(f'Published URDF on {topic_name} from {urdf_path}')

        static_transforms, dynamic_transforms = self._build_transforms_from_urdf(urdf_content)
        if static_transforms:
            self.static_tf_broadcaster.sendTransform(static_transforms)
            self.get_logger().info(f'Published {len(static_transforms)} static transforms on /tf_static')

        if dynamic_transforms:
            self.dynamic_transforms = dynamic_transforms
            self.tf_timer = self.create_timer(0.1, self._publish_dynamic_transforms)
            self.get_logger().info(f'Publishing {len(dynamic_transforms)} dynamic transforms on /tf')

    def _build_transforms_from_urdf(self, urdf_content: str):
        static_transforms = []
        dynamic_transforms = []

        try:
            robot = ET.fromstring(urdf_content)
        except ET.ParseError as exc:
            self.get_logger().error(f'Failed to parse URDF XML for TF publication: {exc}')
            return static_transforms, dynamic_transforms

        now = self.get_clock().now().to_msg()

        for joint in robot.findall('joint'):
            parent_elem = joint.find('parent')
            child_elem = joint.find('child')
            if parent_elem is None or child_elem is None:
                continue

            parent_link = parent_elem.attrib.get('link')
            child_link = child_elem.attrib.get('link')
            if not parent_link or not child_link:
                continue

            origin_elem = joint.find('origin')
            xyz = [0.0, 0.0, 0.0]
            rpy = [0.0, 0.0, 0.0]
            if origin_elem is not None:
                xyz = _parse_xyz_rpy(origin_elem.attrib.get('xyz', ''), xyz)
                rpy = _parse_xyz_rpy(origin_elem.attrib.get('rpy', ''), rpy)

            q_x, q_y, q_z, q_w = _quaternion_from_rpy(rpy[0], rpy[1], rpy[2])

            transform = TransformStamped()
            transform.header.stamp = now
            transform.header.frame_id = parent_link
            transform.child_frame_id = child_link
            transform.transform.translation.x = xyz[0]
            transform.transform.translation.y = xyz[1]
            transform.transform.translation.z = xyz[2]
            transform.transform.rotation.x = q_x
            transform.transform.rotation.y = q_y
            transform.transform.rotation.z = q_z
            transform.transform.rotation.w = q_w

            joint_type = joint.attrib.get('type', 'fixed').lower()
            if joint_type == 'fixed':
                static_transforms.append(transform)
            else:
                dynamic_transforms.append(transform)

        return static_transforms, dynamic_transforms

    def _publish_dynamic_transforms(self):
        if not self.dynamic_transforms:
            return

        now = self.get_clock().now().to_msg()
        for transform in self.dynamic_transforms:
            transform.header.stamp = now

        self.tf_broadcaster.sendTransform(self.dynamic_transforms)


def main(args=None):
    rclpy.init(args=args)
    node = UrdfPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()