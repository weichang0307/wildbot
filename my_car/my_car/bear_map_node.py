import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import PointCloud, CameraInfo, ChannelFloat32
from geometry_msgs.msg import Point32
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, Empty, String
from tf2_ros import Buffer, TransformListener, TransformException


class BearMapNode(Node):
    def __init__(self):
        super().__init__('bear_map_node')
        self.declare_parameter('input_topic', '/yolo/detections_3d')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('control_state_topic', '/my_control/state')
        self.declare_parameter('output_map_topic', '/bear_map')
        self.declare_parameter('output_marker_topic', '/bear_map_markers')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('source_frame', 'car_base')
        self.declare_parameter('marker_scale', 0.12)
        self.declare_parameter('score_text_scale', 0.10)
        self.declare_parameter('marker_ns', 'bear_map')
        self.declare_parameter('marker_color_r', 0.8)
        self.declare_parameter('marker_color_g', 0.5)
        self.declare_parameter('marker_color_b', 0.1)
        self.declare_parameter('marker_color_a', 0.9)
        self.declare_parameter('merge_distance', 0.25)
        self.declare_parameter('initial_score', 1.0)
        self.declare_parameter('score_increase', 1.0)
        self.declare_parameter('score_decrease', 0.5)
        self.declare_parameter('min_score', 0.2)
        self.declare_parameter('max_score', 3.0)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bear_positions = []
        self.bear_counts = []
        self.bear_scores = []
        self._camera_fx = None
        self._camera_fy = None
        self._camera_cx = None
        self._camera_cy = None
        self._camera_width = None
        self._camera_height = None
        self._camera_info_ready = False
        self._control_state = ''

        self.subscription = self.create_subscription(
            PointCloud,
            self.get_parameter('input_topic').value,
            self.detections_callback,
            10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            10)
        self.control_state_subscription = self.create_subscription(
            String,
            self.get_parameter('control_state_topic').value,
            self.control_state_callback,
            10,
        )
        self.remove_clamped_subscription = self.create_subscription(
            Empty,
            '/bear_map/remove_clamped',
            self.remove_clamped_bear_callback,
            10,
        )
        self.map_pub = self.create_publisher(PointCloud, self.get_parameter('output_map_topic').value, 10)
        self.marker_pub = self.create_publisher(MarkerArray, self.get_parameter('output_marker_topic').value, 10)

    def camera_info_callback(self, msg):
        if len(msg.k) < 6 or msg.k[0] <= 0.0 or msg.k[4] <= 0.0:
            return
        self._camera_fx = float(msg.k[0])
        self._camera_fy = float(msg.k[4])
        self._camera_cx = float(msg.k[2])
        self._camera_cy = float(msg.k[5])
        self._camera_width = int(msg.width)
        self._camera_height = int(msg.height)
        self._camera_info_ready = True

    def control_state_callback(self, msg):
        self._control_state = msg.data.strip()

    def is_finding_state(self):
        if not self._control_state:
            return False
        if self._control_state.endswith(':FINDING'):
            return True
        return self._control_state == 'FINDING'

    def detections_callback(self, msg):
        if not self.is_finding_state():
            return

        target_frame = self.get_parameter('target_frame').value
        source_frame = msg.header.frame_id or self.get_parameter('source_frame').value
        try:
            tf_map_from_src = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except TransformException as e:
            self.get_logger().warn(f"TF error: {e}")
            return

        matched_indices = set()
        for pt in msg.points:
            p_map = self._transform_point(pt, tf_map_from_src)
            new_point = Point32(x=float(p_map[0]), y=float(p_map[1]), z=float(p_map[2]))
            bear_index = self.merge_or_add_bear(new_point)
            matched_indices.add(bear_index)

        self.decrease_scores_for_missed_visible_bears(msg.header.stamp, matched_indices)
        self.remove_low_score_bears()
        self.publish_map(msg.header.stamp, target_frame)

    def merge_or_add_bear(self, new_point):
        merge_distance = float(self.get_parameter('merge_distance').value)
        merge_distance_sq = merge_distance * merge_distance
        score_increase = float(self.get_parameter('score_increase').value)
        max_score = float(self.get_parameter('max_score').value)

        closest_index = -1
        closest_distance_sq = float('inf')
        for index, existing in enumerate(self.bear_positions):
            dx = existing.x - new_point.x
            dy = existing.y - new_point.y
            dz = existing.z - new_point.z
            distance_sq = dx * dx + dy * dy + dz * dz
            if distance_sq <= merge_distance_sq and distance_sq < closest_distance_sq:
                closest_distance_sq = distance_sq
                closest_index = index

        if closest_index < 0:
            self.bear_positions.append(new_point)
            self.bear_counts.append(1)
            initial_score = float(self.get_parameter('initial_score').value)
            self.bear_scores.append(initial_score)
            return len(self.bear_positions) - 1

        count = self.bear_counts[closest_index]
        existing = self.bear_positions[closest_index]
        inv = 1.0 / float(count + 1)
        existing.x = float(existing.x + (new_point.x - existing.x) * inv)
        existing.y = float(existing.y + (new_point.y - existing.y) * inv)
        existing.z = float(existing.z + (new_point.z - existing.z) * inv)
        self.bear_counts[closest_index] = count + 1
        self.bear_scores[closest_index] = min(max_score, self.bear_scores[closest_index] + score_increase)
        return closest_index

    def decrease_scores_for_missed_visible_bears(self, stamp, matched_indices):
        if not self._camera_info_ready:
            return

        target_frame = self.get_parameter('target_frame').value
        camera_frame = self.get_parameter('camera_frame').value
        score_decrease = float(self.get_parameter('score_decrease').value)

        try:
            tf_cam_from_map = self.tf_buffer.lookup_transform(
                camera_frame,
                target_frame,
                stamp,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except TransformException as e:
            self.get_logger().debug(f"Camera FOV TF error: {e}")
            return

        for idx, map_point in enumerate(self.bear_positions):
            if idx in matched_indices:
                # print('matched')
                continue
            p_cam = self._transform_point(map_point, tf_cam_from_map)
            if not self._is_in_camera_fov(p_cam):
                continue
            self.bear_scores[idx] -= score_decrease

    def remove_low_score_bears(self):
        min_score = float(self.get_parameter('min_score').value)
        kept_positions = []
        kept_counts = []
        kept_scores = []
        for idx, score in enumerate(self.bear_scores):
            if score < min_score:
                continue
            kept_positions.append(self.bear_positions[idx])
            kept_counts.append(self.bear_counts[idx])
            kept_scores.append(score)
        self.bear_positions = kept_positions
        self.bear_counts = kept_counts
        self.bear_scores = kept_scores

    def remove_clamped_bear_callback(self, _msg):
        if not self.bear_positions:
            return

        target_frame = self.get_parameter('target_frame').value
        source_frame = self.get_parameter('source_frame').value
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.2),
            )
        except TransformException as exc:
            self.get_logger().warn(f'Cannot remove clamped bear without TF: {exc}')
            return

        robot_x = transform.transform.translation.x
        robot_y = transform.transform.translation.y
        remove_index = min(
            range(len(self.bear_positions)),
            key=lambda idx: math.hypot(
                self.bear_positions[idx].x - robot_x,
                self.bear_positions[idx].y - robot_y,
            ),
        )

        self.bear_positions.pop(remove_index)
        self.bear_counts.pop(remove_index)
        self.bear_scores.pop(remove_index)
        self.publish_map(self.get_clock().now().to_msg(), target_frame)

    def _is_in_camera_fov(self, p_cam):
        if p_cam[2] <= 0.0:
            return False
        u = self._camera_fx * (p_cam[0] / p_cam[2]) + self._camera_cx
        v = self._camera_fy * (p_cam[1] / p_cam[2]) + self._camera_cy
        x_start = 0.25 * float(self._camera_width)
        x_end = 0.75 * float(self._camera_width)
        y_start = 0.25 * float(self._camera_height)
        y_end = 0.75 * float(self._camera_height)
        return x_start <= u < x_end and y_start <= v < y_end and p_cam[2] < 0.8

    @staticmethod
    def _transform_point(point, transform_msg):
        t = transform_msg.transform.translation
        q = transform_msg.transform.rotation
        rotated = BearMapNode._rotate_vector((float(point.x), float(point.y), float(point.z)), q)
        return (
            rotated[0] + t.x,
            rotated[1] + t.y,
            rotated[2] + t.z,
        )

    @staticmethod
    def _rotate_vector(vector, quaternion):
        qx = quaternion.x
        qy = quaternion.y
        qz = quaternion.z
        qw = quaternion.w

        tx = 2.0 * (qy * vector[2] - qz * vector[1])
        ty = 2.0 * (qz * vector[0] - qx * vector[2])
        tz = 2.0 * (qx * vector[1] - qy * vector[0])

        return (
            vector[0] + qw * tx + (qy * tz - qz * ty),
            vector[1] + qw * ty + (qz * tx - qx * tz),
            vector[2] + qw * tz + (qx * ty - qy * tx),
        )

    def publish_map(self, stamp, frame):
        pc = PointCloud()
        pc.header = Header()
        pc.header.stamp = stamp
        pc.header.frame_id = frame
        pc.points = self.bear_positions

        score_channel = ChannelFloat32()
        score_channel.name = 'score'
        score_channel.values = [float(score) for score in self.bear_scores]
        pc.channels.append(score_channel)

        self.map_pub.publish(pc)

        marker_array = MarkerArray()
        delete_all = Marker()
        delete_all.header.frame_id = frame
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        marker_array.markers.append(delete_all)

        marker_scale = float(self.get_parameter('marker_scale').value)
        text_scale = float(self.get_parameter('score_text_scale').value)
        ns = self.get_parameter('marker_ns').value
        color_r = float(self.get_parameter('marker_color_r').value)
        color_g = float(self.get_parameter('marker_color_g').value)
        color_b = float(self.get_parameter('marker_color_b').value)
        color_a = float(self.get_parameter('marker_color_a').value)
        max_score = max(float(self.get_parameter('max_score').value), 1e-6)

        for i, pt in enumerate(self.bear_positions):
            score = self.bear_scores[i]
            score_norm = max(0.0, min(1.0, score / max_score))

            marker = Marker()
            marker.header.frame_id = frame
            marker.header.stamp = stamp
            marker.ns = ns
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = pt.x
            marker.pose.position.y = pt.y
            marker.pose.position.z = pt.z
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker_scale
            marker.scale.y = marker_scale
            marker.scale.z = marker_scale
            marker.color.r = color_r * (1.0 - score_norm)
            marker.color.g = color_g * score_norm
            marker.color.b = color_b
            marker.color.a = color_a
            marker_array.markers.append(marker)

            text_marker = Marker()
            text_marker.header.frame_id = frame
            text_marker.header.stamp = stamp
            text_marker.ns = f'{ns}_score'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = pt.x
            text_marker.pose.position.y = pt.y
            text_marker.pose.position.z = pt.z + marker_scale
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = text_scale
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 0.95
            text_marker.text = f'{score:.2f}'
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = BearMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
