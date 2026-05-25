import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import Point32
from sensor_msgs.msg import Image, CompressedImage, PointCloud, CameraInfo
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from ultralytics import YOLO


class MyYoloNode(Node):
    def __init__(self):
        super().__init__('my_yolo_node')

        self.declare_parameter('camera_fx', 615.0)
        self.declare_parameter('camera_fy', 615.0)
        self.declare_parameter('camera_cx', 320.0)
        self.declare_parameter('camera_cy', 240.0)
        self.declare_parameter('camera_frame', 'camera')
        self.declare_parameter('target_frame', 'car_base')
        self.declare_parameter('projection_plane_z', 0.05)
        self.declare_parameter('marker_scale', 0.08)
        self._image_width = None
        self._image_height = None
        self._last_image_frame_id = ''
        self._last_image_stamp = Time()
        self._camera_fx = None
        self._camera_fy = None
        self._camera_cx = None
        self._camera_cy = None
        self._camera_frame_id = ''
        self._warned_missing_camera_info = False
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 1. Image Subscription
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10)
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10)
            
        # 2. Publishers: One for visuals, one for data
        self.image_publisher = self.create_publisher(CompressedImage, '/yolo/detections_image', 10)
        self.bbox_publisher = self.create_publisher(Detection2DArray, '/yolo/detections_data', 10)
        self.point_array_publisher = self.create_publisher(PointCloud, '/yolo/detections_3d', 10)
        self.marker_array_publisher = self.create_publisher(MarkerArray, '/yolo/detections_markers', 10)
        
        self.bridge = CvBridge()
        
        # 3. Load YOLO model
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO('/ros2_ws/my_car/model/best.pt')
        
        # Force YOLO to use the AMD GPU (ROCm maps this to 'cuda')
        import torch
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model.to(device)
        
        self.get_logger().info("YOLO model loaded successfully!")

        # show the device
        self.get_logger().info(f"YOLO model is running on device: {self.model.device}")

    def camera_info_callback(self, msg):
        if len(msg.k) >= 6 and msg.k[0] > 0.0 and msg.k[4] > 0.0:
            self._camera_fx = float(msg.k[0])
            self._camera_fy = float(msg.k[4])
            self._camera_cx = float(msg.k[2])
            self._camera_cy = float(msg.k[5])
            if msg.header.frame_id:
                self._camera_frame_id = msg.header.frame_id
            self._warned_missing_camera_info = False


    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._image_height, self._image_width = cv_image.shape[:2]
        self._last_image_frame_id = msg.header.frame_id
        self._last_image_stamp = Time.from_msg(msg.header.stamp)
        
        # Run the YOLO model on the image silently
        results = self.model(cv_image, verbose=False)

        # --- PART A: Publish the Visual Annotated Image ---
        annotated_image = results[0].plot()
        annotated_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_image)
        annotated_msg.header = msg.header # Keep the timestamp consistent
        self.image_publisher.publish(annotated_msg)

        # --- PART B: Publish the Bounding Box Data for Unity ---
        detection_array = Detection2DArray()
        detection_array.header = msg.header # Sync data to the exact video frame
        target_frame = str(self.get_parameter('target_frame').value)

        point_array_msg = PointCloud()
        point_array_msg.header = msg.header
        point_array_msg.header.frame_id = target_frame

        marker_array_msg = MarkerArray()
        delete_all_marker = Marker()
        delete_all_marker.header = point_array_msg.header
        delete_all_marker.action = Marker.DELETEALL
        marker_array_msg.markers.append(delete_all_marker)
        marker_scale = float(self.get_parameter('marker_scale').value)
        
        # Loop through every detected bear in the frame
        for index, box in enumerate(results[0].boxes):
            # if the confidence is too low, skip this detection
            confidence = float(box.conf[0])
            if confidence < 0.5:
                continue

            detection = Detection2D()
            
            # 1. Extract geometry from YOLO (x_center, y_center, width, height)
            x, y, w, h = box.xywh[0].tolist()
            
            # Map to ROS BoundingBox2D
            detection.bbox.center.position.x = float(x)
            detection.bbox.center.position.y = float(y)
            detection.bbox.size_x = float(w)
            detection.bbox.size_y = float(h)
            
            # 2. Extract class ID and confidence score
            class_id = str(int(box.cls[0]))
            confidence = float(box.conf[0])
            
            # Map to ROS ObjectHypothesis
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_id
            hypothesis.hypothesis.score = confidence

            try:
                point_3d = self.project_to_3d(detection.bbox)
            except (TransformException, ValueError) as e:
                point_3d = None
                self.get_logger().debug(f'Error projecting to 3D: {e}')

            if point_3d is not None:
                point = Point32()
                point.x = point_3d['x']
                point.y = point_3d['y']
                point.z = point_3d['z']
                point_array_msg.points.append(point)

                marker = Marker()
                marker.header = point_array_msg.header
                marker.ns = 'yolo_detections_3d'
                marker.id = index
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = point_3d['x']
                marker.pose.position.y = point_3d['y']
                marker.pose.position.z = point_3d['z']
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker_scale
                marker.scale.y = marker_scale
                marker.scale.z = marker_scale
                marker.color.r = 1.0 - confidence
                marker.color.g = confidence
                marker.color.b = 0.1
                marker.color.a = 0.9
                marker_array_msg.markers.append(marker)
            
            # Attach hypothesis to the detection
            detection.results.append(hypothesis)
            
            # Add the single detection to our array of all detections
            detection_array.detections.append(detection)

        # Publish the array of bounding boxes
        self.bbox_publisher.publish(detection_array)
        self.point_array_publisher.publish(point_array_msg)
        self.marker_array_publisher.publish(marker_array_msg)

    def project_to_3d(self, bbox):
        center_x = float(bbox.center.position.x)
        center_y = float(bbox.center.position.y)

        fx = self._camera_fx if self._camera_fx is not None else float(self.get_parameter('camera_fx').value)
        fy = self._camera_fy if self._camera_fy is not None else float(self.get_parameter('camera_fy').value)
        cx = self._camera_cx if self._camera_cx is not None else float(self.get_parameter('camera_cx').value)
        cy = self._camera_cy if self._camera_cy is not None else float(self.get_parameter('camera_cy').value)
        plane_z = float(self.get_parameter('projection_plane_z').value)
        target_frame = str(self.get_parameter('target_frame').value)
        source_frame = (
            self._camera_frame_id
            or self._last_image_frame_id
            or str(self.get_parameter('camera_frame').value)
        )

        if self._camera_fx is None and not self._warned_missing_camera_info:
            self.get_logger().warn(
                'No CameraInfo received yet; using camera_* parameter intrinsics for projection.'
            )
            self._warned_missing_camera_info = True

        if self._image_width is not None and self._image_height is not None:
            if cx == 320.0:
                cx = self._image_width / 2.0
            if cy == 240.0:
                cy = self._image_height / 2.0

        if fx <= 0.0 or fy <= 0.0:
            raise ValueError('camera_fx and camera_fy must be positive')
        if not source_frame:
            raise ValueError('No source camera frame is available for 3D projection')

        ray_camera = (
            (center_x - cx) / fx,
            (center_y - cy) / fy,
            1.0,
        )

        transform = self.tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            self._last_image_stamp,
        )

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        ray_base = self._rotate_vector(ray_camera, rotation)

        # print(ray_base)

        if abs(ray_base[2]) < 1e-9:
            raise ValueError('Projection ray is parallel to the z=5 plane')

        scale = (plane_z - translation.z) / ray_base[2]
        if scale < 0.0:
            raise ValueError('Projection plane lies behind the camera ray')
        
        x = translation.x + ray_base[0] * scale
        y = translation.y + ray_base[1] * scale
        z = plane_z

        dis = (x - translation.x)**2 + (y - translation.y)**2 + (z - translation.z)**2
        bear_size = 0.11
        theoretical_size_x = bear_size * fx / (dis**0.5)
        theoretical_size_y = bear_size * fy / (dis**0.5)
        size_ratio_x = theoretical_size_x / bbox.size_x 
        size_ratio_y = theoretical_size_y / bbox.size_y
        # print(f'size_ratio_x={size_ratio_x:.2f}, size_ratio_y={size_ratio_y:.2f}')
        if size_ratio_x < 0.8 or size_ratio_x > 1.5 or size_ratio_y < 0.8 or size_ratio_y > 1.5:
            # print(f'Warning: Detected box size is inconsistent with expected bear size at this distance (size_ratio_x={size_ratio_x:.2f}, size_ratio_y={size_ratio_y:.2f}), fx={fx}, fy={fy}, bbox.size_x={bbox.size_x}, bbox.size_y={bbox.size_y}, theoretical_size_x={theoretical_size_x}, theoretical_size_y={theoretical_size_y}')
            raise ValueError('Projected size is unrealistic for a bear at this distance')

        return {
            'x': translation.x + ray_base[0] * scale,
            'y': translation.y + ray_base[1] * scale,
            'z': plane_z,
        }

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

def main(args=None):
    rclpy.init(args=args)
    node = MyYoloNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()