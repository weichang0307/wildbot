import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class MyYoloNode(Node):
    def __init__(self):
        super().__init__('my_yolo_node')
        
        # 1. Image Subscription
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10)
            
        # 2. Publishers: One for visuals, one for data
        self.image_publisher = self.create_publisher(CompressedImage, '/yolo/detections_image', 10)
        self.bbox_publisher = self.create_publisher(Detection2DArray, '/yolo/detections_data', 10)
        
        self.bridge = CvBridge()
        
        # 3. Load YOLO model
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO('/ros2_ws/my_car/model/best.pt')
        self.get_logger().info("YOLO model loaded successfully!")

    def image_callback(self, msg):
        # Convert the ROS Image message to an OpenCV image
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
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
        
        # Loop through every detected bear in the frame
        for box in results[0].boxes:
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
            
            # Attach hypothesis to the detection
            detection.results.append(hypothesis)
            
            # Add the single detection to our array of all detections
            detection_array.detections.append(detection)

        # Publish the array of bounding boxes
        self.bbox_publisher.publish(detection_array)


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