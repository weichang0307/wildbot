# A node that subscribes to the camera feed, runs the YOLO model on the images, and publishes the results image.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO

class MyYoloNode(Node):
    def __init__(self):
        super().__init__('my_yolo_node')
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image/compressed',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(CompressedImage, '/yolo/detections', 10)
        self.bridge = CvBridge()
        # load yolov8n model
        self.model = YOLO('/ros2_ws/src/my_yolo/model/bear1.pt')

    def image_callback(self, msg):
        # print("Received image")
        # Convert the ROS Image message to a OpenCV image
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run the YOLO model on the image
        results = self.model(cv_image)

        # Draw the bounding boxes on the image
        annotated_image = results[0].plot()
        # Convert the annotated image back to a ROS Image message
        annotated_msg = self.bridge.cv2_to_compressed_imgmsg(annotated_image)
        # Publish the annotated image
        self.publisher_.publish(annotated_msg)
        # print("Published annotated image")
        
        
def main(args=None):
    rclpy.init(args=args)
    my_yolo_node = MyYoloNode()
    rclpy.spin(my_yolo_node)
    my_yolo_node.destroy_node()
    rclpy.shutdown()