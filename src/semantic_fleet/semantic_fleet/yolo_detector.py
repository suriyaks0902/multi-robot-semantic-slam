#!/usr/bin/env python3
"""
YOLO Object Detector Node
Subscribes to camera images and publishes detected objects using YOLOv8
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from ultralytics import YOLO
import cv2
import numpy as np

# Import custom messages
from semantic_fleet.msg import DetectedObject, DetectedObjects


class YOLODetector(Node):
    """
    YOLO-based object detection node for semantic SLAM.
    
    Subscribes to: /camera/image_raw (sensor_msgs/Image)
    Publishes to:  ~/detected_objects (semantic_fleet/DetectedObjects)
                   ~/detection_visualization (sensor_msgs/Image) [optional]
    """
    
    def __init__(self):
        super().__init__('yolo_detector')
        
        # Declare parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('publish_visualization', True)
        self.declare_parameter('detection_rate', 10.0)  # Hz
        
        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value
        camera_topic = self.get_parameter('camera_topic').value
        self.pub_viz = self.get_parameter('publish_visualization').value
        detection_rate = self.get_parameter('detection_rate').value
        
        # Load YOLO model
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        try:
            self.model = YOLO(model_path)
            self.get_logger().info('✓ YOLO model loaded successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {str(e)}')
            raise
        
        # CV Bridge for ROS-OpenCV conversion
        self.bridge = CvBridge()
        
        # Rate limiting for detection
        self.last_detection_time = self.get_clock().now()
        self.detection_period = 1.0 / detection_rate
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # Publishers
        self.detection_pub = self.create_publisher(
            DetectedObjects,
            '~/detected_objects',
            10
        )
        
        if self.pub_viz:
            self.viz_pub = self.create_publisher(
                Image,
                '~/detection_visualization',
                10
            )
        
        self.get_logger().info(f'YOLO Detector initialized')
        self.get_logger().info(f'  Listening to: {camera_topic}')
        self.get_logger().info(f'  Publishing to: ~/detected_objects')
        self.get_logger().info(f'  Confidence threshold: {self.conf_threshold}')
        self.get_logger().info(f'  Detection rate: {detection_rate} Hz')
        
    def image_callback(self, msg):
        """Process incoming camera images with YOLO detection"""
        
        # Rate limiting
        current_time = self.get_clock().now()
        time_since_last = (current_time - self.last_detection_time).nanoseconds / 1e9
        if time_since_last < self.detection_period:
            return
        self.last_detection_time = current_time
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Run YOLO detection
            results = self.model(cv_image, conf=self.conf_threshold, verbose=False)
            
            # Create detection message
            detection_msg = DetectedObjects()
            detection_msg.header = msg.header
            
            # Extract detections
            for result in results:
                boxes = result.boxes
                if boxes is None or len(boxes) == 0:
                    continue
                    
                for box in boxes:
                    detected_obj = DetectedObject()
                    detected_obj.header = msg.header
                    
                    # Class information
                    detected_obj.class_id = int(box.cls[0])
                    detected_obj.class_name = self.model.names[detected_obj.class_id]
                    detected_obj.confidence = float(box.conf[0])
                    
                    # Bounding box in image coordinates
                    xyxy = box.xyxy[0].cpu().numpy()
                    detected_obj.bbox_2d = xyxy.tolist()
                    
                    # Initialize 3D position (will be filled by localizer node)
                    detected_obj.position_3d.x = 0.0
                    detected_obj.position_3d.y = 0.0
                    detected_obj.position_3d.z = 0.0
                    
                    detection_msg.objects.append(detected_obj)
                    
                    # Draw on visualization image
                    if self.pub_viz:
                        x1, y1, x2, y2 = map(int, xyxy)
                        # Draw bounding box
                        cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        # Draw label
                        label = f'{detected_obj.class_name}: {detected_obj.confidence:.2f}'
                        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                        cv2.rectangle(cv_image, (x1, y1 - label_size[1] - 10), 
                                    (x1 + label_size[0], y1), (0, 255, 0), -1)
                        cv2.putText(cv_image, label, (x1, y1 - 5),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
            
            # Publish detections
            self.detection_pub.publish(detection_msg)
            
            if len(detection_msg.objects) > 0:
                self.get_logger().info(
                    f'Detected {len(detection_msg.objects)} objects: ' +
                    ', '.join([f'{obj.class_name}({obj.confidence:.2f})' 
                              for obj in detection_msg.objects[:5]])  # Show first 5
                )
            
            # Publish visualization
            if self.pub_viz and len(detection_msg.objects) > 0:
                try:
                    viz_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
                    viz_msg.header = msg.header
                    self.viz_pub.publish(viz_msg)
                except CvBridgeError as e:
                    self.get_logger().warn(f'Visualization conversion error: {str(e)}')
                    
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error: {str(e)}')
        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = YOLODetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

