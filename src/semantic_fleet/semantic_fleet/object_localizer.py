#!/usr/bin/env python3
"""
Object Localizer Node
Transforms detected objects from camera frame to world/map frame using TF2
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_geometry_msgs

# Import custom messages
from semantic_fleet.msg import DetectedObjects


class ObjectLocalizer(Node):
    """
    Transforms detected objects from camera frame to world frame.
    
    Subscribes to: /yolo_detector/detected_objects (DetectedObjects)
    Publishes to:  ~/localized_objects (DetectedObjects)
    """
    
    def __init__(self):
        super().__init__('object_localizer')
        
        # Declare parameters
        self.declare_parameter('camera_frame', 'camera_rgb_optical_frame')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('assumed_object_distance', 1.5)  # meters
        self.declare_parameter('camera_fov_h', 1.085)  # radians (~62 degrees)
        self.declare_parameter('camera_fov_v', 0.785)  # radians (~45 degrees)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        
        # Get parameters
        self.camera_frame = self.get_parameter('camera_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.default_distance = self.get_parameter('assumed_object_distance').value
        self.fov_h = self.get_parameter('camera_fov_h').value
        self.fov_v = self.get_parameter('camera_fov_v').value
        self.img_width = self.get_parameter('image_width').value
        self.img_height = self.get_parameter('image_height').value
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            DetectedObjects,
            '/yolo_detector/detected_objects',
            self.detection_callback,
            10
        )
        
        # Publishers
        self.localized_pub = self.create_publisher(
            DetectedObjects,
            '~/localized_objects',
            10
        )
        
        self.get_logger().info('Object Localizer initialized')
        self.get_logger().info(f'  Camera frame: {self.camera_frame}')
        self.get_logger().info(f'  World frame: {self.world_frame}')
        self.get_logger().info(f'  Default distance: {self.default_distance}m')
        
    def detection_callback(self, msg):
        """Transform detected objects from camera frame to world frame"""
        
        if len(msg.objects) == 0:
            return
        
        try:
            # Get transform from camera to world frame
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time(),  # Get latest available transform
                timeout=Duration(seconds=1.0)
            )
            
            # Create localized message
            localized_msg = DetectedObjects()
            localized_msg.header.frame_id = self.world_frame
            localized_msg.header.stamp = self.get_clock().now().to_msg()
            
            for obj in msg.objects:
                # Estimate 3D position from 2D bounding box center
                bbox = obj.bbox_2d
                if len(bbox) != 4:
                    continue
                
                # Calculate center of bounding box
                center_x = (bbox[0] + bbox[2]) / 2.0
                center_y = (bbox[1] + bbox[3]) / 2.0
                
                # Convert pixel coordinates to camera angles
                # Normalized coordinates (-0.5 to 0.5)
                norm_x = (center_x - self.img_width / 2.0) / self.img_width
                norm_y = (center_y - self.img_height / 2.0) / self.img_height
                
                # Calculate angles
                angle_h = norm_x * self.fov_h
                angle_v = norm_y * self.fov_v
                
                # Create point in camera frame
                # Assume object is at default_distance in front of camera
                point_camera = PointStamped()
                point_camera.header.frame_id = self.camera_frame
                point_camera.header.stamp = msg.header.stamp
                
                # Calculate 3D position (camera looks along +X axis in optical frame)
                point_camera.point.x = self.default_distance
                point_camera.point.y = -self.default_distance * angle_h  # Left/right
                point_camera.point.z = -self.default_distance * angle_v  # Up/down
                
                # Transform to world frame
                try:
                    point_world = tf2_geometry_msgs.do_transform_point(point_camera, transform)
                    
                    # Update object with 3D position
                    obj.position_3d = point_world.point
                    localized_msg.objects.append(obj)
                    
                except Exception as e:
                    self.get_logger().warn(f'Failed to transform point: {str(e)}')
                    continue
            
            # Publish localized objects
            if len(localized_msg.objects) > 0:
                self.localized_pub.publish(localized_msg)
                self.get_logger().info(
                    f'Localized {len(localized_msg.objects)} objects in {self.world_frame} frame',
                    throttle_duration_sec=2.0
                )
            
        except (LookupException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF transform failed: {str(e)}',
                throttle_duration_sec=5.0
            )
        except Exception as e:
            self.get_logger().error(f'Error in detection_callback: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObjectLocalizer()
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

