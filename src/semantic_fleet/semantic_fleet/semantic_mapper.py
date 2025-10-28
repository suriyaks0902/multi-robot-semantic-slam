#!/usr/bin/env python3
"""
Semantic Mapper Node
Maintains a semantic map of detected objects in the world frame
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import numpy as np
import uuid

# Import custom messages
from semantic_fleet.msg import DetectedObjects, SemanticMap


class SemanticMapper(Node):
    """
    Maintains a semantic map by accumulating detected objects.
    Handles data association to avoid duplicates.
    
    Subscribes to: /object_localizer/localized_objects (DetectedObjects)
    Publishes to:  ~/semantic_map (SemanticMap)
    """
    
    def __init__(self):
        super().__init__('semantic_mapper')
        
        # Declare parameters
        self.declare_parameter('matching_distance_threshold', 0.5)  # meters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('min_observations', 2)  # Minimum observations before adding to map
        self.declare_parameter('confidence_decay', 0.95)  # Per update cycle
        self.declare_parameter('min_confidence', 0.3)  # Remove objects below this
        
        # Get parameters
        self.match_threshold = self.get_parameter('matching_distance_threshold').value
        update_rate = self.get_parameter('update_rate').value
        self.min_obs = self.get_parameter('min_observations').value
        self.conf_decay = self.get_parameter('confidence_decay').value
        self.min_conf = self.get_parameter('min_confidence').value
        
        # Semantic map storage
        # Format: {object_id: {'class': str, 'position': Point, 
        #                      'confidence': float, 'observations': int}}
        self.semantic_map = {}
        
        # Temporary detections (need multiple observations)
        self.temp_detections = {}
        
        # Subscribers
        self.detection_sub = self.create_subscription(
            DetectedObjects,
            '/object_localizer/localized_objects',
            self.detection_callback,
            10
        )
        
        # Publishers
        self.map_pub = self.create_publisher(
            SemanticMap,
            '~/semantic_map',
            10
        )
        
        # Timer for periodic map publishing and maintenance
        self.timer = self.create_timer(1.0 / update_rate, self.map_maintenance)
        
        self.get_logger().info('Semantic Mapper initialized')
        self.get_logger().info(f'  Matching threshold: {self.match_threshold}m')
        self.get_logger().info(f'  Minimum observations: {self.min_obs}')
        self.get_logger().info(f'  Update rate: {update_rate} Hz')
        
    def detection_callback(self, msg):
        """Update semantic map with new detections"""
        
        for obj in msg.objects:
            # Find matching object in map or temp detections
            matched_id = self.find_matching_object(obj)
            
            if matched_id:
                # Update existing object
                if matched_id in self.semantic_map:
                    self.update_object(matched_id, obj, in_map=True)
                elif matched_id in self.temp_detections:
                    self.update_object(matched_id, obj, in_map=False)
            else:
                # Add to temporary detections
                self.add_temp_detection(obj)
        
        # Promote temp detections to map if they have enough observations
        self.promote_temp_detections()
    
    def find_matching_object(self, detection):
        """Find existing object that matches the detection"""
        best_match_id = None
        best_distance = float('inf')
        
        # Check semantic map
        for obj_id, map_obj in self.semantic_map.items():
            if map_obj['class'] != detection.class_name:
                continue
            
            distance = self.calculate_distance(map_obj['position'], detection.position_3d)
            if distance < self.match_threshold and distance < best_distance:
                best_match_id = obj_id
                best_distance = distance
        
        # Check temp detections if no map match
        if best_match_id is None:
            for obj_id, temp_obj in self.temp_detections.items():
                if temp_obj['class'] != detection.class_name:
                    continue
                
                distance = self.calculate_distance(temp_obj['position'], detection.position_3d)
                if distance < self.match_threshold and distance < best_distance:
                    best_match_id = obj_id
                    best_distance = distance
        
        return best_match_id
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two points"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return np.sqrt(dx**2 + dy**2 + dz**2)
    
    def update_object(self, obj_id, detection, in_map=True):
        """Update existing object with new observation"""
        storage = self.semantic_map if in_map else self.temp_detections
        obj = storage[obj_id]
        
        # Weighted average of position (more observations = more weight on existing)
        n = obj['observations']
        obj['position'].x = (obj['position'].x * n + detection.position_3d.x) / (n + 1)
        obj['position'].y = (obj['position'].y * n + detection.position_3d.y) / (n + 1)
        obj['position'].z = (obj['position'].z * n + detection.position_3d.z) / (n + 1)
        
        # Update confidence (weighted average with slight bias toward new)
        obj['confidence'] = (obj['confidence'] * 0.7 + detection.confidence * 0.3)
        
        obj['observations'] += 1
        obj['last_seen'] = self.get_clock().now()
    
    def add_temp_detection(self, detection):
        """Add new object to temporary detections"""
        obj_id = str(uuid.uuid4())[:8]
        
        self.temp_detections[obj_id] = {
            'class': detection.class_name,
            'position': Point(
                x=detection.position_3d.x,
                y=detection.position_3d.y,
                z=detection.position_3d.z
            ),
            'confidence': detection.confidence,
            'observations': 1,
            'last_seen': self.get_clock().now()
        }
    
    def promote_temp_detections(self):
        """Move temp detections with enough observations to main map"""
        to_promote = []
        
        for obj_id, obj in self.temp_detections.items():
            if obj['observations'] >= self.min_obs:
                to_promote.append(obj_id)
        
        for obj_id in to_promote:
            self.semantic_map[obj_id] = self.temp_detections.pop(obj_id)
            self.get_logger().info(
                f'Added {self.semantic_map[obj_id]["class"]} to map at '
                f'({self.semantic_map[obj_id]["position"].x:.2f}, '
                f'{self.semantic_map[obj_id]["position"].y:.2f})'
            )
    
    def map_maintenance(self):
        """Periodic map maintenance and publishing"""
        current_time = self.get_clock().now()
        
        # Decay confidence for objects not recently seen
        to_remove = []
        for obj_id, obj in self.semantic_map.items():
            time_since_seen = (current_time - obj['last_seen']).nanoseconds / 1e9
            if time_since_seen > 5.0:  # 5 seconds
                obj['confidence'] *= self.conf_decay
                if obj['confidence'] < self.min_conf:
                    to_remove.append(obj_id)
        
        # Remove low confidence objects
        for obj_id in to_remove:
            removed_obj = self.semantic_map.pop(obj_id)
            self.get_logger().info(f'Removed {removed_obj["class"]} from map (low confidence)')
        
        # Clean up old temp detections
        temp_to_remove = []
        for obj_id, obj in self.temp_detections.items():
            time_since_seen = (current_time - obj['last_seen']).nanoseconds / 1e9
            if time_since_seen > 10.0:  # 10 seconds
                temp_to_remove.append(obj_id)
        
        for obj_id in temp_to_remove:
            self.temp_detections.pop(obj_id)
        
        # Publish semantic map
        self.publish_map()
    
    def publish_map(self):
        """Publish current semantic map"""
        if not self.semantic_map:
            return
        
        map_msg = SemanticMap()
        map_msg.header.frame_id = 'map'
        map_msg.header.stamp = self.get_clock().now().to_msg()
        
        for obj_id, obj in self.semantic_map.items():
            map_msg.object_ids.append(obj_id)
            map_msg.class_names.append(obj['class'])
            map_msg.positions.append(obj['position'])
            map_msg.confidences.append(obj['confidence'])
        
        self.map_pub.publish(map_msg)
        
        self.get_logger().info(
            f'Semantic map: {len(self.semantic_map)} objects, '
            f'{len(self.temp_detections)} pending',
            throttle_duration_sec=5.0
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SemanticMapper()
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

