#!/usr/bin/env python3
"""
Occupancy Map Merger Node

Merges multiple robots' occupancy grid maps into a single unified map.
Transforms each robot's map to a common frame and overlays them.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose, Point, Quaternion
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
from typing import Dict, List, Optional


class OccupancyMapMerger(Node):
    """
    Merges occupancy grid maps from multiple robots into a unified global map.
    """
    
    def __init__(self):
        super().__init__('occupancy_map_merger')
        
        # Parameters
        self.declare_parameter('robot_namespaces', ['robot_1', 'robot_2'])
        self.declare_parameter('global_frame', 'world')
        self.declare_parameter('merged_map_topic', '/merged_map')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.declare_parameter('resolution', 0.05)  # meters per cell
        self.declare_parameter('width', 400)  # cells
        self.declare_parameter('height', 400)  # cells
        self.declare_parameter('origin_x', -10.0)  # meters
        self.declare_parameter('origin_y', -10.0)  # meters
        
        self.robot_namespaces = self.get_parameter('robot_namespaces').value
        self.global_frame = self.get_parameter('global_frame').value
        self.merged_map_topic = self.get_parameter('merged_map_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.resolution = self.get_parameter('resolution').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        
        self.get_logger().info(f'Occupancy Map Merger initialized')
        self.get_logger().info(f'  Global frame: {self.global_frame}')
        self.get_logger().info(f'  Merging maps from: {self.robot_namespaces}')
        self.get_logger().info(f'  Resolution: {self.resolution}m/cell')
        self.get_logger().info(f'  Map size: {self.width}x{self.height} cells')
        
        # TF buffer
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Storage for latest maps from each robot
        self.robot_maps: Dict[str, OccupancyGrid] = {}
        
        # Merged map
        self.merged_map = self._create_empty_map()
        
        # Publisher
        self.merged_map_pub = self.create_publisher(
            OccupancyGrid, 
            self.merged_map_topic, 
            10
        )
        
        # Subscribers for each robot
        self._subscriptions = []
        for namespace in self.robot_namespaces:
            topic = f'/{namespace}/map'
            sub = self.create_subscription(
                OccupancyGrid,
                topic,
                lambda msg, ns=namespace: self.map_callback(ns, msg),
                10
            )
            self._subscriptions.append(sub)
            self.get_logger().info(f'Subscribing to map: {topic}')
        
        # Timer to publish merged map
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.merge_and_publish
        )
        
        self.get_logger().info('Occupancy Map Merger ready!')
    
    def _create_empty_map(self) -> OccupancyGrid:
        """Create an empty occupancy grid map."""
        msg = OccupancyGrid()
        msg.header.frame_id = self.global_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        
        # Set origin
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # Initialize with unknown (-1)
        msg.data = [-1] * (self.width * self.height)
        
        return msg
    
    def map_callback(self, namespace: str, msg: OccupancyGrid):
        """Store the latest map from each robot."""
        self.robot_maps[namespace] = msg
        self.get_logger().info(
            f'✅ Received map from {namespace}: '
            f'{msg.info.width}x{msg.info.height} @ {msg.info.resolution}m/cell, '
            f'frame: {msg.header.frame_id}',
            throttle_duration_sec=5.0
        )
    
    def merge_and_publish(self):
        """Merge all robot maps and publish the result."""
        if not self.robot_maps:
            self.get_logger().warn(
                f'No robot maps received yet. Waiting for maps from: {self.robot_namespaces}',
                throttle_duration_sec=10.0
            )
            return
        
        # Create fresh merged map
        merged = self._create_empty_map()
        merged.header.stamp = self.get_clock().now().to_msg()
        
        # Initialize arrays for merging
        # Use numpy for efficient merging
        # -1 = unknown, 0 = free, 100 = occupied
        merged_data = np.full((self.height, self.width), -1, dtype=np.int8)
        observation_count = np.zeros((self.height, self.width), dtype=np.int32)
        
        # Process each robot's map
        for namespace, robot_map in self.robot_maps.items():
            try:
                # Transform robot map to global frame
                self._merge_robot_map(
                    namespace,
                    robot_map,
                    merged_data,
                    observation_count
                )
            except Exception as e:
                self.get_logger().warn(
                    f'Failed to merge map from {namespace}: {e}',
                    throttle_duration_sec=5.0
                )
        
        # Average multiple observations
        # If a cell was observed by multiple robots, take average
        mask = observation_count > 0
        merged_data[mask] = merged_data[mask] // observation_count[mask]
        
        # Convert back to flat list
        merged.data = merged_data.flatten().tolist()
        
        # Publish
        self.merged_map_pub.publish(merged)
        self.get_logger().info(
            f'📊 Published merged map from {len(self.robot_maps)} robots',
            throttle_duration_sec=5.0
        )
    
    def _merge_robot_map(
        self,
        namespace: str,
        robot_map: OccupancyGrid,
        merged_data: np.ndarray,
        observation_count: np.ndarray
    ):
        """
        Merge a single robot's map into the global merged map.
        """
        # Get transform from robot map frame to global frame
        source_frame = robot_map.header.frame_id
        if not source_frame.startswith('/'):
            # Add namespace if needed
            if '/' not in source_frame:
                source_frame = f'{namespace}/{source_frame}'
        
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                source_frame,
                robot_map.header.stamp,
                timeout=Duration(seconds=0.5)
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup failed for {source_frame} -> {self.global_frame}: {e}',
                throttle_duration_sec=5.0
            )
            return
        
        # Extract transform parameters
        tx = transform.transform.translation.x
        ty = transform.transform.translation.y
        
        # Convert quaternion to yaw
        q = transform.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        
        # Robot map parameters
        robot_res = robot_map.info.resolution
        robot_width = robot_map.info.width
        robot_height = robot_map.info.height
        robot_origin_x = robot_map.info.origin.position.x
        robot_origin_y = robot_map.info.origin.position.y
        
        # Iterate through robot map cells
        robot_data = np.array(robot_map.data).reshape((robot_height, robot_width))
        
        for robot_row in range(robot_height):
            for robot_col in range(robot_width):
                value = robot_data[robot_row, robot_col]
                
                # Skip unknown cells
                if value == -1:
                    continue
                
                # Convert robot cell to world coordinates
                robot_x = robot_origin_x + (robot_col + 0.5) * robot_res
                robot_y = robot_origin_y + (robot_row + 0.5) * robot_res
                
                # Transform to global frame
                global_x = tx + robot_x * math.cos(yaw) - robot_y * math.sin(yaw)
                global_y = ty + robot_x * math.sin(yaw) + robot_y * math.cos(yaw)
                
                # Convert to merged map cell coordinates
                merged_col = int((global_x - self.origin_x) / self.resolution)
                merged_row = int((global_y - self.origin_y) / self.resolution)
                
                # Check bounds
                if 0 <= merged_row < self.height and 0 <= merged_col < self.width:
                    # Accumulate value (will average later)
                    if merged_data[merged_row, merged_col] == -1:
                        merged_data[merged_row, merged_col] = value
                    else:
                        merged_data[merged_row, merged_col] += value
                    
                    observation_count[merged_row, merged_col] += 1


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OccupancyMapMerger()
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

