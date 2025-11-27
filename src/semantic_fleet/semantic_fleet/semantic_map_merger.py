#!/usr/bin/env python3
"""
Semantic Map Merger

Fuses per-robot semantic maps into a single global semantic map using TF
transforms to project each robot's map into a shared frame.
"""

import math
import uuid
from typing import Dict

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Point, PointStamped
from semantic_fleet.msg import SemanticMap
from tf2_geometry_msgs import do_transform_point
from tf2_ros import (
    Buffer,
    ConnectivityException,
    LookupException,
    TransformListener,
    ExtrapolationException,
)


class SemanticMapMerger(Node):
    """
    Merge semantic maps published by multiple robots into a single global map.
    """

    def __init__(self):
        super().__init__('semantic_map_merger')

        # Parameters
        if not self.has_parameter('robot_namespaces'):
            self.declare_parameter(
                'robot_namespaces',
                descriptor=ParameterDescriptor(dynamic_typing=True),
            )
        self.declare_parameter('semantic_map_topic', 'semantic_mapper/semantic_map')
        self.declare_parameter('global_map_topic', '/global_semantic_map')
        self.declare_parameter('global_frame', 'world')
        self.declare_parameter('matching_distance_threshold', 0.75)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('stale_object_timeout', 10.0)
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        self.robot_namespaces = list(self.get_parameter('robot_namespaces').value or [])
        self.semantic_map_topic = self.get_parameter('semantic_map_topic').value
        self.global_map_topic = self.get_parameter('global_map_topic').value
        self.global_frame = self.get_parameter('global_frame').value
        self.match_threshold = float(
            self.get_parameter('matching_distance_threshold').value
        )
        publish_rate = float(self.get_parameter('publish_rate').value)
        self.stale_timeout = float(self.get_parameter('stale_object_timeout').value)

        if not self.robot_namespaces:
            self.get_logger().warning(
                'Parameter "robot_namespaces" is empty. No maps will be merged until it is set.'
            )

        # TF buffer/listener
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Storage for global objects
        self.global_objects: Dict[str, Dict] = {}

        # Publishers / timers
        self.global_map_pub = self.create_publisher(
            SemanticMap, self.global_map_topic, 10
        )
        period = 1.0 / publish_rate if publish_rate > 0.0 else 1.0
        self.publish_timer = self.create_timer(period, self.publish_global_map)

        # Create subscriptions for each robot namespace
        self._subscriptions = []
        for namespace in self.robot_namespaces:
            topic = self._resolve_map_topic(namespace)
            sub = self.create_subscription(
                SemanticMap,
                topic,
                lambda msg, ns=namespace: self.semantic_map_callback(ns, msg),
                10,
            )
            self._subscriptions.append(sub)
            self.get_logger().info(
                f'Subscribing to semantic map "{topic}" for robot "{namespace}"'
            )

        self.get_logger().info(
            f'Publishing merged semantic map to "{self.global_map_topic}" in frame "{self.global_frame}"'
        )

    # ------------------------------------------------------------------ #
    # Subscription handling
    # ------------------------------------------------------------------ #

    def semantic_map_callback(self, namespace: str, msg: SemanticMap) -> None:
        """Process a semantic map update from a robot."""
        if not msg.object_ids:
            return

        stamp = (
            Time.from_msg(msg.header.stamp)
            if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0
            else self.get_clock().now()
        )

        source_frame = self._resolve_frame(namespace, msg.header.frame_id)

        # Attempt to transform from the robot's map frame into the global frame
        try:
            transform = self.tf_buffer.lookup_transform(
                self.global_frame,
                source_frame,
                msg.header.stamp,
                timeout=Duration(seconds=0.5),
            )
        except (LookupException, ConnectivityException, ExtrapolationException) as exc:
            # Fall back to using the raw coordinates if TF is unavailable
            self.get_logger().warn(
                f'Unable to transform from "{source_frame}" to "{self.global_frame}" '
                f'for robot "{namespace}": {exc}. Using local coordinates.',
                throttle_duration_sec=5.0,
            )
            transform = None

        # Iterate through the objects contained in the semantic map
        count = min(
            len(msg.object_ids),
            len(msg.class_names),
            len(msg.positions),
            len(msg.confidences),
        )

        for idx in range(count):
            class_name = msg.class_names[idx]
            confidence = float(msg.confidences[idx])
            position_local = msg.positions[idx]

            global_point = self._transform_point(
                position_local, source_frame, msg.header.stamp, transform
            )

            self._merge_object(
                namespace=namespace,
                class_name=class_name,
                confidence=confidence,
                position=global_point,
                stamp=stamp,
            )

    # ------------------------------------------------------------------ #
    # Merging helpers
    # ------------------------------------------------------------------ #

    def _merge_object(
        self,
        *,
        namespace: str,
        class_name: str,
        confidence: float,
        position: Point,
        stamp: Time,
    ) -> None:
        """Fuse a single detection into the global map."""
        best_id = None
        best_distance = self.match_threshold

        for obj_id, obj in self.global_objects.items():
            if obj['class'] != class_name:
                continue

            distance = self._distance(obj['position'], position)
            if distance < best_distance:
                best_id = obj_id
                best_distance = distance

        if best_id:
            # Update existing entry
            entry = self.global_objects[best_id]
            observations = entry['observations']

            # Weighted average for position and confidence
            entry['position'].x = (
                entry['position'].x * observations + position.x
            ) / (observations + 1)
            entry['position'].y = (
                entry['position'].y * observations + position.y
            ) / (observations + 1)
            entry['position'].z = (
                entry['position'].z * observations + position.z
            ) / (observations + 1)

            entry['confidence'] = max(entry['confidence'], confidence)
            entry['observations'] = observations + 1
            entry['last_seen'] = stamp
            entry['sources'].add(namespace)
        else:
            # Create new entry
            object_id = f'{namespace}-{uuid.uuid4().hex[:8]}'
            self.global_objects[object_id] = {
                'class': class_name,
                'position': Point(x=position.x, y=position.y, z=position.z),
                'confidence': confidence,
                'observations': 1,
                'last_seen': stamp,
                'sources': {namespace},
            }

    def publish_global_map(self) -> None:
        """Publish the merged semantic map."""
        now = self.get_clock().now()
        self._remove_stale_objects(now)

        if not self.global_objects:
            return

        msg = SemanticMap()
        msg.header.frame_id = self.global_frame
        msg.header.stamp = now.to_msg()

        for obj_id, obj in self.global_objects.items():
            msg.object_ids.append(obj_id)
            msg.class_names.append(obj['class'])
            msg.positions.append(obj['position'])
            msg.confidences.append(float(obj['confidence']))

        self.global_map_pub.publish(msg)
        self.get_logger().info(
            f'Published global semantic map with {len(self.global_objects)} objects',
            throttle_duration_sec=5.0,
        )

    # ------------------------------------------------------------------ #
    # Utility helpers
    # ------------------------------------------------------------------ #

    def _resolve_map_topic(self, namespace: str) -> str:
        topic = self.semantic_map_topic.lstrip('/')
        return f'/{namespace}/{topic}'

    def _resolve_frame(self, namespace: str, frame_id: str) -> str:
        """Ensure the frame_id includes the robot namespace."""
        if not frame_id:
            return f'{namespace}/map'

        stripped = frame_id.lstrip('/')
        if stripped.startswith(f'{namespace}/'):
            return stripped

        return f'{namespace}/{stripped}'

    def _transform_point(
        self,
        point: Point,
        source_frame: str,
        stamp_msg,
        transform,
    ) -> Point:
        """Transform a point into the global frame if a transform is available."""
        if transform is None:
            return Point(x=point.x, y=point.y, z=point.z)

        point_stamped = PointStamped()
        point_stamped.header.frame_id = source_frame
        point_stamped.header.stamp = stamp_msg
        point_stamped.point = point

        transformed = do_transform_point(point_stamped, transform)
        return transformed.point

    def _remove_stale_objects(self, now: Time) -> None:
        """Discard objects that have not been observed recently."""
        to_remove = []
        for obj_id, obj in self.global_objects.items():
            age = (now - obj['last_seen']).nanoseconds / 1e9
            if age > self.stale_timeout:
                to_remove.append(obj_id)

        for obj_id in to_remove:
            data = self.global_objects.pop(obj_id)
            self.get_logger().info(
                f'Removed stale object "{obj_id}" ({data["class"]}) from global map',
                throttle_duration_sec=5.0,
            )

    @staticmethod
    def _distance(p1: Point, p2: Point) -> float:
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        dz = p1.z - p2.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SemanticMapMerger()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

