#!/usr/bin/env python3
"""
Semantic Map Visualizer.

Listens to one or more ``semantic_fleet/msg/SemanticMap`` topics and publishes
their contents as ``visualization_msgs/MarkerArray`` streams so they can be
viewed directly in RViz without any custom plugins.
"""

from __future__ import annotations

import hashlib
from typing import Dict, List

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import ParameterType, ParameterValue
from rcl_interfaces.msg import ParameterDescriptor

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from semantic_fleet.msg import SemanticMap


class SemanticMapVisualizer(Node):
    """Convert semantic map messages into RViz MarkerArrays."""

    def __init__(self) -> None:
        super().__init__('semantic_map_visualizer')

        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', True)
        self._declare_parameters()

        topics = list(self._topics)
        if not topics:
            self.get_logger().warning(
                'No semantic_map_topics provided; falling back to ["semantic_mapper/semantic_map"].'
            )
            topics = ['semantic_mapper/semantic_map']

        self._channels: Dict[str, MarkerChannel] = {}
        for topic in topics:
            resolved_topic = self.resolve_topic_name(topic)
            marker_topic = self._build_marker_topic(resolved_topic)
            channel = MarkerChannel(
                node=self,
                input_topic=resolved_topic,
                marker_topic=marker_topic,
                marker_scale=self._marker_scale,
                marker_alpha=self._marker_alpha,
                marker_lifetime=self._marker_lifetime,
                frame_fallback=self._frame_fallback,
            )
            self._channels[resolved_topic] = channel
            self.get_logger().info(
                f'Visualizing "{resolved_topic}" -> "{marker_topic}" with scale {self._marker_scale:.2f} m'
            )

    # --------------------------------------------------------------------- #
    # Parameter helpers
    # --------------------------------------------------------------------- #
    def _declare_parameters(self) -> None:
        topics_descriptor = ParameterDescriptor(
            name='semantic_map_topics',
            type=ParameterType.PARAMETER_STRING_ARRAY,
            description='List of semantic_fleet/SemanticMap topics to visualize.',
        )
        default_topics = ParameterValue(
            type=ParameterType.PARAMETER_STRING_ARRAY,
            string_array_value=['semantic_mapper/semantic_map'],
        )
        self._topics = self.declare_parameter(
            'semantic_map_topics',
            default_topics,
            topics_descriptor,
        ).get_parameter_value().string_array_value

        self._marker_scale = float(self.declare_parameter('marker_scale', 0.35).value)
        self._marker_alpha = float(self.declare_parameter('marker_alpha', 0.85).value)
        self._marker_lifetime = Duration(
            seconds=float(self.declare_parameter('marker_lifetime', 1.5).value)
        )
        self._frame_fallback = str(self.declare_parameter('frame_fallback', 'map').value)

    def _build_marker_topic(self, semantic_topic: str) -> str:
        """Derive a marker topic name from the input topic."""
        if semantic_topic.endswith('/'):
            semantic_topic = semantic_topic.rstrip('/')
        return f'{semantic_topic}_markers'


class MarkerChannel:
    """Pair of subscription/publisher for a single semantic map topic."""

    def __init__(
        self,
        node: SemanticMapVisualizer,
        input_topic: str,
        marker_topic: str,
        *,
        marker_scale: float,
        marker_alpha: float,
        marker_lifetime: Duration,
        frame_fallback: str,
    ) -> None:
        self._node = node
        self._marker_scale = marker_scale
        self._marker_alpha = marker_alpha
        self._marker_lifetime = marker_lifetime
        self._frame_fallback = frame_fallback
        self._marker_topic = marker_topic

        self._publisher = node.create_publisher(MarkerArray, marker_topic, 10)
        self._subscription = node.create_subscription(
            SemanticMap,
            input_topic,
            self._on_message,
            10,
        )

    # ------------------------------------------------------------------ #
    def _on_message(self, msg: SemanticMap) -> None:
        markers = MarkerArray()
        stamp = msg.header.stamp
        if stamp.sec == 0 and stamp.nanosec == 0:
            stamp = self._node.get_clock().now().to_msg()

        count = min(
            len(msg.object_ids),
            len(msg.class_names),
            len(msg.positions),
            len(msg.confidences),
        )
        for idx in range(count):
            object_id = msg.object_ids[idx] or f'obj_{idx}'
            class_name = msg.class_names[idx] or 'unknown'
            position = msg.positions[idx]
            confidence = msg.confidences[idx] if idx < len(msg.confidences) else 0.0

            marker = Marker()
            marker.header.frame_id = msg.header.frame_id or self._frame_fallback
            marker.header.stamp = stamp
            marker.ns = class_name or 'semantic_object'
            marker.id = self._stable_id(object_id, class_name)
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position = position
            marker.pose.orientation.w = 1.0
            marker.scale.x = marker.scale.y = marker.scale.z = self._marker_scale
            color = self._color_for_class(class_name)
            marker.color = ColorRGBA(
                r=color[0],
                g=color[1],
                b=color[2],
                a=self._marker_alpha,
            )
            marker.lifetime = self._marker_lifetime.to_msg()
            marker.text = f'{class_name} ({confidence:.2f})'

            markers.markers.append(marker)

        # Keep RViz clean by issuing a single DELETEALL before publishing new markers
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        markers.markers.insert(0, delete_all)

        self._publisher.publish(markers)

    # ------------------------------------------------------------------ #
    def _stable_id(self, object_id: str, class_name: str) -> int:
        digest = hashlib.sha1(f'{self._marker_topic}:{object_id}:{class_name}'.encode('utf-8')).hexdigest()
        return int(digest[:8], 16)

    def _color_for_class(self, class_name: str) -> List[float]:
        digest = hashlib.md5(class_name.encode('utf-8')).hexdigest()
        r = int(digest[0:2], 16) / 255.0
        g = int(digest[2:4], 16) / 255.0
        b = int(digest[4:6], 16) / 255.0
        return [r, g, b]


def main(args: List[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SemanticMapVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


