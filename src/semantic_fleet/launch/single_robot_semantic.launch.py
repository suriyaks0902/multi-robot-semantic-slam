#!/usr/bin/env python3
"""
Single Robot Semantic SLAM Launch File
Launches all nodes needed for single robot semantic SLAM demonstration
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch file for single robot semantic SLAM.
    
    Nodes launched:
    - yolo_detector: YOLO object detection
    - object_localizer: Transform detections to world frame
    - semantic_mapper: Maintain semantic map
    """
    
    # Package directories
    semantic_fleet_dir = get_package_share_directory('semantic_fleet')
    config_dir = os.path.join(semantic_fleet_dir, 'config')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        
        LogInfo(msg='Starting Semantic Fleet - Single Robot Demo'),
        
        # YOLO Detector Node
        Node(
            package='semantic_fleet',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.6,  # Increased to reduce false positives
                'camera_topic': '/camera/image_raw',  # Correct TurtleBot3 camera topic
                'publish_visualization': True,
                'detection_rate': 5.0,  # Reduced for better performance
            }]
        ),
        
        # Object Localizer Node
        Node(
            package='semantic_fleet',
            executable='object_localizer',
            name='object_localizer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_frame': 'camera_rgb_optical_frame',
                'world_frame': 'map',
                'assumed_object_distance': 1.5,
                'camera_fov_h': 1.085,  # ~62 degrees
                'camera_fov_v': 0.785,  # ~45 degrees
                'image_width': 640,
                'image_height': 480,
            }]
        ),
        
        # Semantic Mapper Node
        Node(
            package='semantic_fleet',
            executable='semantic_mapper',
            name='semantic_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'matching_distance_threshold': 0.5,
                'update_rate': 1.0,
                'min_observations': 2,
                'confidence_decay': 0.95,
                'min_confidence': 0.3,
            }]
        ),
        
        LogInfo(msg='Semantic Fleet nodes launched! Waiting for camera input...'),
    ])

