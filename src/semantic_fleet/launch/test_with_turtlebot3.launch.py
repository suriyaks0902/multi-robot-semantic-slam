#!/usr/bin/env python3
"""
Test Launch File - TurtleBot3 with Semantic SLAM
Launches Gazebo, TurtleBot3, SLAM Toolbox, and Semantic Fleet
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """
    Complete test setup for semantic SLAM with TurtleBot3 in Gazebo.
    
    Components:
    - Gazebo Classic with TurtleBot3 world
    - TurtleBot3 robot model
    - SLAM Toolbox for mapping
    - Semantic Fleet nodes (YOLO + localization + mapping)
    - RViz for visualization
    """
    
    # Launch Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='empty_world')
    
    # Package directories
    turtlebot3_gazebo_dir = FindPackageShare('turtlebot3_gazebo')
    semantic_fleet_dir = FindPackageShare('semantic_fleet')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_world',
            description='Gazebo world file'
        ),
        
        # 1. Launch Gazebo with TurtleBot3
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            additional_env={'GAZEBO_MODEL_PATH': os.environ.get('GAZEBO_MODEL_PATH', '')}
        ),
        
        # 2. Spawn TurtleBot3 in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'turtlebot3_waffle_pi',
                '-topic', 'robot_description',
                '-x', '0.0',
                '-y', '0.0',
                '-z', '0.01'
            ],
            output='screen'
        ),
        
        # 3. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }]
        ),
        
        # 4. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
                'scan_topic': '/scan',
                'mode': 'mapping',
            }]
        ),
        
        # 5. YOLO Detector Node
        Node(
            package='semantic_fleet',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.5,
                'camera_topic': '/camera/rgb/image_raw',
                'publish_visualization': True,
                'detection_rate': 5.0,  # Lower rate for better performance
            }],
            remappings=[
                ('/camera/image_raw', '/camera/rgb/image_raw'),
            ]
        ),
        
        # 6. Object Localizer Node
        Node(
            package='semantic_fleet',
            executable='object_localizer',
            name='object_localizer',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'camera_frame': 'camera_rgb_optical_frame',
                'world_frame': 'map',
                'assumed_object_distance': 2.0,
                'camera_fov_h': 1.085,
                'camera_fov_v': 0.785,
                'image_width': 640,
                'image_height': 480,
            }]
        ),
        
        # 7. Semantic Mapper Node
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
        
        # 8. RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                semantic_fleet_dir,
                'rviz',
                'semantic_slam.rviz'
            ])],
            condition=lambda: os.path.exists(
                os.path.join(
                    os.environ.get('HOME', ''),
                    'Desktop/fleet_ws/src/semantic_fleet/rviz/semantic_slam.rviz'
                )
            )
        ),
    ])

