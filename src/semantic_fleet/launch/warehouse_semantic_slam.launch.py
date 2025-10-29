#!/usr/bin/env python3
"""
Warehouse Semantic SLAM Launch
Uses dynamic logistics warehouse with TurtleBot3 and semantic fleet
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    semantic_fleet_dir = get_package_share_directory('semantic_fleet')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    warehouse_world = os.path.join(semantic_fleet_dir, 'worlds', 'warehouse.world')
    models_dir = os.path.join(semantic_fleet_dir, 'models')
    urdf_file = os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    # Use Gazebo SDF for spawning so diff_drive plugin is available
    turtlebot3_model_folder = 'turtlebot3_waffle_pi'
    sdf_file = os.path.join(turtlebot3_gazebo_dir, 'models', turtlebot3_model_folder, 'model.sdf')
    
    
    # Get existing GAZEBO_MODEL_PATH and add both warehouse models and TB3 Gazebo models
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    tb3_models = os.path.join(turtlebot3_description_dir, '..')  # Parent of description package
    tb3_gazebo_models = os.path.join(turtlebot3_gazebo_dir, 'models')
    new_model_path = f"{models_dir}:{tb3_gazebo_models}:{tb3_models}:{existing_model_path}"
    
    return LaunchDescription([
        # Set Gazebo model path
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path),
        
        # 1. Launch Gazebo with warehouse world
        ExecuteProcess(
            cmd=['gazebo', '--verbose', warehouse_world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        
        # 2. Spawn TurtleBot3 using official turtlebot3_gazebo launcher
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_dir, 'launch', 'spawn_turtlebot3.launch.py')
            ),
            launch_arguments={
                'x_pose': '0.0',
                'y_pose': '0.0'
            }.items()
        ),
        
        # 4. SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'base_frame': 'base_footprint',
                'odom_frame': 'odom',
                'map_frame': 'map',
            }]
        ),
        
        # 5. Semantic Fleet - YOLO Detector
        Node(
            package='semantic_fleet',
            executable='yolo_detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'model_path': 'yolov8n.pt',
                'confidence_threshold': 0.6,
                'camera_topic': '/camera/image_raw',
                'publish_visualization': True,
                'detection_rate': 5.0,
            }]
        ),
        
        # 6. Object Localizer
        Node(
            package='semantic_fleet',
            executable='object_localizer',
            name='object_localizer',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'camera_frame': 'camera_rgb_optical_frame',
                'world_frame': 'map',
                'assumed_object_distance': 2.0,
            }]
        ),
        
        # 7. Semantic Mapper
        Node(
            package='semantic_fleet',
            executable='semantic_mapper',
            name='semantic_mapper',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'matching_distance_threshold': 0.5,
                'update_rate': 1.0,
                'min_observations': 2,
            }]
        ),
    ])
