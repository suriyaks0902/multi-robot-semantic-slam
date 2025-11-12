#!/usr/bin/env python3
"""
Multi-Robot Semantic SLAM Launch

Spawns three TurtleBot3 robots in the warehouse world, each running the
semantic perception stack (SLAM Toolbox, YOLO detector, object localizer,
semantic mapper) inside an isolated ROS 2 namespace.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    semantic_fleet_dir = get_package_share_directory('semantic_fleet')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    warehouse_world = os.path.join(semantic_fleet_dir, 'worlds', 'warehouse.world')
    models_dir = os.path.join(semantic_fleet_dir, 'models')

    urdf_file = os.path.join(turtlebot3_description_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    turtlebot3_model_folder = 'turtlebot3_waffle_pi'
    sdf_file = os.path.join(turtlebot3_gazebo_dir, 'models', turtlebot3_model_folder, 'model.sdf')

    # Ensure Python venv is in path for YOLO node
    venv_path = os.path.join(os.path.expanduser('~'), 'Desktop', 'fleet_ws', '.venv')
    python_path = os.path.join(venv_path, 'lib', 'python3.10', 'site-packages')
    existing_python_path = os.environ.get('PYTHONPATH', '')
    new_python_path = f"{python_path}:{existing_python_path}" if existing_python_path else python_path

    # Extend Gazebo model path with semantic_fleet and turtlebot3 models
    existing_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    tb3_models_root = os.path.join(turtlebot3_description_dir, '..')
    tb3_gazebo_models = os.path.join(turtlebot3_gazebo_dir, 'models')
    new_model_path = f"{models_dir}:{tb3_gazebo_models}:{tb3_models_root}:{existing_model_path}"

    robots = [
        {'name': 'robot_1', 'x': -2.0, 'y': 0.0, 'yaw': 0.0},
        {'name': 'robot_2', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        {'name': 'robot_3', 'x': 2.0, 'y': 0.0, 'yaw': 0.0},
    ]

    robot_groups = []
    for robot in robots:
        namespace = robot['name']
        frame_prefix = f'{namespace}/'

        robot_groups.append(
            GroupAction([
                PushRosNamespace(namespace),

                # Spawn robot into Gazebo with namespaced topics
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', namespace,
                        '-file', sdf_file,
                        '-robot_namespace', namespace,
                        '-x', str(robot['x']),
                        '-y', str(robot['y']),
                        '-z', '0.01',
                        '-Y', str(robot.get('yaw', 0.0)),
                    ],
                    output='screen',
                ),

                # Robot State Publisher for TF tree
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'robot_description': Command([
                            'xacro ', urdf_file, ' namespace:=', frame_prefix
                        ]),
                    }],
                ),

                # SLAM Toolbox instance (per robot)
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'base_frame': f'{namespace}/base_footprint',
                        'odom_frame': f'{namespace}/odom',
                        'map_frame': f'{namespace}/map',
                        'scan_topic': 'scan',
                        'mode': 'mapping',
                        'map_update_interval': 0.5,
                        'resolution': 0.025,
                        'max_laser_range': 10.0,
                        'minimum_travel_distance': 0.05,
                        'minimum_travel_heading': 0.05,
                        'transform_timeout': 0.5,
                        'tf_buffer_duration': 30.0,
                        'stack_size_to_use': 40000000,
                        'scan_buffer_size': 25,
                        'scan_buffer_maximum_scan_distance': 20.0,
                        'link_match_minimum_response_fine': 0.05,
                        'loop_search_maximum_distance': 5.0,
                    }],
                ),

                # YOLO detector per robot
                Node(
                    package='semantic_fleet',
                    executable='yolo_detector',
                    name='yolo_detector',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'model_path': 'yolov8n.pt',
                        'confidence_threshold': 0.6,
                        'camera_topic': 'camera/image_raw',
                        'publish_visualization': True,
                        'detection_rate': 5.0,
                    }],
                ),

                # Object localization per robot
                Node(
                    package='semantic_fleet',
                    executable='object_localizer',
                    name='object_localizer',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'camera_frame': f'{namespace}/camera_rgb_optical_frame',
                        'world_frame': f'{namespace}/map',
                        'assumed_object_distance': 2.0,
                    }],
                ),

                # Semantic mapper per robot
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
                    }],
                ),
            ])
        )

    return LaunchDescription([
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path),
        SetEnvironmentVariable('PYTHONPATH', new_python_path),

        ExecuteProcess(
            cmd=[
                'gazebo', '--verbose', warehouse_world,
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so',
            ],
            output='screen',
        ),
        *robot_groups,
    ])

