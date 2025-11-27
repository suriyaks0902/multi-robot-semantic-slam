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
from math import sin, cos
from typing import Dict


def generate_launch_description():
    semantic_fleet_dir = get_package_share_directory('semantic_fleet')
    turtlebot3_description_dir = get_package_share_directory('turtlebot3_description')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')

    # Use no-roof small warehouse for better performance (fewer models = faster simulation)
    warehouse_world = os.path.join(semantic_fleet_dir, 'worlds', 'no_roof_small_warehouse.world')
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
        # Two robots for better performance - positioned near walls for immediate SLAM initialization
        # Spawn closer to walls so laser can detect obstacles immediately (laser range = 3.5m)
        {'name': 'robot_1', 'x': -2.0, 'y': -1.0, 'yaw': 0.0},  # Closer to wall
        {'name': 'robot_2', 'x': 2.0,  'y': 1.0,  'yaw': 0.0},  # Closer to wall
    ]

    # Cache original SDF so we only read it once
    with open(sdf_file, 'r') as f:
        sdf_template = f.read()

    robot_groups = []
    for robot in robots:
        namespace = robot['name']
        frame_prefix = f'{namespace}/'
        yaw = robot.get('yaw', 0.0)
        qz = sin(yaw / 2.0)
        qw = cos(yaw / 2.0)

        # Write a namespaced copy of the TurtleBot3 SDF so Gazebo publishes TF
        # frames that match our ROS 2 naming convention. We keep a shared "odom"
        # frame but namespace the robot base and sensor frames.
        namespaced_sdf_path = f'/tmp/semantic_fleet_{namespace}_turtlebot3.sdf'
        sdf_ns = sdf_template
        replacements: Dict[str, str] = {
            '<robot_base_frame>base_footprint</robot_base_frame>':
            f'<robot_base_frame>{namespace}/base_footprint</robot_base_frame>',
            '<frame_name>base_scan</frame_name>':
            f'<frame_name>{namespace}/base_scan</frame_name>',
            '<frame_name>camera_rgb_optical_frame</frame_name>':
            f'<frame_name>{namespace}/camera_rgb_optical_frame</frame_name>',
            '<command_topic>cmd_vel</command_topic>':
            f'<command_topic>{namespace}/cmd_vel</command_topic>',
            # ADD THIS: Namespace the odom frame in diff_drive plugin
            '<odometry_frame>odom</odometry_frame>':
            f'<odometry_frame>{namespace}/odom</odometry_frame>',
            # Also handle if it's written differently
            'odom_frame">odom</odom_frame>':
            f'odom_frame">{namespace}/odom</odom_frame>',
        }
        for old, new in replacements.items():
            sdf_ns = sdf_ns.replace(old, new)
        
        # Add command timeout to stop robot if no commands received (0.5 seconds)
        # This prevents the robot from continuing to move after teleop stops
        if '<command_timeout>' not in sdf_ns:
            # Insert command_timeout right after command_topic
            sdf_ns = sdf_ns.replace(
                f'<command_topic>{namespace}/cmd_vel</command_topic>',
                f'<command_topic>{namespace}/cmd_vel</command_topic>\n      <command_timeout>0.5</command_timeout>'
            )
        
        os.makedirs(os.path.dirname(namespaced_sdf_path), exist_ok=True)
        with open(namespaced_sdf_path, 'w') as f:
            f.write(sdf_ns)

        robot_groups.append(
            GroupAction([
                PushRosNamespace(namespace),

                # Spawn robot into Gazebo with namespaced topics
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', namespace,
                        '-file', namespaced_sdf_path,
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
                # Explicitly remap map topics to ensure namespacing works
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    remappings=[
                        ('/map', f'/{namespace}/map'),
                        ('/map_metadata', f'/{namespace}/map_metadata'),
                    ],
                    parameters=[{
                        'use_sim_time': True,
                        'base_frame': f'{namespace}/base_footprint',
                        'odom_frame': f'{namespace}/odom',
                        'map_frame': f'{namespace}/map',
                        'scan_topic': 'scan',
                        'mode': 'mapping',
                        'map_update_interval': 1.0,  # Reduced from 0.5 to 1.0 to reduce RViz queue overflow
                        'resolution': 0.025,
                        'max_laser_range': 3.5,  # Match actual laser range (was 10.0)
                        'minimum_laser_range': 0.1,  # Match actual laser min range (was 0.0)
                        'minimum_travel_distance': 0.05,
                        'minimum_travel_heading': 0.05,
                        'transform_timeout': 1.0,  # Increased for better reliability
                        'tf_buffer_duration': 30.0,
                        'stack_size_to_use': 40000000,
                        'scan_buffer_size': 50,  # Increased from 25 to handle queue overflow
                        'scan_buffer_maximum_scan_distance': 3.5,  # Match laser range (was 20.0)
                        'link_match_minimum_response_fine': 0.05,
                        'loop_search_maximum_distance': 5.0,
                        'enable_interactive_mode': False,
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
                        'output_frame': f'{namespace}/map',
                    }],
                ),
            ])
        )

    # Create world frame as a standalone root frame
    # Then connect both robot odom frames to world for visualization
    # robot_1 spawns at (-2.0, -1.0), robot_2 spawns at (2.0, 1.0)
    # We'll place world at robot_1's spawn position, so robot_1/odom is at (0,0) in world
    # and robot_2/odom is at (4.0, 2.0) in world
    
    # First create world frame (we'll use robot_1/odom as temporary parent, then make world root)
    world_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='semantic_fleet_world_base',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'robot_1/odom', 'world'],
    )
    
    # Create world frame and connect both robot odom frames to it
    # Since robot_1/odom and robot_2/odom are root frames from Gazebo, we need to:
    # 1. Create world as a child of robot_1/odom (so it exists)
    # 2. Connect robot_2/odom to world (robot_2 is at (4.0, 2.0) relative to robot_1)
    # This creates: robot_1/odom -> world -> robot_2/odom
    # For RViz, we'll use world as fixed frame
    
    # Create world frame as PARENT of robot_1/map (identity transform)
    # This makes 'world' the root of the TF tree
    # SLAM publishes map -> odom, so we need world -> map (not world -> odom)
    world_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='semantic_fleet_world_frame',
        arguments=['--x', '0.0', '--y', '0.0', '--z', '0.0', 
                  '--roll', '0.0', '--pitch', '0.0', '--yaw', '0.0',
                  '--frame-id', 'world', '--child-frame-id', 'robot_1/map'],
    )
    
    # TF Republisher: Dynamically republishes all robots' map frames (except reference robot) under world frame
    # This creates: world -> robot_i/map (static, with offset) -> robot_i/odom (dynamic, from SLAM)
    # SLAM publishes map -> odom, so we must connect world -> map (not world -> odom) to avoid conflicts.
    
    # Calculate offsets for each robot relative to reference robot (robot_1)
    ref_robot = next((r for r in robots if r['name'] == 'robot_1'), None)
    ref_x = ref_robot['x'] if ref_robot else 0.0
    ref_y = ref_robot['y'] if ref_robot else 0.0
    ref_z = ref_robot.get('z', 0.0)
    ref_yaw = ref_robot.get('yaw', 0.0)
    
    # Build flattened parameters (ROS 2 doesn't handle nested dicts well)
    # Format: robot_2.offset_x, robot_2.offset_y, etc.
    tf_republisher_params = {
        'use_sim_time': True,
        'target_parent': 'world',
        'reference_robot': 'robot_1',
    }
    
    # Add flattened offset parameters for each robot
    for robot in robots:
        if robot['name'] != 'robot_1':  # Skip reference robot
            tf_republisher_params[f"{robot['name']}.offset_x"] = float(robot['x'] - ref_x)
            tf_republisher_params[f"{robot['name']}.offset_y"] = float(robot['y'] - ref_y)
            tf_republisher_params[f"{robot['name']}.offset_z"] = float(robot.get('z', 0.0) - ref_z)
            tf_republisher_params[f"{robot['name']}.offset_yaw"] = float(robot.get('yaw', 0.0) - ref_yaw)
    
    tf_republisher = Node(
        package='semantic_fleet',
        executable='tf_republisher',
        name='tf_republisher',
        output='screen',
        parameters=[tf_republisher_params],
    )

    merger_node = Node(
        package='semantic_fleet',
        executable='semantic_map_merger',
        name='semantic_map_merger',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_namespaces': [robot['name'] for robot in robots],
            'semantic_map_topic': 'semantic_mapper/semantic_map',
            'global_map_topic': '/semantic_fleet/global_semantic_map',
            'global_frame': 'world',  # Changed from 'world' to 'odom' since all maps connect to odom
            'matching_distance_threshold': 0.75,
            'publish_rate': 2.0,
            'stale_object_timeout': 15.0,
        }],
    )

    visualization_topics = [
        f'/{robot["name"]}/semantic_mapper/semantic_map' for robot in robots
    ] + ['/semantic_fleet/global_semantic_map']

    semantic_map_visualizer = Node(
        package='semantic_fleet',
        executable='semantic_map_visualizer',
        name='semantic_map_visualizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'semantic_map_topics': visualization_topics,
            'marker_scale': 0.35,
            'marker_lifetime': 2.0,
            'frame_fallback': 'map',
        }],
    )

    rviz_config = os.path.join(semantic_fleet_dir, 'rviz', 'semantic_slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        parameters=[{
            'use_sim_time': True,
        }],
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
        world_frame,
        tf_republisher,
        *robot_groups,
        merger_node,
        semantic_map_visualizer,
        rviz_node,
    ])

