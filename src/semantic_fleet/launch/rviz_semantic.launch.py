#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d',
                       '/home/suriya/Desktop/fleet_ws/install/semantic_fleet/share/semantic_fleet/rviz/semantic_slam.rviz'],
            parameters=[{'use_sim_time': True}],
        )
    ])


