#!/usr/bin/env python3
"""
TF Republisher Node
Dynamically republishes multiple robots' TF frames under world frame for multi-robot visualization
"""

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import math


class TFRepublisher(Node):
    """
    Dynamically republishes multiple robots' TF trees under world frame
    Supports any number of robots by taking a list of robot configurations
    """
    
    def __init__(self):
        super().__init__('tf_republisher')
        
        # Parameters
        self.declare_parameter('target_parent', 'world')
        self.declare_parameter('reference_robot', 'robot_1')  # Robot that defines world origin
        
        self.target_parent = self.get_parameter('target_parent').value
        self.reference_robot = self.get_parameter('reference_robot').value
        
        # Declare wildcard parameters to accept any robot offset parameters
        # This allows launch file to pass robot_1.offset_x, robot_2.offset_x, etc.
        robot_params = []
        for i in range(1, 11):  # Support up to 10 robots
            robot_params.extend([
                (f'robot_{i}.offset_x', 0.0),
                (f'robot_{i}.offset_y', 0.0),
                (f'robot_{i}.offset_z', 0.0),
                (f'robot_{i}.offset_yaw', 0.0),
            ])
        
        self.declare_parameters(
            namespace='',
            parameters=robot_params
        )
        
        # Parse flattened robot offset parameters (format: robot_1.offset_x, robot_2.offset_y, etc.)
        # Get all parameter names to find robot names
        all_param_names = [name for name in self._parameters.keys()]
        robot_names = set()
        
        for param_name in all_param_names:
            if '.' in param_name and param_name.endswith('.offset_x'):
                robot_name = param_name.split('.')[0]
                robot_names.add(robot_name)
        
        # Parse offset parameters for each discovered robot
        self.robots = {}
        for robot_name in robot_names:
            try:
                offset_x = float(self.get_parameter(f'{robot_name}.offset_x').value)
                offset_y = float(self.get_parameter(f'{robot_name}.offset_y').value)
                offset_z = float(self.get_parameter(f'{robot_name}.offset_z').value)
                offset_yaw = float(self.get_parameter(f'{robot_name}.offset_yaw').value)
                
                # Add robot (even if offsets are zero - all robots need TF published)
                self.robots[robot_name] = {
                    'offset_x': offset_x,
                    'offset_y': offset_y,
                    'offset_z': offset_z,
                    'offset_yaw': offset_yaw,
                }
            except Exception as e:
                self.get_logger().warn(f'Failed to get parameters for {robot_name}: {e}')
        
        # Log initialization info
        self.get_logger().info(f'TF Republisher initialized')
        self.get_logger().info(f'  Target parent frame: {self.target_parent}')
        self.get_logger().info(f'  Reference robot: {self.reference_robot}')
        self.get_logger().info(f'  Republishing {len(self.robots)} robots: {list(self.robots.keys())}')
        for robot_name, config in self.robots.items():
            self.get_logger().info(
                f'    {robot_name}: offset ({config["offset_x"]:.2f}, {config["offset_y"]:.2f}, {config["offset_z"]:.2f})'
            )
        
        # TF buffer and broadcaster
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Publish static transforms once (they don't change)
        # Note: We only need to republish the odom frame.
        # All other frames (base_footprint, base_link, map, etc.) are already
        # published by Gazebo/SLAM as children of odom, so they'll automatically
        # be connected to world through the odom transform.
        try:
            self.publish_static_transforms()
        except Exception as e:
            self.get_logger().error(f'Failed to publish static transforms: {e}')
    
    def publish_static_transforms(self):
        """
        Publish static transforms from 'world' to each robot's 'map' frame.
        
        This creates a bridge connecting all robots' TF trees to a common 'world' frame.
        SLAM publishes map -> odom, so we publish world -> map (not world -> odom) to avoid conflicts.
        Using static transforms avoids TF_OLD_DATA warnings since they don't have
        timestamps that conflict with Gazebo's dynamic transforms.
        """
        
        transforms = []
        for robot_name, config in self.robots.items():
            # Create world -> robot_i/map transform
            transform = TransformStamped()
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.header.frame_id = self.target_parent
            transform.child_frame_id = f'{robot_name}/map'
            transform.transform.translation.x = config['offset_x']
            transform.transform.translation.y = config['offset_y']
            transform.transform.translation.z = config['offset_z']
            
            # Convert yaw to quaternion
            yaw = config['offset_yaw']
            transform.transform.rotation.w = math.cos(yaw / 2.0)
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = math.sin(yaw / 2.0)
            
            transforms.append(transform)
        
        # Publish all static transforms at once
        if transforms:
            self.tf_broadcaster.sendTransform(transforms)
            self.get_logger().info(f'Published {len(transforms)} static TF transforms')
            for t in transforms:
                self.get_logger().info(
                    f'  {t.header.frame_id} → {t.child_frame_id}: '
                    f'[{t.transform.translation.x:.2f}, {t.transform.translation.y:.2f}, {t.transform.translation.z:.2f}]'
                )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = TFRepublisher()
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
