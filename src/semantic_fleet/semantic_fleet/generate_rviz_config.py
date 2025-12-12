#!/usr/bin/env python3
"""
Dynamic RViz Configuration Generator

Generates RViz config files with the correct number of robot map displays
based on the num_robots parameter. 
"""

import sys
import os


def generate_map_display(robot_num):
    """Generate a Map display YAML block for a robot"""
    return f"""    - Class: rviz_default_plugins/Map
      Enabled: true
      Name: Map - Robot {robot_num}
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_{robot_num}/map
      Alpha: 0.5 
      Color Scheme: costmap
      Draw Behind: true
      Update Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Best Effort
        Value: /robot_{robot_num}/map_updates"""


def generate_rviz_config(num_robots, base_config_path, output_path):
    """
    Generate dynamic RViz configuration preserving exact format
    
    Args:
        num_robots: Number of robots to configure displays for
        base_config_path: Path to base RViz config template
        output_path: Where to write the generated config
    """
    
    # Read base config as text (preserve exact formatting)
    with open(base_config_path, 'r') as f:
        lines = f.readlines()
    
    # Find where to insert map displays (after RobotModel, before LaserScan)
    output_lines = []
    skip_mode = False
    inserted = False
    
    for i, line in enumerate(lines):
        # Detect start of a "Map - Robot" block
        if '    - Class: rviz_default_plugins/Map' in line:
            # Check if next non-empty line contains "Map - Robot"
            next_lines = ''.join(lines[i:min(i+5, len(lines))])
            if 'Name: Map - Robot' in next_lines:
                skip_mode = True
                continue
        
        # If we're skipping a Map - Robot block, continue until next display
        if skip_mode:
            # Next display starts with "    - Class:"
            if line.startswith('    - Class:') and 'Map' not in line:
                skip_mode = False
                # Insert all robot maps before this line
                if not inserted:
                    for robot_num in range(1, num_robots + 1):
                        output_lines.append(generate_map_display(robot_num) + '\n')
                    inserted = True
                output_lines.append(line)
            continue
        
        output_lines.append(line)
    
    # Write generated config
    with open(output_path, 'w') as f:
        f.writelines(output_lines)
    
    print(f"✅ Generated RViz config for {num_robots} robots: {output_path}")


if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: generate_rviz_config.py <num_robots> <base_config> <output_config>")
        sys.exit(1)
    
    num_robots = int(sys.argv[1])
    base_config = sys.argv[2]
    output_config = sys.argv[3]
    
    generate_rviz_config(num_robots, base_config, output_config)

