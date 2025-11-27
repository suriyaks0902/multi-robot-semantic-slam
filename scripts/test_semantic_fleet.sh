#!/bin/bash

# Test Script for Semantic Fleet with TurtleBot3
# This script provides step-by-step instructions for testing

echo "======================================================================"
echo "  Semantic Fleet Testing Guide - TurtleBot3 + Gazebo"
echo "======================================================================"
echo ""
echo "This will guide you through testing the semantic_fleet package."
echo "You'll need to run commands in SEPARATE TERMINALS."
echo ""
echo "======================================================================"
echo ""

# Check if environment is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS 2 not sourced! Please run:"
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

# Check if workspace is built
if [ ! -d "$HOME/Desktop/fleet_ws/install/semantic_fleet" ]; then
    echo "❌ Workspace not built! Please run:"
    echo "   cd ~/Desktop/fleet_ws"
    echo "   colcon build --packages-select semantic_fleet"
    exit 1
fi

echo "✓ ROS 2 Humble detected"
echo "✓ Workspace built"
echo ""

# Check TURTLEBOT3_MODEL
if [ -z "$TURTLEBOT3_MODEL" ]; then
    echo "⚠️  TURTLEBOT3_MODEL not set. Setting to waffle_pi..."
    export TURTLEBOT3_MODEL=waffle_pi
fi

echo "✓ TurtleBot3 model: $TURTLEBOT3_MODEL"
echo ""

echo "======================================================================"
echo "  STEP-BY-STEP TESTING INSTRUCTIONS"
echo "======================================================================"
echo ""
echo "Open 5 SEPARATE TERMINAL WINDOWS and run these commands:"
echo ""
echo "----------------------------------------------------------------------"
echo "TERMINAL 1: Source and launch Gazebo + TurtleBot3"
echo "----------------------------------------------------------------------"
echo "cd ~/Desktop/fleet_ws"
echo "source setup_workspace.sh"
echo "export TURTLEBOT3_MODEL=waffle_pi"
echo "ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "----------------------------------------------------------------------"
echo "TERMINAL 2: Launch SLAM Toolbox"
echo "----------------------------------------------------------------------"
echo "cd ~/Desktop/fleet_ws"
echo "source setup_workspace.sh"
echo "ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True"
echo ""
echo "  OR (if using SLAM Toolbox):"
echo ""
echo "ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True"
echo ""
echo "----------------------------------------------------------------------"
echo "TERMINAL 3: Launch Semantic Fleet Nodes"
echo "----------------------------------------------------------------------"
echo "cd ~/Desktop/fleet_ws"
echo "source setup_workspace.sh"
echo "ros2 launch semantic_fleet single_robot_semantic.launch.py"
echo ""
echo "----------------------------------------------------------------------"
echo "TERMINAL 4: Teleop - Drive the Robot"
echo "----------------------------------------------------------------------"
echo "cd ~/Desktop/fleet_ws"
echo "source setup_workspace.sh"
echo "export TURTLEBOT3_MODEL=waffle_pi"
echo "ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "----------------------------------------------------------------------"
echo "TERMINAL 5: Monitor Topics (Optional)"
echo "----------------------------------------------------------------------"
echo "cd ~/Desktop/fleet_ws"
echo "source setup_workspace.sh"
echo ""
echo "# List all topics"
echo "ros2 topic list"
echo ""
echo "# View detected objects"
echo "ros2 topic echo /yolo_detector/detected_objects"
echo ""
echo "# View semantic map"
echo "ros2 topic echo /semantic_mapper/semantic_map"
echo ""
echo "# View detection visualization"
echo "ros2 run rqt_image_view rqt_image_view"
echo "  (Select /yolo_detector/detection_visualization)"
echo ""
echo "======================================================================"
echo ""
echo "📝 NOTES:"
echo "  - Make sure YOLOv8 model is downloaded (first run downloads ~6MB)"
echo "  - Gazebo world has objects like chairs, tables that YOLO can detect"
echo "  - Drive the robot around to see different objects"
echo "  - Check detection_visualization to see YOLO working"
echo ""
echo "======================================================================"
echo ""
read -p "Press ENTER to create a quick-start alias..."

# Add helpful alias to bashrc
if ! grep -q "alias test_semantic_fleet" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Semantic Fleet Quick Start" >> ~/.bashrc
    echo "alias fleet_env='cd ~/Desktop/fleet_ws && source setup_workspace.sh && export TURTLEBOT3_MODEL=waffle_pi'" >> ~/.bashrc
    echo ""
    echo "✓ Added 'fleet_env' alias to ~/.bashrc"
    echo "  Usage: Run 'fleet_env' in each terminal before launching"
fi

echo ""
echo "======================================================================"
echo "Ready to test! Open terminals and follow the instructions above."
echo "======================================================================"

