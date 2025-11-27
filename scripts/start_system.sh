#!/bin/bash

# Clean startup script for multi-robot semantic SLAM

cd /home/suriya/Desktop/fleet_ws
source install/setup.bash

echo "========================================="
echo "  Starting Multi-Robot Semantic SLAM"
echo "========================================="
echo ""

# Clean up any existing processes
echo "Cleaning up existing processes..."
pkill -9 -f "gzserver|gzclient|ros2 launch|slam_toolbox|rviz" 2>/dev/null
sleep 2

# Start the system
echo "Launching system (this will take ~15 seconds)..."
ros2 launch semantic_fleet multi_robot_semantic.launch.py > /tmp/system_launch.log 2>&1 &
LAUNCH_PID=$!

echo "Launch PID: $LAUNCH_PID"
echo ""

# Wait for initialization
echo "Waiting for system to initialize..."
for i in {1..15}; do
    echo -n "."
    sleep 1
done
echo ""
echo ""

# Verify system status
echo "========================================="
echo "  System Status"
echo "========================================="
echo ""

# Check Gazebo
GAZEBO=$(ps aux | grep gzserver | grep -v grep | wc -l)
echo "Gazebo: $([ "$GAZEBO" -gt 0 ] && echo '✅ Running' || echo '❌ Not running')"

# Check SLAM nodes
SLAM=$(ros2 node list 2>/dev/null | grep slam | wc -l)
echo "SLAM nodes: $SLAM/2 $([ "$SLAM" -eq 2 ] && echo '✅' || echo '❌')"

# Check RViz
RVIZ=$(ps aux | grep rviz2 | grep -v grep | wc -l)
echo "RViz: $([ "$RVIZ" -gt 0 ] && echo '✅ Running' || echo '❌ Not running')"

# Check map topics
MAPS=$(ros2 topic list 2>/dev/null | grep -E "robot_[0-9]+/map$" | wc -l)
echo "Map topics: $MAPS/2 $([ "$MAPS" -eq 2 ] && echo '✅' || echo '❌')"

echo ""
echo "========================================="
echo "  Ready to Test!"
echo "========================================="
echo ""
echo "Open 2 new terminals and run teleop for each robot:"
echo ""
echo "Terminal 1:"
echo "  cd /home/suriya/Desktop/fleet_ws"
echo "  source install/setup.bash"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_1/cmd_vel"
echo ""
echo "Terminal 2:"
echo "  cd /home/suriya/Desktop/fleet_ws"
echo "  source install/setup.bash"
echo "  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_2/cmd_vel"
echo ""
echo "Then drive the robots around and watch the maps update in RViz!"
echo ""
echo "Launch log: /tmp/system_launch.log"
echo ""

