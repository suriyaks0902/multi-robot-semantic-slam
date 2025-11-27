#!/bin/bash

echo "🎮 Starting Teleop Controls for Both Robots"
echo "==========================================="
echo ""

cd /home/suriya/Desktop/fleet_ws
source install/setup.bash

# Check if system is running
if ! ros2 node list 2>/dev/null | grep -q robot_1; then
    echo "❌ System not running! Launch it first:"
    echo "   ros2 launch semantic_fleet multi_robot_semantic.launch.py"
    exit 1
fi

# Robot 1 teleop
gnome-terminal --title="🤖 Robot 1 Teleop" -- bash -c "
    cd /home/suriya/Desktop/fleet_ws && \
    source install/setup.bash && \
    echo '═══════════════════════════════════════' && \
    echo '🤖 Robot 1 Teleop Controls' && \
    echo '═══════════════════════════════════════' && \
    echo '' && \
    echo 'Movement Controls:' && \
    echo '  W - Forward' && \
    echo '  X - Backward' && \
    echo '  A - Turn Left' && \
    echo '  D - Turn Right' && \
    echo '  S - Stop' && \
    echo '' && \
    echo 'Speed Controls:' && \
    echo '  Q - Increase speed' && \
    echo '  Z - Decrease speed' && \
    echo '' && \
    echo 'Press CTRL+C to exit' && \
    echo '═══════════════════════════════════════' && \
    echo '' && \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_1/cmd_vel; \
    exec bash
" &

sleep 1

# Robot 2 teleop
gnome-terminal --title="🤖 Robot 2 Teleop" -- bash -c "
    cd /home/suriya/Desktop/fleet_ws && \
    source install/setup.bash && \
    echo '═══════════════════════════════════════' && \
    echo '🤖 Robot 2 Teleop Controls' && \
    echo '═══════════════════════════════════════' && \
    echo '' && \
    echo 'Movement Controls:' && \
    echo '  W - Forward' && \
    echo '  X - Backward' && \
    echo '  A - Turn Left' && \
    echo '  D - Turn Right' && \
    echo '  S - Stop' && \
    echo '' && \
    echo 'Speed Controls:' && \
    echo '  Q - Increase speed' && \
    echo '  Z - Decrease speed' && \
    echo '' && \
    echo 'Press CTRL+C to exit' && \
    echo '═══════════════════════════════════════' && \
    echo '' && \
    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_2/cmd_vel; \
    exec bash
" &

echo ""
echo "✅ Teleop windows opened!"
echo ""
echo "📋 Tips:"
echo "  1. Click on each terminal to control that robot"
echo "  2. Use W/A/S/D/X keys (must hold down)"
echo "  3. Watch Gazebo and RViz as robots move"
echo "  4. Maps will build as robots explore"
echo ""

