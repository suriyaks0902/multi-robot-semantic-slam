#!/bin/bash

# Comprehensive verification script for SLAM map updates

cd /home/suriya/Desktop/fleet_ws
source install/setup.bash

echo "======================================"
echo "  SLAM Map Update Verification Tool"
echo "======================================"
echo ""

# Colors
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Check SLAM nodes
echo "Test 1: Checking SLAM nodes..."
SLAM_NODES=$(ros2 node list 2>/dev/null | grep slam | wc -l)
if [ "$SLAM_NODES" -eq 2 ]; then
    echo -e "${GREEN}✅ PASSED${NC}: Both SLAM nodes running"
    ros2 node list 2>/dev/null | grep slam | sed 's/^/   /'
else
    echo -e "${RED}❌ FAILED${NC}: Expected 2 SLAM nodes, found $SLAM_NODES"
fi
echo ""

# Test 2: Check map topics
echo "Test 2: Checking map topics..."
MAP_TOPICS=$(ros2 topic list 2>/dev/null | grep -E "robot_[0-9]+/map$" | wc -l)
if [ "$MAP_TOPICS" -eq 2 ]; then
    echo -e "${GREEN}✅ PASSED${NC}: Both map topics exist"
    ros2 topic list 2>/dev/null | grep -E "robot_[0-9]+/map$" | sed 's/^/   /'
else
    echo -e "${RED}❌ FAILED${NC}: Expected 2 map topics, found $MAP_TOPICS"
fi
echo ""

# Test 3: Check map data
echo "Test 3: Checking map data..."
MAP1_DATA=$(timeout 2 ros2 topic echo /robot_1/map --once 2>/dev/null | grep "width:" | awk '{print $2}')
if [ ! -z "$MAP1_DATA" ]; then
    echo -e "${GREEN}✅ PASSED${NC}: Robot 1 map has data (width: $MAP1_DATA)"
else
    echo -e "${YELLOW}⚠️  WARNING${NC}: Robot 1 map has no data yet (robot needs to move)"
fi

MAP2_DATA=$(timeout 2 ros2 topic echo /robot_2/map --once 2>/dev/null | grep "width:" | awk '{print $2}')
if [ ! -z "$MAP2_DATA" ]; then
    echo -e "${GREEN}✅ PASSED${NC}: Robot 2 map has data (width: $MAP2_DATA)"
else
    echo -e "${YELLOW}⚠️  WARNING${NC}: Robot 2 map has no data yet (robot needs to move)"
fi
echo ""

# Test 4: Live map update test
echo "Test 4: Testing live map updates..."
echo "   Getting initial timestamp..."
TIMESTAMP1=$(timeout 2 ros2 topic echo /robot_1/map --once 2>/dev/null | grep -A 1 "stamp:" | grep "sec:" | head -1 | awk '{print $2}')
if [ -z "$TIMESTAMP1" ]; then
    echo -e "${RED}❌ FAILED${NC}: Could not read initial map timestamp"
else
    echo "   Initial: $TIMESTAMP1 sec"
    echo "   Moving robot_1 forward for 2 seconds..."
    timeout 2 ros2 topic pub /robot_1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" > /dev/null 2>&1 &
    sleep 2.5
    echo "   Getting new timestamp..."
    TIMESTAMP2=$(timeout 2 ros2 topic echo /robot_1/map --once 2>/dev/null | grep -A 1 "stamp:" | grep "sec:" | head -1 | awk '{print $2}')
    echo "   New: $TIMESTAMP2 sec"
    
    if [ "$TIMESTAMP1" != "$TIMESTAMP2" ]; then
        echo -e "${GREEN}✅ PASSED${NC}: Map is updating! (Δt = $((TIMESTAMP2 - TIMESTAMP1)) sec)"
    else
        echo -e "${YELLOW}⚠️  WARNING${NC}: Map timestamp unchanged (SLAM might be paused)"
    fi
fi
echo ""

# Test 5: Check RViz
echo "Test 5: Checking RViz..."
RVIZ_RUNNING=$(ps aux | grep rviz2 | grep -v grep | wc -l)
if [ "$RVIZ_RUNNING" -gt 0 ]; then
    echo -e "${GREEN}✅ PASSED${NC}: RViz is running"
else
    echo -e "${YELLOW}⚠️  WARNING${NC}: RViz not running"
    echo "   Start with: rviz2 -d src/semantic_fleet/rviz/semantic_slam.rviz"
fi
echo ""

# Test 6: Check TF tree
echo "Test 6: Checking TF tree..."
WORLD_FRAME=$(timeout 2 ros2 run tf2_ros tf2_echo world robot_1/map 2>&1 | grep "Translation:" | wc -l)
if [ "$WORLD_FRAME" -gt 0 ]; then
    echo -e "${GREEN}✅ PASSED${NC}: world → robot_1/map transform exists"
else
    echo -e "${RED}❌ FAILED${NC}: world → robot_1/map transform missing"
fi
echo ""

# Summary
echo "======================================"
echo "  Verification Complete!"
echo "======================================"
echo ""
echo "📋 Summary:"
echo "   - SLAM nodes: $SLAM_NODES/2"
echo "   - Map topics: $MAP_TOPICS/2"
echo "   - RViz running: $([ "$RVIZ_RUNNING" -gt 0 ] && echo 'Yes' || echo 'No')"
echo ""
echo "📖 Next Steps:"
echo "   1. If all tests passed, drive robots to see map updates in RViz"
echo "   2. Use teleop to control robots:"
echo "      Terminal 1: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_1/cmd_vel"
echo "      Terminal 2: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/robot_2/cmd_vel"
echo "   3. Watch the maps grow in RViz as robots explore!"
echo ""
echo "📚 Documentation: MAP_UPDATE_FIX.md"
echo ""

