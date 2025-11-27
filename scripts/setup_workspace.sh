#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/suriya/Desktop/fleet_ws/install/setup.bash
source /home/suriya/Desktop/fleet_ws/.venv/bin/activate
export PYTHONPATH=/home/suriya/Desktop/fleet_ws/.venv/lib/python3.10/site-packages:$PYTHONPATH
export TURTLEBOT3_MODEL=waffle_pi
echo "✅ Environment ready! TurtleBot3 model: $TURTLEBOT3_MODEL"
