#!/bin/bash
# MoveIt demo (fake controllers) on its own ROS master, port 11312 -- never the robot's.
HERE="$(cd "$(dirname "$0")" && pwd)"; source "$HERE/env.sh"
setsid nohup roslaunch -p 11312 panda_moveit_config demo.launch use_rviz:=false > "$SIM_DIR/demo.log" 2>&1 < /dev/null &
echo "demo.launch starting; wait for 'You can start planning now!' in $SIM_DIR/demo.log"
