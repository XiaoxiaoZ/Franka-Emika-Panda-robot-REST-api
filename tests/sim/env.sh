# Source this for anything that must talk to the SIMULATION master only.
source /opt/ros/noetic/setup.bash
source "$HOME/catkin_ws/devel/setup.bash"
export SIM_DIR="${SIM_DIR:-/tmp/franka_sim}"
export ROS_MASTER_URI=http://127.0.0.1:11312
export ROS_HOSTNAME=127.0.0.1
export ROS_LOG_DIR="$SIM_DIR/roslog"
mkdir -p "$SIM_DIR"
