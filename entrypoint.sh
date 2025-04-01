#!/bin/bash

<<<<<<< HEAD:entrypoint.sh
# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source any additional workspace if it exists
if [ -f /action/ros2_ws/install/setup.bash ]; then
    source /action/ros2_ws/install/setup.bash
fi

# Set up ROS environment variables
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0

# If no command is provided, start bash
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    # Execute the provided command
    exec "$@"
fi 
=======
set -e

# ====== Entry point for the Docker container ======
# This part is used to reindex the rosbag files after the container is stopped.
# It listens for SIGTERM and SIGINT signals, which are sent when the container is stopped.
_term() {
  echo "[entrypoint] Caught SIGTERM, stopping recorder..."

  pkill -SIGINT -f recorder_node || true

  sleep 2 

  # rosbag reindex rosbag mcap 
 if [ -f /tmp/latest_bag_path.txt ]; then
    LATEST_BAG=$(cat /tmp/latest_bag_path.txt)
    echo "[entrypoint] Reindexing rosbag in: $LATEST_BAG"
    ros2 bag reindex "$LATEST_BAG"
  else
    echo "[entrypoint] No latest bag path file found, skip reindex."
  fi

  exit 0
}

trap _term SIGTERM SIGINT

# main process of the container
source /opt/ros/${ROS_DISTRO}/setup.bash

cd /action/ros2_ws
colcon build --event-handlers console_direct+

source /action/ros2_ws/install/setup.bash

python3 -m http.server 8000 --directory /input/models/RMUL2024_world/meshes &

ros2 launch nav_test test_nav.launch.py &
ros2 launch stl_visualizer visualize.launch.py &
ros2 run coscene_recorder coscene_recorder &

# ===== Wait for all background jobs =====
wait
>>>>>>> dev/local_zeng:docker/entrypoint.sh
