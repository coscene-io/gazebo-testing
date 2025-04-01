#!/bin/bash

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