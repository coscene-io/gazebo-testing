# TurtleBot3 Robot Navigation Package (ROS2 Humble)

## 📖 Introduction  
This repository provides a TurtleBot3 robot simulation environment and navigation stack based on:  
✅ Ubuntu 22.04 (x86)  
✅ ROS2 Humble  
✅ Gazebo 11  
✅ Navigation2  

*Prerequisites*: All listed components must be installed before use.

### Core Features  
- **Map Building**  
  Uses `RMUL2024_world.world` environment from Robot Master competition  
- **Autonomous Navigation Loop**  
  - Executes navigation through 10 randomly selected waypoints  
  - Predefined waypoints stored in YAML configuration (20-point pool)  
- **Test Reporting**  
  Generates comprehensive `test_report` after navigation tasks  

---

## 🚧 Current TODO List

### High Priority
1. **Topic Enhancement** 🔄 Developing  
   - [x] Add support for `/plan` visualization topics
   ```
      use /received_global_plan instead. /plan doesn't contain frame.id in every pose. 
   ```
   - [ ] Implement 3D map topics (OctoMap/PointCloud2)
   - [ ] Integrate camera topics (`/camera/rgb/image_raw`)

2. **System Lifecycle** ⚙️ Planned  
   - [ ] Graceful shutdown procedure for nav_test
   ```python
   # Proposed shutdown sequence
   def shutdown_handler(signum, frame):
       stop_navigation()
       destroy_nodes()
       sys.exit(0)
   ```
   - [ ] Automatic cleanup of dependent nodes, make sure mcap is finalized and indexed.

3. **Test Reporting** 📊 In Progress  
   - [x] JSON report generation (current)
   - [ ] JUnit XML report format from pytest
   - [ ] Key moment markers in reports
   ```xml
   <!-- JUnit example -->
   <testcase name="NavigationToPoint2" status="FAILED">
     <failure message="Timeout exceeded"/>
   </testcase>
   ```

### Load from data folder
4. **Test Case extracted** ✅ Completed  
   ```bash
   # Single directory mounting
   docker run -v ./coscene/records/case1:/cos/files ...
   ```
   Directory structure:
   ```
   /cos/files
   ├── maps/
   ├── models/
   ├── worlds/
   └── config/
   ```

5. **Platform Integration** 🔗 Testing  
   - [ ] coBridge service exposure on regression test platform
 - currently it's exposed from docker
   ```yaml
   # docker-compose.yml
   cobridge:
     ports:
       - "21274:21274"
   ```
   - [ ] One-click playback in coStudio

---

## 🛠️ Usage Guide

### 1. Setup Environment
```bash
# Clone repository
git clone <repo-url>
cd gazebo-testing/ros2_ws

# Build package
colcon build
```

### 2. Environment Configuration
```bash
ROS_WORKSPACE="~/gazebo-testing/ros2_ws/"

# ROS2 Environment
source /opt/ros/humble/setup.bash

# TurtleBot3 Configuration
source ${ROS_WORKSPACE}/install/setup.bash
export ROS_DOMAIN_ID=30  # TURTLEBOT3 ID
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${ROS_WORKSPACE}/src/turtlebot3_simulations/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle_pi

# Workspace Setup
source ${ROS_WORKSPACE}/install/setup.bash
```

### 3. Launch Simulation
```bash
# Start Gazebo world
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# beware of this bug: https://github.com/ROBOTIS-GIT/turtlebot3/pull/916
# the fix is in the devel branch but doesn't seems to be released to the apt source
# (In new terminal) Configure navigation
cp -r sim_map ~/desired_location
cd ~/desired_location/20250314
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true map:=normal.yaml

# (In new terminal) localization
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '
{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: "map"
  },
  pose: {
    pose: {
      position: {x: 0.00392, y: -0.0045, z: 0.191},                       
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}                     
    },
    covariance: [
      0.25, 0.0, 0.0, 0.0, 0.0, 0.0,                          
      0.0, 0.25, 0.0, 0.0, 0.0, 0.0,           
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.06853891945200942, 0.0, 0.0,                     
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0
    ]
  }
}' --once


# (In new terminal) Start test suite
ros2 launch nav_test test_nav.launch.py

# (In new terminal) Data recording
python3 ros_bag_recorder.py
```

---

## ⚠️ Important Notes
1. **Data Recording Configuration**  
   - Modify `ros_bag_recorder.py` to:  
     - Set custom save directory  (RECORD_DIR = "/home/qingyu/turtlebot3_ws_my/bag") 
     - Select ROS2 topics for recording  
   - Output format: MCAP files (20MB/file limit)  

2. **System Requirements**  
   Ensure proper installation of:  
   - ROS2 Humble  
   - TurtleBot3 packages  
   - Navigation2 stack  

3. **Waypoint Customization**  
   Edit the `test_points.yaml` file to modify navigation targets (/home/username/folder/gazebo-testing/turtlebot3_ws_my/src/nav_test/config)
