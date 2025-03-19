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

# beware of this bug: https://github.com/ROBOTIS-GIT/turtlebot3/pull/916a
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

4. **Workspace Directory Structure**
```
gazebo-testing/
├── docker-compose.yml
├── Dockerfile
├── launch_nav2.sh
├── record_topics.sh
├── ros_entrypoint.sh
├── start_kasm.sh
└── turtlebot3_ws_my
    ├── ros_bag_recorder.py
    ├── sim_map
    │   └── 20250314
    │       ├── normal.pgm
    │       └── normal.yaml
    └── src
        ├── DynamixelSDK
        │   ├── CONTRIBUTING.md
        │   ├── Doxyfile
        │   ├── dynamixel_sdk
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   ├── include
        │   │   ├── package.xml
        │   │   └── src
        │   ├── dynamixel_sdk_custom_interfaces
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   ├── msg
        │   │   ├── package.xml
        │   │   └── srv
        │   ├── dynamixel_sdk_examples
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   ├── include
        │   │   ├── package.xml
        │   │   └── src
        │   ├── LICENSE
        │   └── README.md
        ├── nav_loop.py
        ├── nav_test
        │   ├── config
        │   │   └── test_points.yaml
        │   ├── launch
        │   │   └── test_nav.launch.py
        │   ├── nav_test
        │   │   ├── __init__.py
        │   │   └── nav_controller.py
        │   ├── package.xml
        │   ├── resource
        │   │   └── nav_test
        │   ├── setup.cfg
        │   └── setup.py
        ├── ros_bag_recorder.py
        ├── turtlebot3
        │   ├── CONTRIBUTING.md
        │   ├── ISSUE_TEMPLATE.md
        │   ├── LICENSE
        │   ├── README.md
        │   ├── turtlebot3
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   └── package.xml
        │   ├── turtlebot3_bringup
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   ├── launch
        │   │   ├── package.xml
        │   │   ├── param
        │   │   └── script
        │   ├── turtlebot3_cartographer
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   ├── config
        │   │   ├── launch
        │   │   ├── package.xml
        │   │   └── rviz
        │   ├── turtlebot3_ci.repos
        │   ├── turtlebot3_description
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   ├── meshes
        │   │   ├── package.xml
        │   │   ├── rviz
        │   │   └── urdf
        │   ├── turtlebot3_example
        │   │   ├── CHANGELOG.rst
        │   │   ├── package.xml
        │   │   ├── resource
        │   │   ├── setup.cfg
        │   │   ├── setup.py
        │   │   └── turtlebot3_example
        │   ├── turtlebot3_navigation2
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   ├── launch
        │   │   ├── map
        │   │   ├── package.xml
        │   │   ├── param
        │   │   └── rviz
        │   ├── turtlebot3_node
        │   │   ├── CHANGELOG.rst
        │   │   ├── CMakeLists.txt
        │   │   ├── include
        │   │   ├── package.xml
        │   │   ├── param
        │   │   └── src
        │   ├── turtlebot3.repos
        │   └── turtlebot3_teleop
        │       ├── CHANGELOG.rst
        │       ├── package.xml
        │       ├── resource
        │       ├── setup.cfg
        │       ├── setup.py
        │       └── turtlebot3_teleop
        ├── turtlebot3_msgs
        │   ├── action
        │   │   └── Patrol.action
        │   ├── CHANGELOG.rst
        │   ├── CMakeLists.txt
        │   ├── LICENSE
        │   ├── msg
        │   │   ├── SensorState.msg
        │   │   ├── Sound.msg
        │   │   └── VersionInfo.msg
        │   ├── package.xml
        │   ├── README.md
        │   └── srv
        │       ├── Dqn.srv
        │       └── Sound.srv
        └── turtlebot3_simulations
            ├── CONTRIBUTING.md
            ├── LICENSE
            ├── README.md
            ├── turtlebot3_fake_node
            │   ├── CHANGELOG.rst
            │   ├── CMakeLists.txt
            │   ├── include
            │   ├── launch
            │   ├── package.xml
            │   ├── param
            │   └── src
            ├── turtlebot3_gazebo
            │   ├── CHANGELOG.rst
            │   ├── CMakeLists.txt
            │   ├── include
            │   ├── launch
            │   ├── models
            │   ├── package.xml
            │   ├── rviz
            │   ├── src
            │   ├── urdf
            │   └── worlds
            ├── turtlebot3_simulations
            │   ├── CHANGELOG.rst
            │   ├── CMakeLists.txt
            │   └── package.xml
            └── turtlebot3_simulations_ci.repos
```
