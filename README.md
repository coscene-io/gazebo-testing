# TurtleBot3 Robot Navigation Test  Repo

## 📖 Introduction  
This repository provides a TurtleBot3 robot simulation environment and navigation stack based on:  
✅ Ubuntu 22.04 (x86)  
✅ ROS2 Humble  
✅ Gazebo 11  
✅ Navigation2  

*Prerequisites*: 
1. You must provide a base image which contains Ubuntu 22.04 (x86) & ROS2 Humble. We recommand osrf/ros:humble-desktop-full.
2. If you need run this repo locally, please make sure you have the docker environment on your pc. 

### Core Features  
- **Map Building**  
  Uses `RMUL2024_world.world` environment from Robot Master competition  
- **Autonomous Navigation Loop**  
  - Executes navigation through 10 randomly selected waypoints  
  - Predefined waypoints stored in YAML configuration (20-point pool)  
- **Test Reporting**  
  Generates comprehensive `test_report` after navigation tasks  
- **Provide online & offline Visualization function**
  Follow the guide from  https://docs.coscene.cn/docs/client/connect-by-cobridge  part 3
  We provide a recommendation layout. in /coscene/layout

---

## 🛠️ Usage Guide
### Function difference

|Function| On-line | Off-line |
|----------|----------|-------------|
| Map Building | Build map in RMUL2024 world | Build map in RMUL2024 world |
| Autonomous Navigation | Automatically navigate through 10 waypoints | Automatically navigate through 10 waypoints |
| Waypoint Pool | Predefined 20 navigation points(can be edit before test) | Predefined 20 navigation points(can be edit before test) |
| Test Reporting | Generate navigation test reports | Generate navigation test reports |
| Visualization | Connect via CoBridge to view in real-time(auto) | Connect via CoBridge to view in real-time(manual select) |
| Gazebo Simulation | Full physics simulation with Gazebo 11(headless) | Full physics simulation with Gazebo 11(headless) |
| YAML Configuration | Configure test parameters(test points and initial pose of robot) via YAML | Configure test parameters(test points and initial pose of robot) via YAML |
| MCAP Recording | Record data as MCAP files (5Mins per file) | Record data as MCAP files (5Mins per file) |
| Custom Recorder | Supports custom topic selection for recording | Supports custom topic selection for recording |
| Dockerized Environment | Not Necessary | Fully containerized test environment |

### Local Usage
#### 1. Install Docker on your local PC

Follow the official guide to install Docker:
[Docker installation guide](https://docs.docker.com/get-docker/)

After installation, verify by running:

```bash
docker run hello-world
```

This command pulls a minimal Docker image and runs a test container. If you see a hello message, Docker works correctly.

#### 2. Clone the repository

```bash
git clone <repo-url>
cd gazebo-testing/
```

#### 3. Build and run the project

```bash
docker compose up --build
```

This will automatically build and run the whole repo.

#### 4. Install CoStudio

Download and install CoStudio to your local PC. You can find the installation guide here:

[CoStudio Download & Install](https://docs.coscene.cn/docs/client/connect-by-cobridge)

#### 5. Open CoStudio and connect locally

- Select **Local Connection**.
- Import the layout file from `/coscene/layout` in this repo.
- You can now observe the simulation visually.


### Online Usage

## 📌 Project CI/CD and Docker Workflow Overview

This project includes a seamless pipeline to manage code building, testing, Docker image generation, and usage in local or remote environments.

### 🔧 Workflow Overview

```plaintext
Git push / PR Merge
        │
        ▼
GitHub Actions Triggered
 ┌─────────────────────┬──────────────────────────┐
 │ colcon-build.yml    │ docker-build-push.yml    │
 │ Build & Test Source │ Build & Push Docker Image│
 └─────────────────────┴──────────────────────────┘
        │
        ▼
Docker image pushed to cr.coscene.cn
        │
        ▼
Users pull and run the Docker image via docker-compose
        │
        ▼
CoStudio connects (local or remote) for visualization
```

---

### 🚀 Components Explained

| Component | File | Role |
|-----------|------|------|
| **Build & Test ROS2 Workspace** | `.github/workflows/colcon-build.yml` | Internal CI: Builds source, runs tests, generates `install` artifact |
| **Build & Publish Docker Image** | `.github/workflows/docker-build-push.yml` | Production use: Builds Docker image and pushes to private registry |
| **Authentication Secrets** | `.secrets` (or `.secrets.example`) | Secure storage for Docker registry and CoScene API credentials |

---

### 🧰 User Operations

You can:
- Edit navigation waypoints via `./coscene/records/case1/config/test_points.yaml`.
- Change data recording parameters in `./coscene_recorder/config`.
- Customize layouts for visualization in `/coscene/layout`.


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
