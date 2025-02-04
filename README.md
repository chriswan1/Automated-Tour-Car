# Automated Tour Car - LiDAR Workspace  
This repository contains ROS 2 packages for the **Automated Tour Car** project. The workspace is built using **ROS 2 Humble**.

---

## Workspace Structure
```
Automated-Tour-Car/
│── ws_lidar/                # ROS 2 workspace
│   ├── src/                 # Source code for packages
│   │   ├── lidar/           # LiDAR scanning and visualization package
│   │   ├── object_avoidance/ # Object detection & avoidance logic
│   │   ├── sllidar_ros2/    # Driver for LiDAR sensor
│   ├── install/             # Built package files
│   ├── build/               # Compiled binaries
│   ├── log/                 # ROS 2 logs
```

---

## Packages Overview
### 1. `lidar`
- Handles **LiDAR data processing**.
- Publishes scan data to `/scan` for other nodes.
- Uses **rqt, RViz** for visualization.

### 2. `object_avoidance`
- Implements **obstacle detection & avoidance**.
- Subscribes to `/scan` and processes **distance data**.
- Publishes movement commands to a `/cmd_vel` topic.

### 3. `sllidar_ros2`
- Provides **ROS 2 driver support** for the **WayPonDEV LD14P LiDAR**.
- Reads LiDAR sensor data and publishes to `/scan`.

---

## Installation & Setup
### 1. Clone the Repository
```bash
git clone https://github.com/your-username/Automated-Tour-Car.git
cd Automated-Tour-Car/ws_lidar
```

### 2. Build the Workspace
```bash
colcon build
source install/setup.bash
```

---

## Running the Packages
### Start the LiDAR Driver
```bash
ros2 launch sllidar_ros2 sllidar.launch.py
```

### Run Lidar Publisher Node
```bash
ros2 run lidar lidar_publisher
```

### Run Lidar Subscriber Node
```bash
ros2 run lidar lidar_subscriber
```

### Run Object Avoidance Publisher Node
```bash
ros2 run object_avoidance publisher
```

### Run Object Avoidance Subscriber Node
```bash
ros2 run object_avoidance subscriber
```

### Visualize LiDAR Data
```bash
rviz2
```
