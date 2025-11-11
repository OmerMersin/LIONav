# 🦁 LIONav

**LIONav (LiDAR-Inertial Odometry Navigation)** is a ROS 2 workspace for autonomous navigation and Return-to-Launch (RTL) in **GPS-denied environments** such as caves, mines, or indoor facilities.

It combines **LIO-SAM**, **MAVROS**, and **Ouster LiDAR** to provide accurate real-time localization using LiDAR and IMU data.

---

## 📦 Main Components
- **LIO-SAM** – LiDAR-IMU odometry and mapping backend  
- **MAVROS** – IMU and flight-controller interface (CUAV V7 Nano)  
- **Ouster ROS** – LiDAR driver (OS0-32/64)  
- **utils** – Helper nodes for time sync, outlier filtering, and data relaying  

---

## 🚀 Quick Start

```bash
# Clone and build
git clone https://github.com/OmerMersin/LIONav.git
cd LIONav/ros2_ws
colcon build
source install/setup.bash

# Run Ouster LiDAR
ros2 launch ouster_ros sensor.launch.xml sensor_hostname:=os-xxxx.local timestamp_mode:=TIME_FROM_ROS_TIME

# Run MAVROS (adjust serial port)
ros2 run mavros mavros_node --ros-args -p fcu_url:=serial:///dev/ttyTHS1:921600 -p use_companion_time:=true

# Start LIO-SAM
ros2 launch lio_sam run.launch.py
```

---

## 🧭 Description
LIONav performs:
- Real-time LiDAR-IMU fusion  
- Mapping and localization without GPS  
- Support for Return-to-Launch trajectories  
- Runtime LiDAR noise filtering and timestamp matching  

---

## 📂 Structure
```
LIONav/
├── ros2_ws/           # ROS2 workspace (LIO-SAM, Ouster-ROS, etc.)
└── utils/             # Python nodes for time sync & filtering
```

---

## ⚙️ Requirements
- Ubuntu 22.04 / ROS 2 Humble  
- GTSAM 4.2+  
- MAVROS  
- Open3D (for optional filtering)

---

## 🪪 License
MIT License © 2025 Ömer Mersin  
