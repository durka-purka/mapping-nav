# Autonomous Rover - Mapping & Navigation

A ROS 2-based autonomous rover system for the Australian Rover Challenge 2026, integrating SLAM, sensor fusion, and autonomous navigation.

## Overview

This project combines visual SLAM, IMU sensor fusion, and Nav2 navigation stack to enable autonomous mapping and navigation for a mobile rover equipped with:
- **RealSense D435** RGB-D camera for visual odometry and depth sensing
- **MPU6500** IMU for attitude estimation and sensor fusion

## Key Components

| Component | Purpose |
|-----------|---------|
| **RTAB-Map SLAM** | Visual odometry and loop closure detection |
| **EKF Sensor Fusion** | Fuses visual odometry with IMU data via robot_localization |
| **Nav2 Stack** | Autonomous navigation, path planning, and obstacle avoidance |
| **TF Tree** | Transforms between map, odometry, and sensor frames |

## Quick Start

### Prerequisites
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build --packages-select navigation ros2_mpu6500 realsense2_camera
```

### Launch Sequence
1. **Terminal 1** - Start SLAM & sensor fusion:
   ```bash
   ros2 launch your_package mapping_launch.py
   ```
   
2. **Terminal 2** - Start Nav2 stack:
   ```bash
   ros2 launch your_package navigation_launch.py
   ```

3. **Terminal 3** - (Optional) High-level autonomy:
   ```bash
   ros2 launch your_package nav2_complete.launch.py
   ```

## Architecture

```
Sensors → RTAB-Map SLAM → Sensor Fusion (EKF) → Nav2 Navigation Stack
              ↓                    ↓
         Visual Odometry    Filtered Odometry
```

## Key Topics

- **Sensor Data**: `/camera/camera/*` (RGB-D), `/imu/data` (raw IMU)
- **Odometry**: `/rtabmap/odom` (visual), `/odometry/filtered` (fused)
- **Map**: `/rtabmap/grid_map`, `/map`
- **Navigation**: `/cmd_vel` (velocity commands)

## Documentation

- [ARCHITECTURE.md](ARCHITECTURE.md) - Detailed system design
- [RUN_INSTRUCTIONS.md](RUN_INSTRUCTIONS.md) - Complete launch guide
- [COMPATIBILITY_ANALYSIS.md](COMPATIBILITY_ANALYSIS.md) - Component compatibility

## Status

Unfortunately, current implementation needs improvement.
