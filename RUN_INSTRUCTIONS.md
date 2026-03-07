# ROS 2 Launch Instructions - Australian Rover Challenge 2026

## Prerequisites

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select navigation ros2_mpu6500 realsense2_camera
source install/setup.bash
```

## System Architecture

### Launch Files Execution Order

1. **mapping_launch.py** (provides SLAM + sensor fusion)
2. **navigation_launch.py** (provides Nav2 navigation stack)
3. **nav2_complete.launch.py** (provides high-level autonomy)

---

## Running the System

### Terminal 1: Start SLAM + Sensor Fusion

```bash
ros2 launch your_package mapping_launch.py
```

**Outputs:**
- RealSense D435 RGB-D camera streams
- MPU6500 IMU data
- RTAB-Map visual odometry (`/rtabmap/odom`)
- EKF sensor fusion (`/odometry/filtered`)
- RTAB-Map grid (`/rtabmap/grid_map`)
- TF tree (map, base_link, camera_link, imu_link, etc.)

**Wait 10 seconds for RTAB-Map to initialize**

---

### Terminal 2: Start Nav2 Navigation Stack

```bash
ros2 launch your_package navigation_launch.py
```

**Outputs:**
- Map relay (RTAB-Map → Nav2)
- Nav2 costmap servers (global + local)
- Nav2 planners, controllers, behavior servers
- Navigate_to_pose action server

**Status:**
```bash
ros2 topic list | grep nav
ros2 action list
```

Expected to see `/navigate_to_pose` action server available.

---

### Terminal 3: Start Autonomous Mission

First, create a landmarks file:

```bash
cat > landmarks.yaml << 'EOF'
landmarks:
  - id: 1
    name: "Landmark Alpha"
    position_x: 2.0
    position_y: 0.0
    height_above_ground: 0.5
  - id: 2
    name: "Landmark Beta"
    position_x: 2.0
    position_y: 2.0
    height_above_ground: 0.5
EOF
```

Then launch:

```bash
ros2 launch navigation nav2_complete.launch.py \
  autonomous_phase:=true \
  landmarks_file:=$(pwd)/landmarks.yaml \
  record_rosbag:=true
```

**Outputs:**
- Autonomous Phase Manager (monitors phase transitions)
- Start Frame Reference Manager (creates start_frame)
- Waypoint Handler (landmark navigation)
- Frontier Explorer (cube search)
- Collision Recovery (stuck detection)
- Object Reporter (coordinate transformations)
- ROSBAG recording

---

### Terminal 4: Visualize in RViz

```bash
rviz2 -d src/navigation/rviz/navigation.rviz
```

**Expected Displays:**
- Global costmap with obstacles
- Local costmap (3m × 3m rolling window)
- Planned path to goal
- Frontier visualization markers
- TF tree (map, base_link, start_frame)

---

### Terminal 5: Monitor System Status

```bash
# Check phase transitions
ros2 topic echo autonomous_phase/status

# Monitor landmark progress
ros2 topic echo waypoint_handler/status

# Watch detected cubes
ros2 topic echo object_reporter/cube_report

# Check collision events
ros2 topic echo collision_recovery/status

# Network diagnostics
ros2 topic hz /odometry/filtered
ros2 topic bw /camera/camera/depth/color/points
```

---

## Key Topic Names (Updated for Launch Files)

### Sensor Topics (from mapping_launch.py)

| Topic | Type | Frame | Source |
|-------|------|-------|--------|
| `/imu/data` | IMU | imu_link | MPU6500 raw data |
| `/imu/data_filtered` | IMU | imu_link | Madgwick filter |
| `/camera/camera/color/image_raw` | Image | camera_color_optical_frame | RealSense RGB |
| `/camera/camera/aligned_depth_to_color/image_raw` | Image | camera_depth_optical_frame | RealSense depth |
| `/camera/camera/depth/color/points` | PointCloud2 | camera_depth_optical_frame | RealSense PointCloud |

### Odometry Topics (from mapping_launch.py)

| Topic | Type | Owner | Usage |
|-------|------|-------|-------|
| `/rtabmap/odom` | Odometry | RTAB-Map | Visual odometry input to EKF |
| `/odometry/filtered` | Odometry | EKF (robot_localization) | **Used by all navigation nodes** |

### Map Topics (from navigation_launch.py)

| Topic | Type | Source | Usage |
|-------|------|--------|-------|
| `/map` | OccupancyGrid | map_relay | Nav2 costmap from RTAB-Map |
| `/global_costmap/costmap` | OccupancyGrid | nav2_costmap_2d | Global path planning |
| `/local_costmap/costmap` | OccupancyGrid | nav2_costmap_2d | Local obstacle avoidance |
| `/rtabmap/grid_map` | OccupancyGrid | RTAB-Map | Original SLAM map |

### Frame Names (from mapping_launch.py)

| Frame | Parent | Broadcast By | Description |
|-------|--------|--------------|-------------|
| map | — | RTAB-Map EKF | Global reference (SLAM origin) |
| odom | — | RTAB-Map | Odometry frame (SLAM output) |
| base_link | odom | EKF TF | Robot center |
| camera_link | base_link | static_transform_publisher | Camera mount |
| imu_link | base_link | static_transform_publisher | IMU mount |
| camera_depth_frame | camera_link | static_transform_publisher | Depth sensor frame |
| camera_depth_optical_frame | camera_depth_frame | static_transform_publisher | Depth optical frame |
| camera_color_optical_frame | camera_color_frame | static_transform_publisher | Color optical frame |
| start_frame | map | start_frame_reference.py | Robot's starting position (custom) |

---

## Code-to-Launch Mapping

### autonomous_phase_manager.py

**Subscribes to:**
- `/odometry/filtered` → Detects first movement

**Publishes:**
- `/autonomous_phase/status` → Phase transitions
- `/autonomous_phase/events` → Audit log

**Launched by:** nav2_complete.launch.py

---

### start_frame_reference.py

**Subscribes to:**
- `/initialpose` → Sets robot's initial pose

**Broadcasts:**
- `/tf_static` → Creates map → start_frame transform

**Launched by:** nav2_complete.launch.py

---

### waypoint_handler.py

**Subscribes to:**
- `/odometry/filtered` → Current robot pose
- `/perception/placard_text` → Detected text at landmarks

**Publishes to:**
- `/navigate_to_pose` action → Nav2 goals

**Launched by:** nav2_complete.launch.py

---

### frontier_exploration.py

**Subscribes to:**
- `/global_costmap/costmap` → frontier detection
- `/odometry/filtered` → Current pose
- `/perception/cube_detections` → Found cubes

**Publishes to:**
- `/navigate_to_pose` action → Explore frontiers

**Launched by:** nav2_complete.launch.py

---

### collision_recovery.py

**Subscribes to:**
- `/odometry/filtered` → Stuck detection

**Publishes to:**
- `/cmd_vel` → Recovery maneuvers (backup, spin)
- Recovery actions (backup, spin) from nav2_behaviors

**Launched by:** nav2_complete.launch.py

---

### object_reporter.py

**Subscribes to:**
- `/perception/cube_detections` → input (map frame)
- `/perception/landmark_detections` → input (map frame)

**Uses TF to transform:**
- map → start_frame (created by start_frame_reference.py)

**Publishes:**
- `/object_reporter/cube_report` → output (start_frame)
- `/object_reporter/landmark_report` → output (start_frame)

**Launched by:** nav2_complete.launch.py

---

## Troubleshooting

### Issue: "Package not found"
```bash
colcon build --packages-select navigation
source install/setup.bash
```

### Issue: "No action server /navigate_to_pose"
- Check Terminal 2 Nav2 launch completed
- Verify: `ros2 action list` shows `/navigate_to_pose`

### Issue: "TF frame [start_frame] does not exist"
- Check Terminal 1 mapping_launch.py is running
- Check Terminal 3 nav2_complete.launch.py started autonomous_phase_manager

### Issue: Low odometry update rate
```bash
ros2 topic hz /odometry/filtered
# Should be 50 Hz (EKF frequency from mapping_launch.py)
```

### Issue: No depth camera PointCloud
```bash
ros2 topic list | grep points
# Should see: /camera/camera/depth/color/points
```

---

## Configuration Files Updated for Launch Compatibility

- **nav2_params.yaml** ✓ Removed AMCL (laser-based), using EKF odometry
- **costmap_params.yaml** ✓ Depth camera as observation source only
- **collision_recovery.py** ✓ No LIDAR checking (RealSense depth only)
- **All topic names** ✓ Updated to match launch file outputs

---

## Final Verification

Before running mission:

```bash
# 1. Check all nodes running
ros2 node list | wc -l
# Should be 15+ nodes

# 2. Check critical topics
ros2 topic list | grep -E "odometry/filtered|costmap|cmd_vel|navigate_to_pose"

# 3. Verify action servers
ros2 action list
# Should include: /navigate_to_pose

# 4. Check TF tree
ros2 run tf2_tools view_frames
# Should include: map, base_link, start_frame, camera_*

# 5. Monitor CPU/Memory
top
# Should see ros2 processes using 5-20% CPU
```

Ready to launch! 🚀
