# Pre-Mission Checklist

## Before Running Any Launch Files

### Hardware Checklist
- [ ] RealSense D435 camera connected via USB
- [ ] MPU6500 IMU connected via I2C (`i2cdetect -y 1` shows 0x68 or 0x69)
- [ ] Robot base controller connected and ready
- [ ] Battery charged to 100%
- [ ] All mechanical systems functional (wheels, motors, etc.)

### Software Checklist
- [ ] Ubuntu 22.04 (Jammy) with ROS 2 Humble installed
- [ ] ROS 2 workspace built: `colcon build --packages-select navigation ros2_mpu6500 realsense2_camera`
- [ ] Workspace sourced: `source ~/ros2_ws/install/setup.bash`
- [ ] No stray ROS nodes: `killall -9 ros2` if needed

---

## Terminal 1: Mapping/SLAM Verification

```bash
# Launch SLAM
ros2 launch your_package mapping_launch.py
```

**Wait 10 seconds, then check in another terminal:**

### Verify Sensors
```bash
# RealSense camera
ros2 topic echo /camera/camera/color/image_raw --once
# Should show: Image with width=640, height=480, encoding=rgb8

# MPU6500 IMU
ros2 topic echo /imu/data --once
# Should show: linear_acceleration and angular_velocity values

# RealSense depth PointCloud
ros2 topic list | grep points
# Should see: /camera/camera/depth/color/points
```

### Verify Odometry
```bash
# Check if EKF is running
ros2 topic hz /odometry/filtered
# Should show: ~50 Hz (configured in mapping_launch.py)

# Check one odometry message
ros2 topic echo /odometry/filtered --once
# Should show: pose with x, y, z position; pose_covariance reasonable values
```

### Verify SLAM Map
```bash
# Check RTAB-Map is building map
ros2 topic echo /rtabmap/grid_map --once
# Should show: OccupancyGrid with width/height > 0, data contains map cells
```

### Verify TF Tree
```bash
# Check all frames exist
ros2 run tf2_tools view_frames
# Should create frames.pdf with tree showing: map, odom, base_link, camera_link, imu_link
```

**✓ Mapping launch checklist complete - PROCEED TO TERMINAL 2**

---

## Terminal 2: Nav2 Navigation Stack Verification

```bash
# Launch Nav2
ros2 launch your_package navigation_launch.py
```

**Wait 5 seconds, then check in another terminal:**

### Verify Map Relay
```bash
# Check map is being relayed
ros2 topic echo /map --once
# Should show: OccupancyGrid similar to /rtabmap/grid_map
```

### Verify Costmaps
```bash
# Global costmap
ros2 topic echo /global_costmap/costmap --once
# Should show: OccupancyGrid with robot footprint

# Local costmap
ros2 topic echo /local_costmap/costmap --once
# Should show: 3x3m rolling window around robot
```

### Verify Nav2 Servers
```bash
# Check planner server
ros2 topic echo /plan --once
# (May be empty until first goal sent)

# Check controller is ready
ros2 node info /controller_server
# Should show: Services like /controller_server/list_controllers

# Check if action server exists
ros2 action list
# MUST see: /navigate_to_pose
```

**✓ Navigation launch checklist complete - PROCEED TO TERMINAL 3**

---

## Terminal 3: Autonomous Mission Stack Verification

### Create Landmarks File
```bash
cat > landmarks.yaml << 'EOF'
landmarks:
  - id: 1
    name: "Test Landmark 1"
    position_x: 1.0
    position_y: 0.0
    height_above_ground: 0.5
  - id: 2
    name: "Test Landmark 2"
    position_x: 1.0
    position_y: 1.0
    height_above_ground: 0.5
EOF
```

### Launch Autonomy Stack
```bash
ros2 launch navigation nav2_complete.launch.py \
  autonomous_phase:=true \
  landmarks_file:=$(pwd)/landmarks.yaml \
  record_rosbag:=true
```

**Wait 5 seconds, then check in another terminal:**

### Verify Phase Manager
```bash
# Check initial phase
ros2 topic echo autonomous_phase/status --once
# Should show: {"phase": "INITIALIZATION", ...}

# Check audit log was created
ls -l /tmp/arc2026_phase_audit.json
# File should exist with size > 0
```

### Verify Start Frame Reference
```bash
# Check if start_frame exists in TF tree
ros2 run tf2_tools view_frames
# frames.pdf should show: map → start_frame (new frame)

# Verify transform
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id start_frame
# Should show: Received static transform (not an error)
```

### Verify Waypoint Handler
```bash
# Check landmarks loaded
ros2 topic echo waypoint_handler/status --once
# Status should mention number of landmarks loaded

# Check navigation state
ros2 topic echo waypoint_handler/current_landmark --one
# Should show landmark index (probably -1 or 0 before starting)
```

### Verify Frontier Explorer
```bash
# Check frontier explorer status
ros2 topic echo frontier_explorer/status --once
# Should show: initialized, ready to detect frontiers
```

### Verify Collision Recovery
```bash
# Check recovery handler status
ros2 topic echo collision_recovery/status --once
# Should be empty until collision occurs (normal)
```

### Verify Object Reporter
```bash
# Check if transformation is cached
ros2 service call /start_frame_reference/initialize std_srvs/srv/SetBool '{}' 
# Should initialize start_frame

# Verify transform lookup works
ros2 run tf2_tools view_frames
# Confirm start_frame → parent is map
```

### Verify ROSBAG Recording
```bash
# Check rosbag is recording
ls -lh /tmp/arc2026_*/
# Directory should exist with index.db3 growing in size

# Monitor rosbag size
watch -n 1 'du -sh /tmp/arc2026_*/'
# Size should increase over time
```

**✓ Autonomy launch checklist complete - READY FOR MISSION**

---

## Pre-Movement Verification (Critical!)

Before allowing robot to move, verify all compliance systems:

```bash
# 1. Check phase is INITIALIZATION (not AUTONOMOUS yet)
ros2 topic echo autonomous_phase/status
# MUST be: "phase": "INITIALIZATION"

# 2. Check landmarks loaded correctly
ros2 topic echo waypoint_handler/status | grep landmarks
# Should show: "X landmarks loaded"

# 3. Check start_frame was created
ros2 param get /start_frame_reference initialized
# Should be: True

# 4. Check odometry is updating
ros2 topic hz /odometry/filtered
# MUST be: ~50 Hz

# 5. Check no issues in logs
ros2 node list | wc -l
# Should have 15+ nodes running

# 6. Check costmap is active
ros2 topic hz /global_costmap/costmap
# MUST be: > 0.5 Hz
```

---

## Motion Sequence Verification

### 1. Initial Position
```bash
# Robot should be at origin (0, 0) in map frame
ros2 topic echo /odometry/filtered --once | grep "position"
# x, y, z should be ~0

# Verify this matches start_frame
ros2 topic echo /tf_static
# start_frame translation should match initial odometry
```

### 2. First Movement Trigger
```bash
# User manually moves robot forward ~1 meter
# Observer watches odometry and phase

# On separate terminal:
ros2 topic echo autonomous_phase/status
# Watch for transition: INITIALIZATION → AUTONOMOUS

# Check audit log
cat /tmp/arc2026_phase_audit.json | python3 -m json.tool | grep "AUTONOMOUS"
# Should see phase change event with timestamp and pose
```

### 3. First Goal Navigation
```bash
# After phase changes to AUTONOMOUS, trigger first landmark
# Monitor progress:
ros2 topic echo waypoint_handler/status
# Should show: "Navigating to landmark 0"

# Check nav goal
ros2 topic echo /goal_pose
# Should match first landmark position

# Monitor odometry
ros2 topic hz /odometry/filtered
# Should drop slightly during heavy computation (>30 Hz minimum)
```

---

## Real-Time Monitoring Commands

Run these in separate terminals during mission:

```bash
# Terminal A: Current position
watch -n 0.5 'ros2 topic echo /odometry/filtered --once | grep -A2 position'

# Terminal B: Phase status
watch -n 1 'ros2 topic echo autonomous_phase/status'

# Terminal C: Waypoint progress
watch -n 1 'ros2 topic echo waypoint_handler/status'

# Terminal D: Collision events
ros2 topic echo collision_recovery/status

# Terminal E: Detected objects
ros2 topic echo object_reporter/cube_report

# Terminal F: System health
watch -n 1 'ros2 node list | wc -l'
```

---

## Post-Mission Checklist

```bash
# 1. Kill all launch files (Ctrl+C in each terminal)

# 2. Save rosbag
cp -r /tmp/arc2026_* ~/arc2026_runs/

# 3. Extract audit log
cat /tmp/arc2026_phase_audit.json | python3 -m json.tool > audit_log.json

# 4. Get final reports
ros2 topic echo object_reporter/cube_report > final_detections.txt
ros2 topic echo collision_recovery/status > collision_report.txt

# 5. Verify all data
ls -lh ~/arc2026_runs/
ls -lh /tmp/arc2026_*

# 6. Check ROS cleanup
ros2 node list
# Should be empty or very few nodes
```

---

## Troubleshooting Quick Links

| Issue | Terminal Command | Expected Result |
|-------|------------------|-----------------|
| No /odometry/filtered | `ros2 topic list \| grep odom` | Should see /odometry/filtered |
| TF tree broken | `ros2 run tf2_tools view_frames` | frames.pdf generated successfully |
| Nav2 not responding | `ros2 action list` | /navigate_to_pose in list |
| Landmarks not loaded | `ros2 param get waypoint_handler landmarks` | Shows loaded landmark list |
| Start frame not created | `ros2 service call ... SetBool '{}'` | Returns True |
| Rosbag not recording | `ls -l /tmp/arc2026_*/` | Directory exists with files |
| Low odometry rate | `ros2 topic hz /odometry/filtered` | ~50 Hz |
| Costmap empty | `ros2 topic echo /global_costmap/costmap --once` | Contains obstacle data |

---

## Go/No-Go Decision Criteria

### PROCEED TO MISSION if:
- ✓ All nodes running (15+ nodes)
- ✓ /odometry/filtered publishing at 50 Hz
- ✓ /map contains valid costmap
- ✓ /navigate_to_pose action server available
- ✓ TF tree complete (map → base_link → camera_link, etc.)
- ✓ start_frame created successfully
- ✓ Landmarks loaded (3+ landmarks)
- ✓ ROSBAG recording active

### DO NOT PROCEED if:
- ✗ Any critical node crashed (check ros2 node list)
- ✗ /odometry/filtered update rate < 30 Hz
- ✗ /map is empty or all occupied
- ✗ TF frames missing
- ✗ Navigation action server not responding
- ✗ Phase manager not initialized

---

## Emergency Stop

```bash
# Kill all ROS processes immediately
killall -9 ros2

# Manually stop robot (hardware e-stop)
# Contact pit crew for assistance
```
