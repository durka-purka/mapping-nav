# System Architecture - Launch File Integration

## Data Flow Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│              SENSOR LAYER (mapping_launch.py)                    │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  RealSense D435              MPU6500 IMU                         │
│  ├─ RGB Image                ├─ Accelerometer                    │
│  ├─ Depth Image              ├─ Gyroscope                       │
│  └─ IR Stereo                └─ Temperature                     │
│       ↓                           ↓                             │
│   /camera/camera/*         /imu/data (raw)                      │
│       ↓                           ↓                             │
│    RTAB-SLAM ←──────────────────→ Madgwick Filter              │
│       │                           │                             │
│       └─────────────────────────→ /imu/data_filtered            │
│                                    ↓                            │
│               ┌────────────────────┼────────────────────┐       │
│               │       EKF Sensor Fusion                 │       │
│               │   (robot_localization)                  │       │
│               │   Inputs:                               │       │
│               │   - /rtabmap/odom (visual odometry)    │       │
│               │   - /imu/data_filtered (attitude)      │       │
│               │   Output:                               │       │
│               │   - /odometry/filtered ← MAIN ODOM     │       │
│               └────────────────────┬────────────────────┘       │
│                                    │                            │
│  TF Tree Broadcasting:             │                            │
│  - map (origin)                    │                            │
│  - odom                            │                            │
│  - base_link ←──────────────────────┘                           │
│  - camera_link, imu_link, etc.                                 │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
         ↓ (/odometry/filtered, /map, /tf, /rtabmap/grid_map)
┌─────────────────────────────────────────────────────────────────┐
│              NAV2 STACK (navigation_launch.py)                   │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  /camera/camera/depth/color/points (PointCloud2)               │
│             ↓                                                    │
│  ┌─────────────────────────────────┐                           │
│  │  Global Costmap Server          │                           │
│  │  - Static Layer (from /map)      │                           │
│  │  - Obstacle Layer (from points)  │                           │
│  │  - Inflation Layer               │                           │
│  │  Output: /global_costmap/costmap │                           │
│  └──────────────┬────────────────────┘                           │
│                 │                                                │
│  ┌──────────────┼────────────────┐                              │
│  │ /odometry/filtered ────────────┤                              │
│  │                                 ↓                            │
│  │  • Planner Server (A*)          │ /local_costmap/costmap    │
│  │  • Controller Server (DWB)      │                           │
│  │  • Behavior Server (backup, spin)  │                        │
│  │  • BT Navigator                    │                        │
│  └────────┬──────────────────────┘                              │
│           │                                                      │
│  /navigate_to_pose action server                               │
│           ↓                                                      │
│  /cmd_vel (to robot base controller)                           │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
         ↓ (nav goals, status, costmaps)
┌─────────────────────────────────────────────────────────────────┐
│         AUTONOMY LAYER (nav2_complete.launch.py)                │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ autonomous_phase_manager.py                               │ │
│  │ • Subscribes: /odometry/filtered (detects first movement) │ │
│  │ • Publishes: /autonomous_phase/{status,events}            │ │
│  │ • Rule: Lock manual control after first robot movement    │ │
│  │ • Audit: Saves phase transitions to JSON log             │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ start_frame_reference.py                                  │ │
│  │ • Subscribes: /initialpose (robot initial position)       │ │
│  │ • Creates TF: map → start_frame (robot origin)            │ │
│  │ • Uses: /odometry/filtered to capture initial pose        │ │
│  │ • Rule: All detections reported relative to start position│ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ waypoint_handler.py                                       │ │
│  │ • Subscribes: /odometry/filtered, /perception/placard_*   │ │
│  │ • Action: /navigate_to_pose (sends landmark goals)        │ │
│  │ • Loads: landmarks.yaml (Activity 2 targets)              │ │
│  │ • Detects arrival and triggers imaging                    │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ frontier_exploration.py                                   │ │
│  │ • Subscribes: /global_costmap/costmap, /odometry/filtered │ │
│  │ • Action: /navigate_to_pose (explores frontiers)          │ │
│  │ • Detects unknown/known boundaries                        │ │
│  │ • Activity 3: Find 4 colored cubes                        │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ collision_recovery.py                                     │ │
│  │ • Subscribes: /odometry/filtered (stuck detection)        │ │
│  │ • Detects: No forward progress for 10+ seconds            │ │
│  │ • Actions: backup, spin (from nav2_behaviors)            │ │
│  │ • Publishes: /collision_recovery/status                  │ │
│  │ • Tracks: Collision penalties (10% per collision)        │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ object_reporter.py                                        │ │
│  │ • Subscribes: /perception/{cube,landmark}_detections      │ │
│  │ • Transforms: map frame → start_frame (compliance)        │ │
│  │ • Uses TF: Looks up start_frame transform                │ │
│  │ • Publishes: /object_reporter/{cube,landmark}_report      │ │
│  │ • Deduplicates: Prevents duplicate detections             │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                   │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ ROSBAG Recording                                          │ │
│  │ • Records ALL topics for post-mission analysis            │ │
│  │ • Output: /tmp/arc2026_TIMESTAMP/                         │ │
│  │ • Topics: odometry, maps, costmaps, commands, events      │ │
│  └────────────────────────────────────────────────────────────┘ │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
         ↓ (reports, judgement-ready audit logs)
┌─────────────────────────────────────────────────────────────────┐
│              JUDGE INTERFACE                                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  /tmp/arc2026_phase_audit.json (autonomous_phase_manager)      │
│  ├─ Phase transitions (INITIALIZATION → AUTONOMOUS → etc)      │
│  ├─ Manual control attempts (should be 0 after first move)     │
│  ├─ Rover poses at each event                                  │
│  └─ Timestamps for verification                               │
│                                                                   │
│  Final Reports (from various nodes)                             │
│  ├─ Detected cubes ({color, position in start_frame})         │
│  ├─ Visited landmarks ({name, placard_text})                  │
│  ├─ Collision events ({count, positions, penalty %})          │
│  └─ Navigation summary ({success rate, time})                 │
│                                                                   │
│  ROSBAG Recording (/tmp/arc2026_TIMESTAMP/)                    │
│  └─ Reproducible mission data for judges                       │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## Communication Sequence (Initialization Phase)

```
1. START: ros2 launch mapping_launch.py
   ├─ RealSense camera → RGB/depth images
   ├─ MPU6500 IMU → accel/gyro
   ├─ Madgwick filter → /imu/data_filtered
   ├─ RTAB-Map → /rtabmap/odom (visual odometry)
   ├─ EKF fusion → /odometry/filtered
   ├─ Static TF publishers → base_link, camera_link, etc.
   └─ Ready: "/odometry/filtered published, TF tree complete"

2. WAIT: ~10 seconds (RTAB-Map initialization)
   ├─ RTAB-Map needs to track features
   └─ Grid map building (/rtabmap/grid_map)

3. START: ros2 launch navigation_launch.py
   ├─ map_relay: /rtabmap/grid_map → /map
   ├─ global_costmap: Subscribes to /map, /camera/camera/depth/color/points
   ├─ local_costmap: Rolling 3m×3m window
   ├─ planner_server: Waits for /map and /odometry/filtered
   ├─ controller_server: Waits for /odometry/filtered
   ├─ bt_navigator: Activates action server /navigate_to_pose
   └─ Ready: "Nav2 stack running, action server available"

4. START: ros2 launch nav2_complete.launch.py
   ├─ autonomous_phase_manager:
   │  ├─ State = INITIALIZATION
   │  ├─ Subscribes /odometry/filtered
   │  └─ Monitors /cmd_vel_manual
   │
   ├─ start_frame_reference:
   │  └─ Waits for /initialpose to set robot's initial pose
   │
   ├─ waypoint_handler:
   │  ├─ Loads landmarks.yaml
   │  └─ Waits for user trigger to start navigation
   │
   ├─ frontier_explorer:
   │  ├─ Subscribes /global_costmap/costmap
   │  └─ Ready to detect frontiers
   │
   ├─ collision_recovery:
   │  ├─ Subscribes /odometry/filtered
   │  └─ Monitors for stuck conditions
   │
   ├─ object_reporter:
   │  ├─ TF buffer ready
   │  └─ Waiting for /perception/* detections
   │
   └─ ROSBAG recording: Started

5. USER: Sets initial pose via RViz ("/initialpose")
   ├─ start_frame_reference:
   │  ├─ Captures robot pose in map frame
   │  ├─ Creates map → start_frame transform
   │  └─ Publishes: "Start frame initialized"
   │
   └─ object_reporter: Now ready to transform detections

6. MISSION START: User triggers waypoint_handler
   ├─ autonomous_phase_manager:
   │  ├─ Detects first odometry update
   │  ├─ Transitions: INITIALIZATION → AUTONOMOUS
   │  ├─ Locks manual control
   │  └─ Publishes: /autonomous_phase/events
   │
   └─ waypoint_handler:
      ├─ Sends first landmark to /navigate_to_pose
      └─ Nav2 begins planning and following path

---

## Key Integration Points

### Odometry Topic: /odometry/filtered
- **Source**: EKF (robot_localization) from mapping_launch.py
- **Consumers**:
  - autonomous_phase_manager (detects movement)
  - waypoint_handler (checks arrival at landmarks)
  - frontier_explorer (tracks current position)
  - collision_recovery (stuck detection)
  - Navigation manager (mission progress)

### TF Tree
- **Root**: map (from RTAB-Map)
- **Broadcast by**: mapping_launch.py (EKF node + static transforms)
- **Used by**:
  - start_frame_reference (creates start_frame as child of map)
  - object_reporter (transforms map → start_frame)
  - All Nav2 components (path planning in map frame)
  - Visualization (RViz renders all frames)

### Navigation Action Server: /navigate_to_pose
- **Owner**: nav2_bt_navigator (from navigation_launch.py)
- **Clients**:
  - waypoint_handler (sends landmark goals)
  - frontier_explorer (sends frontier goals)
  - Can also be called manually via ros2 action send_goal

### Depth Camera PointCloud: /camera/camera/depth/color/points
- **Source**: RealSense camera (mapping_launch.py with alignment)
- **Consumers**:
  - global_costmap (obstacle detection)
  - local_costmap (real-time obstacles)
  - Perception module (detection algorithms)

### Detection Topics
- **Input**: /perception/{cube,landmark}_detections (in map frame)
- **Consumer**: object_reporter
- **Output**: /object_reporter/{cube,landmark}_report (in start_frame)

---

## Launch File Dependencies

```
launch_order.txt:
├─ [1] mapping_launch.py
│   ├─ Provides: /odometry/filtered, /map, /tf tree
│   └─ Must start first (RTAB-Map needs time to initialize)
├─ [2] navigation_launch.py
│   ├─ Depends on: /odometry/filtered, /map
│   └─ Provides: /navigate_to_pose action server, costmaps
├─ [3] nav2_complete.launch.py
│   ├─ Depends on: /navigate_to_pose, /odometry/filtered, /tf tree
│   └─ Provides: High-level autonomy and compliance
```

---

## Code Update Summary

✅ **Updated Files**:
- autonomous_phase_manager.py → Uses /odometry/filtered
- start_frame_reference.py → References map, base_link frames
- waypoint_handler.py → Sends goals to /navigate_to_pose
- frontier_exploration.py → Subscribes /global_costmap/costmap
- collision_recovery.py → Removed LIDAR, uses depth camera only
- object_reporter.py → Transforms map → start_frame
- navigation_manager.py → Coordinates with Nav2

✅ **Configuration Files Updated**:
- nav2_params.yaml → Removed AMCL (laser), added EKF odometry
- costmap_params.yaml → RealSense depth camera only
- All topic names → Absolute paths with leading /

✅ **Documentation**:
- Added launch file dependencies to each module
- Frame names documented
- Topic names clarified
- TF tree explained
