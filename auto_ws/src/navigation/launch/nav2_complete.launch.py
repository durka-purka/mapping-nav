"""
COMPLETE AUTONOMOUS NAVIGATION + COMPLIANCE STACK
Australian Rover Challenge 2026

Launches:
  ✓ Nav2 bringup
  ✓ Navigation Manager
  ✓ Waypoint Handler
  ✓ Frontier Explorer
  ✓ Collision Recovery
  ✓ Autonomous Phase Manager (Rule Compliance)
  ✓ Start Frame Reference Manager (Coordinate Compliance)
  ✓ Object Reporter (Detection Reporting)
  ✓ ROSBAG Recording (Full Audit)

This is the competition-grade launch file with full compliance checks.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, ExecuteProcess
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from datetime import datetime


def generate_launch_description():
    """Generate complete autonomous navigation stack with compliance"""
    
    # ═══════════════════════════════════════════════════════════════
    # LAUNCH ARGUMENTS
    # ═══════════════════════════════════════════════════════════════
    
    landmarks_file_arg = DeclareLaunchArgument(
        'landmarks_file',
        default_value='',
        description='YAML file with landmark locations from task schematic'
    )
    
    autonomous_phase_arg = DeclareLaunchArgument(
        'autonomous_phase',
        default_value='true',
        description='Running in autonomous phase'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )
    
    record_rosbag_arg = DeclareLaunchArgument(
        'record_rosbag',
        default_value='true',
        description='Record all topics to rosbag for audit'
    )
    
    bag_output_arg = DeclareLaunchArgument(
        'bag_output',
        default_value=f'/tmp/arc2026_{datetime.now().strftime("%Y%m%d_%H%M%S")}',
        description='Output directory for rosbag recording'
    )
    
    # ═══════════════════════════════════════════════════════════════
    # NAV2 BRINGUP
    # ═══════════════════════════════════════════════════════════════
    
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'bringup_launch.py')
        ),
        launch_arguments={
            'namespace': '',
            'use_namespace': 'false',
            'slam': 'false',
            'map': PathJoinSubstitution([
                get_package_share_directory('navigation'),
                'maps', 'arena.yaml'
            ]),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': PathJoinSubstitution([
                get_package_share_directory('navigation'),
                'config', 'nav2_params.yaml'
            ]),
            'autostart': 'true',
            'use_composition': 'false'
        }.items()
    )
    
    # ═══════════════════════════════════════════════════════════════
    # COMPLIANCE NODES (Critical for Rule Enforcement)
    # ═══════════════════════════════════════════════════════════════
    
    # 1. Autonomous Phase Manager
    autonomous_phase_manager = Node(
        package='navigation',
        executable='autonomous_phase_manager',
        name='autonomous_phase_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'start_in_autonomous_mode': LaunchConfiguration('autonomous_phase'),
            'enable_manual_override_lock': True,
            'movement_threshold': 0.01,
            'log_file_path': '/tmp/arc2026_phase_audit.json',
        }]
    )
    
    # 2. Start Frame Reference Manager
    start_frame_reference = Node(
        package='navigation',
        executable='start_frame_reference',
        name='start_frame_reference',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'reference_frame_id': 'start_frame',
            'robot_initial_frame_id': 'map',
            'auto_initialize': True,
        }]
    )
    
    # 3. Object Reporter
    object_reporter = Node(
        package='navigation',
        executable='object_reporter',
        name='object_reporter',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autonomous_phase': LaunchConfiguration('autonomous_phase'),
            'reference_frame': 'start_frame',
            'detection_frame': 'map',
        }]
    )
    
    # ═══════════════════════════════════════════════════════════════
    # NAVIGATION NODES
    # ═══════════════════════════════════════════════════════════════
    
    map_relay = Node(
        package='topic_tools',
        executable='relay',
        name='map_relay',
        arguments=['/rtabmap/grid_map', '/map'],
        output='screen'
    )
    
    navigation_manager = Node(
        package='navigation',
        executable='navigation_manager',
        name='navigation_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'nav_timeout': 300,
            'max_retries': 3,
            'exploration_enabled': True,
            'autonomous_mode': LaunchConfiguration('autonomous_phase'),
        }]
    )
    
    waypoint_handler = Node(
        package='navigation',
        executable='waypoint_handler',
        name='waypoint_handler',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'landmarks_file': LaunchConfiguration('landmarks_file'),
            'arrival_distance_threshold': 0.3,
            'arrival_angle_threshold': 0.3,
            'imaging_distance': 0.5,
            'timeout_per_landmark': 120,
            'nav_attempt_timeout': 60,
        }]
    )
    
    frontier_explorer = Node(
        package='navigation',
        executable='frontier_explorer',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'costmap_topic': '/global_costmap/costmap',
            'min_frontier_size': 10,
            'exploration_frequency': 1.0,
            'max_exploration_time': 600,
            'cube_detection_enabled': True,
            'visualize_frontiers': True,
        }]
    )
    
    collision_recovery = Node(
        package='navigation',
        executable='collision_recovery',
        name='collision_recovery',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'stuck_timeout': 10.0,
            'min_forward_progress': 0.1,
            'max_recovery_attempts': 4,
            'autonomous_mode': LaunchConfiguration('autonomous_phase'),
        }]
    )
    
    # ═══════════════════════════════════════════════════════════════
    # ROSBAG RECORDING (Full Audit Trail)
    # ═══════════════════════════════════════════════════════════════
    
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('bag_output'),
            # Core navigation
            '/odom',
            '/odometry/filtered',
            '/cmd_vel',
            # SLAM
            '/rtabmap/grid_map',
            '/rtabmap/cloud_map',
            # Nav2
            '/map',
            '/global_costmap/costmap',
            '/local_costmap/costmap',
            # Compliance & Reporting
            '/autonomous_phase/status',
            '/autonomous_phase/events',
            '/object_reporter/cube_report',
            '/object_reporter/landmark_report',
            '/waypoint_handler/status',
            '/frontier_explorer/status',
            '/collision_recovery/status',
            # TF
            '/tf',
            '/tf_static',
        ],
        condition=lambda context: LaunchConfiguration('record_rosbag').perform(context) == 'true',
        output='screen'
    )
    
    # ═══════════════════════════════════════════════════════════════
    # BUILD LAUNCH DESCRIPTION
    # ═══════════════════════════════════════════════════════════════
    
    ld = LaunchDescription()
    
    # Declare arguments
    ld.add_action(landmarks_file_arg)
    ld.add_action(autonomous_phase_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(record_rosbag_arg)
    ld.add_action(bag_output_arg)
    
    # Log startup
    ld.add_action(LogInfo(msg='╔═══════════════════════════════════════════════════════════╗'))
    ld.add_action(LogInfo(msg='║  AUTONOMOUS ROVER CHALLENGE 2026 - NAVIGATION STACK 2.0   ║'))
    ld.add_action(LogInfo(msg='║  Rule Compliance: Full Compliance Mode Enabled             ║'))
    ld.add_action(LogInfo(msg='╚═════════════════════���═════════════════════════════════════╝'))
    ld.add_action(LogInfo(msg=''))
    ld.add_action(LogInfo(msg=['Autonomous Phase: ', LaunchConfiguration('autonomous_phase')]))
    ld.add_action(LogInfo(msg=['Landmarks File: ', LaunchConfiguration('landmarks_file')]))
    ld.add_action(LogInfo(msg=['ROS Bag Recording: ', LaunchConfiguration('record_rosbag')]))
    ld.add_action(LogInfo(msg=''))
    
    # Launch Nav2
    ld.add_action(nav2_bringup)
    ld.add_action(map_relay)
    
    # Launch compliance nodes FIRST (before navigation)
    ld.add_action(autonomous_phase_manager)
    ld.add_action(start_frame_reference)
    ld.add_action(object_reporter)
    
    # Launch navigation nodes
    ld.add_action(navigation_manager)
    ld.add_action(waypoint_handler)
    ld.add_action(frontier_explorer)
    ld.add_action(collision_recovery)
    
    # Recording
    ld.add_action(rosbag_record)
    
    return ld