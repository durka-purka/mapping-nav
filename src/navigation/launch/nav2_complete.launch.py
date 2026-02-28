

# """
# Complete Nav2 + Custom Navigation Stack
# Australian Rover Challenge 2026

# Launches:
# - Nav2 bringup (planner, controller, recovery)
# - Navigation Manager (high-level coordination)
# - Waypoint Handler (landmark navigation)
# - Frontier Explorer (exploratory mapping)
# - Collision Recovery Handler (failure detection & recovery)

# Usage:
#   ros2 launch navigation nav2_complete_autonomous.launch.py \
#     landmarks_file:=/path/to/landmarks.yaml \
#     autonomous_phase:=true
# """

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory
# import os


# def generate_launch_description():
#     """Generate launch description for complete navigation stack"""
    
#     # ═══════════════════════════════════════════════════════════════════
#     # LAUNCH ARGUMENTS
#     # ═══════════════════════════════════════════════════════════════════
    
#     landmarks_file_arg = DeclareLaunchArgument(
#         'landmarks_file',
#         default_value='',
#         description='Path to landmarks YAML file from task schematic'
#     )
    
#     autonomous_phase_arg = DeclareLaunchArgument(
#         'autonomous_phase',
#         default_value='true',
#         description='Running in autonomous phase (penalties apply)'
#     )
    
#     use_sim_time_arg = DeclareLaunchArgument(
#         'use_sim_time',
#         default_value='false',
#         description='Use simulation time if true'
#     )
    
#     map_yaml_arg = DeclareLaunchArgument(
#         'map',
#         default_value=PathJoinSubstitution([
#             get_package_share_directory('navigation'),
#             'maps', 'arena.yaml'
#         ]),
#         description='Full path to map YAML file'
#     )
    
#     nav2_params_arg = DeclareLaunchArgument(
#         'params_file',
#         default_value=PathJoinSubstitution([
#             get_package_share_directory('navigation'),
#             'config', 'nav2_params.yaml'
#         ]),
#         description='Nav2 parameters YAML file'
#     )
    
#     # ═══════════════════════════════════════════════════════════════════
#     # NAV2 BRINGUP
#     # ═══════════════════════════════════════════════════════════════════
    
#     nav2_launch_dir = os.path.join(
#         get_package_share_directory('nav2_bringup'),
#         'launch'
#     )
    
#     nav2_bringup = IncludeLaunchDescription(
#         PythonLaunchDescriptionSource(
#             os.path.join(nav2_launch_dir, 'bringup_launch.py')
#         ),
#         launch_arguments={
#             'namespace': '',
#             'use_namespace': 'false',
#             'slam': 'false',  # SLAM handled by RTAB-Map separately
#             'map': LaunchConfiguration('map'),
#             'use_sim_time': LaunchConfiguration('use_sim_time'),
#             'params_file': LaunchConfiguration('params_file'),
#             'autostart': 'true',
#             'use_composition': 'false'
#         }.items()
#     )
    
#     # ═══════════════════════════════════════════════════════════════════
#     # MAP RELAY: RTAB-Map → Nav2
#     # ═══════════════════════════════════════════════════════════════════
    
#     map_relay = Node(
#         package='topic_tools',
#         executable='relay',
#         name='map_relay',
#         arguments=['/rtabmap/grid_map', '/map'],
#         output='screen'
#     )
    
#     # ═══════════════════════════════════════════════════════════════════
#     # CUSTOM NAVIGATION NODES
#     # ═══════════════════════════════════════════════════════════════════
    
#     # Navigation Manager - High-level orchestration
#     navigation_manager = Node(
#         package='navigation',
#         executable='navigation_manager',
#         name='navigation_manager',
#         output='screen',
#         parameters=[{
#             'use_sim_time': LaunchConfiguration('use_sim_time'),
#             'nav_timeout': 300,
#             'max_retries': 3,
#             'exploration_enabled': True,
#             'autonomous_mode': LaunchConfiguration('autonomous_phase'),
#         }]
#     )
    
#     # Waypoint Handler - Activity 2: Landmark navigation
#     waypoint_handler = Node(
#         package='navigation',
#         executable='waypoint_handler',
#         name='waypoint_handler',
#         output='screen',
#         parameters=[{
#             'use_sim_time': LaunchConfiguration('use_sim_time'),
#             'landmarks_file': LaunchConfiguration('landmarks_file'),
#             'arrival_distance_threshold': 0.3,
#             'arrival_angle_threshold': 0.3,
#             'timeout_per_landmark': 120,
#             'nav_attempt_timeout': 60,
#         }]
#     )
    
#     # Frontier Explorer - Activity 3: Exploratory mapping
#     frontier_explorer = Node(
#         package='navigation',
#         executable='frontier_explorer',
#         name='frontier_explorer',
#         output='screen',
#         parameters=[{
#             'use_sim_time': LaunchConfiguration('use_sim_time'),
#             'costmap_topic': '/global_costmap/costmap',
#             'min_frontier_size': 10,
#             'exploration_frequency': 1.0,
#             'max_exploration_time': 600,
#             'cube_detection_enabled': True,
#             'visualize_frontiers': True,
#         }]
#     )
    
#     # Collision Recovery Handler - Autonomous recovery behaviors
#     collision_recovery = Node(
#         package='navigation',
#     )

"""
COMPLETE NAVIGATION STACK LAUNCHER
Australian Rover Challenge 2026

Launches:
  ✓ Nav2 bringup (planner, controller, recovery server)
  ✓ Navigation Manager (high-level orchestration)
  ✓ Waypoint Handler (Activity 2: landmark navigation)
  ✓ Frontier Explorer (Activity 3: exploratory mapping)
  ✓ Collision Recovery Handler (autonomous failure detection & recovery)
  ✓ Map relay (RTAB-Map → Nav2)

Usage:
  ros2 launch navigation nav2_complete_autonomous.launch.py \
    landmarks_file:=/path/to/landmarks.yaml \
    autonomous_phase:=true \
    map:=/path/to/arena.yaml
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate complete navigation launch description"""
    
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
        description='Running in autonomous phase (10% penalty per collision)'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock (for Gazebo)'
    )
    
    map_yaml_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            get_package_share_directory('navigation'),
            'maps', 'arena.yaml'
        ]),
        description='YAML file for initial map (RTAB-Map output)'
    )
    
    nav2_params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('navigation'),
            'config', 'nav2_params.yaml'
        ]),
        description='Nav2 configuration file'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            get_package_share_directory('navigation'),
            'rviz', 'navigation.rviz'
        ]),
        description='RViz configuration for visualization'
    )
    
    # ═══════════════════════════════════════════════════════════════
    # NAV2 BRINGUP (complete stack)
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
            'slam': 'false',  # SLAM handled by mapping package (RTAB-Map)
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': 'true',
            'use_composition': 'false'
        }.items()
    )
    
    # ═══════════════════════════════════════════════════════════════
    # MAP RELAY: RTAB-Map output → Nav2 input
    # ═══════════════════════════════════════════════════════════════
    
    map_relay = Node(
        package='topic_tools',
        executable='relay',
        name='map_relay',
        arguments=['/rtabmap/grid_map', '/map'],
        output='screen'
    )
    
    # ═══════════════════════════════════════════════════════════════
    # CUSTOM NAVIGATION NODES
    # ═══════════════════════════════════════════════════════════════
    
    # 1. NAVIGATION MANAGER
    # High-level orchestration and mode management
    navigation_manager = Node(
        package='navigation',
        executable='navigation_manager',
        name='navigation_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'nav_timeout': 300,  # 5 minutes per landmark
            'max_retries': 3,
            'exploration_enabled': True,
            'autonomous_mode': LaunchConfiguration('autonomous_phase'),
        }]
    )
    
    # 2. WAYPOINT HANDLER
    # Activity 2: Autonomous landmark navigation
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
    
    # 3. FRONTIER EXPLORER
    # Activity 3: Exploratory mapping & cube detection
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
    
    # 4. COLLISION RECOVERY HANDLER
    # Autonomous stuck detection & recovery (Activity 1)
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
            'enable_lidar_checking': False,
            'obstacle_threshold_distance': 0.3,
            'autonomous_mode': LaunchConfiguration('autonomous_phase'),
        }]
    )
    
    # ═══════════════════════════════════════════════════════════════
    # RVIZ VISUALIZATION (optional)
    # ═══════════════════════════════════════════════════════════════
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')]
    )
    
    # ═══════════════════════════════════════════════════════════════
    # BUILD LAUNCH DESCRIPTION
    # ═══════════════════════════════════════════════════════════════
    
    ld = LaunchDescription()
    
    # Declare all arguments
    ld.add_action(landmarks_file_arg)
    ld.add_action(autonomous_phase_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(map_yaml_arg)
    ld.add_action(nav2_params_arg)
    ld.add_action(rviz_config_arg)
    
    # Log startup info
    ld.add_action(LogInfo(msg='Starting Navigation Stack for ARC 2026'))
    ld.add_action(LogInfo(msg=['Landmarks file: ', LaunchConfiguration('landmarks_file')]))
    ld.add_action(LogInfo(msg=['Autonomous phase: ', LaunchConfiguration('autonomous_phase')]))
    
    # Launch Nav2
    ld.add_action(nav2_bringup)
    ld.add_action(map_relay)
    
    # Launch custom navigation nodes
    ld.add_action(navigation_manager)
    ld.add_action(waypoint_handler)
    ld.add_action(frontier_explorer)
    ld.add_action(collision_recovery)
    
    # Launch visualization
    ld.add_action(rviz)
    
    return ld