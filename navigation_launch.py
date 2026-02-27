"""
ROS 2 Launch File: Navigation
For RealSense D435 + MPU6500 IMU robot

File: navigation_launch.py
Purpose: Autonomous navigation using Nav2 with depth camera obstacle avoidance

Prerequisites:
  - mapping_launch.py must be running first
  - RTAB-Map must have built some map

Launch with:
  ros2 launch your_package navigation_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ═══════════════════════════════════════════════════════════════════
    # MAP RELAY: Bridge RTAB-Map → Nav2
    # ═══════════════════════════════════════════════════════════════════
    map_relay = Node(
        package='topic_tools',
        executable='relay',
        name='map_relay',
        arguments=['/rtabmap/grid_map', '/map'],
        output='screen',
    )

    # ═══════════════════════════════════════════════════════════════════
    # CONTROLLER SERVER: Local path execution + obstacle avoidance
    # ═══════════════════════════════════════════════════════════════════
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'controller_frequency': 20.0,
            'min_x_velocity_threshold': 0.001,
            'min_y_velocity_threshold': 0.5,
            'min_theta_velocity_threshold': 0.001,
            'failure_tolerance': 0.3,
            'progress_checker_plugin': 'progress_checker',
            'goal_checker_plugin': 'goal_checker',
            'controller_plugins': ['FollowPath'],
            'odom_topic': '/odometry/filtered',
            
            'progress_checker': {
                'plugin': 'nav2_controller::SimpleProgressChecker',
                'required_movement_radius': 0.5,
                'movement_time_allowance': 10.0,
            },
            
            'goal_checker': {
                'plugin': 'nav2_controller::SimpleGoalChecker',
                'xy_goal_tolerance': 0.10,
                'yaw_goal_tolerance': 0.15,
                'stateful': True,
            },
            
            'FollowPath': {
                'plugin': 'dwb_core::DWBLocalPlanner',
                'debug_trajectory_details': True,
                'min_vel_x': 0.0,
                'min_vel_y': 0.0,
                'max_vel_x': 0.3,
                'max_vel_y': 0.0,
                'max_vel_theta': 1.0,
                'min_speed_xy': 0.0,
                'max_speed_xy': 0.3,
                'min_speed_theta': 0.0,
                'acc_lim_x': 2.0,
                'acc_lim_y': 0.0,
                'acc_lim_theta': 3.0,
                'decel_lim_x': -2.5,
                'decel_lim_y': 0.0,
                'decel_lim_theta': -3.2,
                'vx_samples': 20,
                'vy_samples': 0,
                'vtheta_samples': 20,
                'sim_time': 2.0,
                'linear_granularity': 0.05,
                'angular_granularity': 0.025,
                'transform_tolerance': 0.2,
                'xy_goal_tolerance': 0.10,
                'trans_stopped_velocity': 0.25,
                'short_circuit_trajectory_evaluation': True,
                'critics': ['RotateToGoal', 'Oscillation', 'BaseObstacle', 
                           'GoalAlign', 'PathAlign', 'PathDist', 'GoalDist'],
                'BaseObstacle.scale': 0.05,
                'PathAlign.scale': 32.0,
                'PathAlign.forward_point_distance': 0.1,
                'GoalAlign.scale': 24.0,
                'GoalAlign.forward_point_distance': 0.1,
                'PathDist.scale': 32.0,
                'GoalDist.scale': 24.0,
                'RotateToGoal.scale': 32.0,
                'RotateToGoal.slowing_factor': 5.0,
                'RotateToGoal.lookahead_time': -1.0,
            },
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ],
    )

    # ═══════════════════════════════════════════════════════════════════
    # PLANNER SERVER: Global path planning (A*)
    # ═══════════════════════════════════════════════════════════════════
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'planner_plugins': ['GridBased'],
            'GridBased': {
                'plugin': 'nav2_navfn_planner::NavfnPlanner',
                'tolerance': 0.5,
                'use_astar': True,
                'allow_unknown': True,
            },
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    # BEHAVIOR SERVER: Recovery behaviors when stuck
    # ═══════════════════════════════════════════════════════════════════
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'local_costmap_topic': 'local_costmap/costmap_raw',
            'global_costmap_topic': 'global_costmap/costmap_raw',
            'local_footprint_topic': 'local_costmap/published_footprint',
            'global_footprint_topic': 'global_costmap/published_footprint',
            'cycle_frequency': 10.0,
            'behavior_plugins': ['spin', 'backup', 'drive_on_heading', 'wait'],
            'spin': {
                'plugin': 'nav2_behaviors::Spin',
            },
            'backup': {
                'plugin': 'nav2_behaviors::BackUp',
            },
            'drive_on_heading': {
                'plugin': 'nav2_behaviors::DriveOnHeading',
            },
            'wait': {
                'plugin': 'nav2_behaviors::Wait',
            },
            'local_frame': 'odom',
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'transform_tolerance': 0.1,
            'simulate_ahead_time': 2.0,
            'max_rotational_vel': 1.0,
            'min_rotational_vel': 0.4,
            'rotational_acc_lim': 3.2,
        }],
        remappings=[
            ('cmd_vel', '/cmd_vel'),
        ],
    )

    # ═══════════════════════════════════════════════════════════════════
    # BT NAVIGATOR: Orchestrates everything
    # ═══════════════════════════════════════════════════════════════════
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'global_frame': 'map',
            'robot_base_frame': 'base_link',
            'odom_topic': '/odometry/filtered',
            'bt_loop_duration': 10,
            'default_server_timeout': 20,
            'action_server_result_timeout': 900.0,
            'navigators': ['navigate_to_pose', 'navigate_through_poses'],
            'navigate_to_pose': {
                'plugin': 'nav2_bt_navigator::NavigateToPoseNavigator'
            },
            'navigate_through_poses': {
                'plugin': 'nav2_bt_navigator::NavigateThroughPosesNavigator'
            },
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    # LOCAL COSTMAP: 3m×3m rolling window with real-time obstacles
    # ═══════════════════════════════════════════════════════════════════
    local_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='local_costmap',
        output='screen',
        parameters=[{
            'local_costmap': {
                'ros__parameters': {
                    'use_sim_time': False,
                    'update_frequency': 5.0,
                    'publish_frequency': 2.0,
                    'global_frame': 'odom',
                    'robot_base_frame': 'base_link',
                    'rolling_window': True,
                    'width': 3,
                    'height': 3,
                    'resolution': 0.05,
                    'robot_radius': 0.22,
                    'plugins': ['voxel_layer', 'inflation_layer'],
                    'voxel_layer': {
                        'plugin': 'nav2_costmap_2d::VoxelLayer',
                        'enabled': True,
                        'publish_voxel_map': True,
                        'origin_z': 0.0,
                        'z_resolution': 0.05,
                        'z_voxels': 16,
                        'max_obstacle_height': 2.0,
                        'min_obstacle_height': 0.1,
                        'mark_threshold': 0,
                        'observation_sources': 'scan',
                        'scan': {
                            'topic': '/camera/camera/depth/color/points',
                            'sensor_frame': 'camera_depth_optical_frame',
                            'data_type': 'PointCloud2',
                            'min_obstacle_height': 0.1,
                            'max_obstacle_height': 2.0,
                            'obstacle_max_range': 2.5,
                            'obstacle_min_range': 0.3,
                            'raytrace_max_range': 3.0,
                            'raytrace_min_range': 0.0,
                            'clearing': True,
                            'marking': True,
                        },
                    },
                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',
                        'cost_scaling_factor': 3.0,
                        'inflation_radius': 0.55,
                    },
                    'always_send_full_costmap': True,
                }
            }
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    # GLOBAL COSTMAP: Full map with obstacles
    # ═══════════════════════════════════════════════════════════════════
    global_costmap = Node(
        package='nav2_costmap_2d',
        executable='nav2_costmap_2d',
        name='global_costmap',
        output='screen',
        parameters=[{
            'global_costmap': {
                'ros__parameters': {
                    'use_sim_time': False,
                    'update_frequency': 1.0,
                    'publish_frequency': 1.0,
                    'global_frame': 'map',
                    'robot_base_frame': 'base_link',
                    'robot_radius': 0.22,
                    'resolution': 0.05,
                    'track_unknown_space': True,
                    'plugins': ['static_layer', 'obstacle_layer', 'inflation_layer'],
                    'static_layer': {
                        'plugin': 'nav2_costmap_2d::StaticLayer',
                        'map_subscribe_transient_local': True,
                    },
                    'obstacle_layer': {
                        'plugin': 'nav2_costmap_2d::ObstacleLayer',
                        'enabled': True,
                        'observation_sources': 'scan',
                        'scan': {
                            'topic': '/camera/camera/depth/color/points',
                            'sensor_frame': 'camera_depth_optical_frame',
                            'data_type': 'PointCloud2',
                            'min_obstacle_height': 0.1,
                            'max_obstacle_height': 2.0,
                            'obstacle_max_range': 2.5,
                            'raytrace_max_range': 3.0,
                            'clearing': True,
                            'marking': True,
                        },
                    },
                    'inflation_layer': {
                        'plugin': 'nav2_costmap_2d::InflationLayer',
                        'cost_scaling_factor': 3.0,
                        'inflation_radius': 0.55,
                    },
                    'always_send_full_costmap': True,
                }
            }
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    # LIFECYCLE MANAGER: Brings all Nav2 nodes up in order
    # ═══════════════════════════════════════════════════════════════════
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ],
            'bond_timeout': 4.0,
        }],
    )

    # ═══════════════════════════════════════════════════════════════════
    # LAUNCH ALL NODES
    # ═══════════════════════════════════════════════════════════════════
    return LaunchDescription([
        map_relay,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        local_costmap,
        global_costmap,
        lifecycle_manager,
    ])
