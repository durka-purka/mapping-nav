"""
Navigation Testing in Gazebo (Without LiDAR)

This launch file:
1. Starts Gazebo with rover model
2. Spawns dummy sensor publishers (camera, IMU)
3. Runs Nav2 stack (without LiDAR, using depth camera only)
4. Launches custom navigation nodes
5. Opens RViz visualization

Compatible with:
- RealSense D435 (simulated as dummy depth publisher)
- MPU6500 IMU (simulated as dummy IMU publisher)
- NO LiDAR required
- RTAB-Map SLAM (optional, can be added later)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for Gazebo-based navigation testing"""
    
    # Package directories
    nav_dir = get_package_share_directory('navigation')
    rover_dir = get_package_share_directory('Rover Assembly URDF')
    
    # =========================================================================
    # LAUNCH ARGUMENTS
    # =========================================================================
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo simulation time'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for visualization'
    )
    
    landmarks_file_arg = DeclareLaunchArgument(
        'landmarks_file',
        default_value=PathJoinSubstitution([nav_dir, 'example_input', 'landmarks.yaml']),
        description='Path to landmarks YAML file'
    )
    
    # =========================================================================
    # GAZEBO SETUP
    # =========================================================================
    
    # Start Gazebo server and client
    gazebo_server = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so'
        ],
        output='screen',
        name='gazebo_server'
    )
    
    # Spawn rover in Gazebo
    spawn_rover = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'rover',
            '-file', os.path.join(rover_dir, 'urdf', 'Rover Assembly URDF.urdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.5',
        ],
        output='screen'
    )
    
    # =========================================================================
    # ROBOT STATE PUBLISHING
    # =========================================================================
    
    # Read URDF
    urdf_file = os.path.join(rover_dir, 'urdf', 'Rover Assembly URDF.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description,
        }]
    )
    
    # Joint State Publisher (from Gazebo)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # =========================================================================
    # DUMMY SENSOR PUBLISHERS (Simulating RealSense D435 + IMU)
    # =========================================================================
    
    # Dummy Odometry (from Gazebo ground truth)
    dummy_odom = Node(
        package='navigation',
        executable='dummy_odom_publisher.py',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Dummy IMU (simulating MPU6500)
    dummy_imu = Node(
        package='navigation',
        executable='dummy_imu_publisher.py',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # Dummy Depth Camera (simulating RealSense D435)
    dummy_depth = Node(
        package='navigation',
        executable='dummy_depth_publisher.py',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )
    
    # =========================================================================
    # EKF SENSOR FUSION (with simulated sensors)
    # =========================================================================
    
    # Create EKF config for Gazebo
    ekf_config_file = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'config', 'ekf_sim.yaml'
    ])
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_file],
        remappings=[
            ('odometry/filtered', 'odometry/filtered'),
        ]
    )
    
    # =========================================================================
    # NAV2 STACK (configured for depth camera, NO LiDAR)
    # =========================================================================
    
    # Nav2 parameters with depth-only costmap
    nav2_config = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'config', 'nav2_params_gazebo.yaml'
    ])
    
    # Nav2 Bringup (without map server - using runtime SLAM instead)
    nav2_nodes = [
        # Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/odom', 'odometry/filtered'),
            ]
        ),
        
        # Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_config],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/odom', 'odometry/filtered'),
            ]
        ),
        
        # Behavior Tree Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config],
        ),
        
        # Recovery Server
        Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_config],
        ),
        
        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '-p', 'autostart:=true'],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'recoveries_server'
                ]
            }]
        ),
    ]
    
    # =========================================================================
    # CUSTOM NAVIGATION NODES
    # =========================================================================
    
    navigation_manager = Node(
        package='navigation',
        executable='navigation_manager.py',
        name='navigation_manager',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'nav_timeout': 300,
            'max_retries': 3,
            'exploration_enabled': True,
        }]
    )
    
    waypoint_handler = Node(
        package='navigation',
        executable='waypoint_handler.py',
        name='waypoint_handler',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'landmarks_file': LaunchConfiguration('landmarks_file'),
            'arrival_distance_threshold': 0.3,
            'arrival_angle_threshold': 0.3,
        }]
    )
    
    frontier_explorer = Node(
        package='navigation',
        executable='frontier_explorer.py',
        name='frontier_explorer',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'costmap_topic': '/global_costmap/costmap',
            'min_frontier_size': 10,
            'exploration_frequency': 1.0,
        }]
    )
    
    collision_recovery = Node(
        package='navigation',
        executable='collision_recovery.py',
        name='collision_recovery',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'stuck_timeout': 10.0,
            'max_recovery_attempts': 4,
        }]
    )
    
    # =========================================================================
    # RVIZ VISUALIZATION
    # =========================================================================
    
    rviz_config = PathJoinSubstitution([
        FindPackageShare('navigation'),
        'rviz', 'navigation.rviz'
    ])
    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=lambda context: LaunchConfiguration('use_rviz').perform(context) == 'true'
    )
    
    # =========================================================================
    # BUILD LAUNCH DESCRIPTION
    # =========================================================================
    
    ld = LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        landmarks_file_arg,
        
        LogInfo(msg='Starting Gazebo Navigation Testing...'),
        LogInfo(msg='Configuration: RealSense D435 (simulated) + MPU6500 IMU (simulated)'),
        LogInfo(msg='NO LiDAR - Depth camera only costmap'),
        
        # Gazebo
        gazebo_server,
        spawn_rover,
        
        # State publishing
        robot_state_publisher,
        joint_state_publisher,
        
        # Sensors (dummy for testing)
        dummy_odom,
        dummy_imu,
        dummy_depth,
        
        # Sensor fusion
        ekf_node,
        
        # Nav2
        *nav2_nodes,
        
        # Custom nodes
        navigation_manager,
        waypoint_handler,
        frontier_explorer,
        collision_recovery,
        
        # Visualization
        rviz,
    ])
    
    return ld