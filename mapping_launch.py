"""
ROS 2 Launch File: Mapping & Localization
For RealSense D435 + MPU6500 IMU robot

File: mapping_launch.py
Purpose: SLAM (mapping while moving) using RTAB-Map + sensor fusion

Launch with:
  ros2 launch your_package mapping_launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ═══════════════════════════════════════════════════════════════════
    # SENSOR 1: RealSense D435 Camera
    # ═══════════════════════════════════════════════════════════════════
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'depth_module.depth_profile': '640x480x30',
            'rgb_camera.color_profile': '640x480x30',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'depth_module.exposure.1': '8500',
            'depth_module.gain.1': '16',
            'infra_module.exposure.1': '8500',
            'infra_module.gain.1': '16',
            'enable_sync': 'true',
            'camera_name': 'camera',
            'camera_type': 'D435',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'unite_imu_method': '0',
            'publish_tf': 'false',
        }.items()
    )

    # ═══════════════════════════════════════════════════════════════════
    # SENSOR 2: MPU6500 IMU
    # ═══════════════════════════════════════════════════════════════════
    imu_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros2_mpu6500'),
                'launch',
                'ros2_mpu6500.launch.py'
            ])
        ]),
        launch_arguments={
            'publish_tf': 'false',
        }.items()
    )

    # ═══════════════════════════════════════════════════════════════════
    # PROCESSING 1: Madgwick IMU Filter
    # ═══════════════════════════════════════════════════════════════════
    madgwick_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter_madgwick',
        output='screen',
        parameters=[{
            'use_mag': False,
            'gain': 0.03,
            'zeta': 0.0,
            'world_frame': 'enu',
            'fixed_frame': 'imu_link',
            'publish_tf': False,
            'remove_gravitational_acceleration': True,
            'stateless': False,
        }],
        remappings=[
            ('imu/data_raw', '/imu/data'),
            ('imu/data', '/imu/data_filtered')
        ]
    )

    # ═══════════════════════════════════════════════════════════════════
    # PROCESSING 2: RTAB-Map SLAM
    # ═══════════════════════════════════════════════════════════════════
    rtabmap_args = {
        'rgb_topic': '/camera/camera/color/image_raw',
        'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        'frame_id': 'camera_link',
        'odom_frame_id': 'odom',
        'map_frame_id': 'map',
        'publish_tf': 'false',
        'imu_topic': '/imu/data_filtered',
        'wait_imu_to_init': 'true',
        'always_check_imu_tf': 'false',
        'subscribe_rgbd': 'false',
        'subscribe_rgb': 'true',
        'subscribe_depth': 'true',
        'subscribe_scan_cloud': 'false',
        'Mem/STMSize': '30',
        'Mem/rehearsalSimilarity': '0.7',
        'Grid/FromDepth': 'true',
        'Grid/3D': 'true',
        'Grid/RangeMax': '5.0',
        'Grid/Resolution': '0.05',
        'Grid/RayTracing': 'true',
        'visual_odometry': 'true',
        'approx_sync': 'true',
        'approx_sync_max_interval': '0.1',
        'queue_size': '50',
        'qos': '2',
        'Rtabmap/DetectionRate': '1.0',
        'Mem/IncrementalMemory': 'true',
        'Mem/InitWMWithAllNodes': 'false',
        'wait_for_transform': '0.2',
        'wait_for_transform_duration': '0.2',
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/ProximityBySpace': 'true',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/OptimizeFromGraphEnd': 'false',
        'Odom/Strategy': '0',
        'Odom/MaxDepth': '8.0',
        'Odom/MinDepth': '0.3',
        'Odom/GuessMotion': 'true',
        'OdomF2M/MaxSize': '3000',
        'OdomF2M/ScanSubtractRadius': '0.1',
        'OdomF2M/ScanRange': '5.0',
        'OdomF2M/MaxOptimizationError': '1.0',
        'Vis/MaxFeatures': '1000',
        'Vis/MinInliers': '8',
        'Vis/FeatureType': '6',
        'Vis/CorNNDR': '0.8',
        'RGBD/MaxDepth': '8.0',
        'RGBD/MinDepth': '0.2',
        'cloud_output_voxelized': 'true',
        'cloud_voxel_size': '0.05',
    }

    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rtabmap_launch'),
                'launch',
                'rtabmap.launch.py'
            ])
        ]),
        launch_arguments=rtabmap_args.items()
    )

    # ═══════════════════════════════════════════════════════════════════
    # PROCESSING 3: EKF Sensor Fusion
    # ═══════════════════════════════════════════════════════════════════
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[{
            'frequency': 50.0,
            'two_d_mode': False,
            'publish_tf': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_link_frame': 'base_link',
            'world_frame': 'odom',
            'odom0': '/rtabmap/odom',
            'odom0_config': [True,  True,  True,
                             False, False, False,
                             True,  True,  True,
                             False, False, False,
                             False, False, False],
            'odom0_queue_size': 30,
            'odom0_differential': False,
            'odom0_relative': False,
            'odom0_pose_rejection_threshold': 10.0,
            'odom0_twist_rejection_threshold': 5.0,
            'imu0': '/imu/data_filtered',
            'imu0_config': [False, False, False,
                            True,  True,  True,
                            False, False, False,
                            False, False, True,
                            True,  True,  True],
            'imu0_remove_gravitational_acceleration': True,
            'imu0_differential': False,
            'imu0_queue_size': 100,
            'imu0_relative': False,
            'imu0_pose_rejection_threshold': 2.0,
            'imu0_twist_rejection_threshold': 2.0,
            'process_noise_covariance': [
                0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.05, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.03, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.06, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.025,0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.04, 0.0,  0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.02, 0.0,  0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,  0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.01, 0.0,
                0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.0,  0.015
            ],
        }],
        remappings=[
            ('odometry/filtered', '/odometry/filtered')
        ]
    )

    # ═══════════════════════════════════════════════════════════════════
    # TF TREE: Static Transforms
    # ═══════════════════════════════════════════════════════════════════
    base_to_camera_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link'
        ]
    )

    base_to_imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ]
    )

    camera_to_depth_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_depth_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_depth_frame'
        ]
    )

    camera_to_color_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_to_color_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_frame'
        ]
    )

    depth_to_depth_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='depth_to_depth_optical_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '-0.5',
            '--frame-id', 'camera_depth_frame',
            '--child-frame-id', 'camera_depth_optical_frame'
        ]
    )

    color_to_color_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='color_to_color_optical_broadcaster',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0.5', '--qy', '-0.5', '--qz', '0.5', '--qw', '-0.5',
            '--frame-id', 'camera_color_frame',
            '--child-frame-id', 'camera_color_optical_frame'
        ]
    )

    # ═══════════════════════════════════════════════════════════════════
    # LAUNCH ALL NODES
    # ═══════════════════════════════════════════════════════════════════
    return LaunchDescription([
        realsense_launch,
        imu_launch,
        base_to_camera_tf,
        base_to_imu_tf,
        camera_to_depth_tf,
        camera_to_color_tf,
        depth_to_depth_optical_tf,
        color_to_color_optical_tf,
        madgwick_node,
        rtabmap_launch,
        ekf_node,
    ])
