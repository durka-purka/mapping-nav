import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg         = get_package_share_directory('rover_assembly_urdf')
    nav_pkg     = get_package_share_directory('navigation')
    urdf_file   = os.path.join(pkg, 'urdf', 'Rover Assembly URDF.urdf')
    
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # ── 1. Gazebo Garden with default world ──
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim'],
        output='screen'
    )

    # ── 2. Robot State Publisher ──
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # ── 3. Spawn rover (wait 5s for Gz to load) ──
    spawn = TimerAction(period=5.0, actions=[
        Node(
            package='ros_gz_sim', executable='create',
            arguments=['-topic', 'robot_description', '-entity', 'rover',
                       '-x', '0', '-y', '0', '-z', '0.4'],
            output='screen'
        )
    ])

    # ── Bridges ──
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock]'],
        output='screen'
    )

    camera_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image]'],
        output='screen'
    )

    camera_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/color/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo]'],
        output='screen'
    )

    depth_image_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/camera/aligned_depth_to_color/image_raw@sensor_msgs/msg/Image[gz.msgs.Image]'],
        output='screen'
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/imu/data_raw@sensor_msgs/msg/Imu[gz.msgs.IMU]'],
        output='screen'
    )

    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry]'],
        output='screen'
    )

    # ── RTAB-Map (commented out for now) ──
    # rtabmap = TimerAction(period=8.0, actions=[
    #     Node(
    #         package='rtabmap_ros', executable='rtabmap',
    #         name='rtabmap',
    #         output='screen',
    #         parameters=[{
    #             'use_sim_time': True,
    #             'subscribe_depth': True,
    #             'subscribe_rgb': True,
    #             'frame_id': 'base_link',
    #             'odom_frame_id': 'odom',
    #             'approx_sync': True,
    #             'Mem/IncrementalMemory': 'true',
    #             'Mem/InitWMWithAllNodes': 'false',
    #             'RGBD/NeighborLinkRefining': 'true',
    #             'Reg/Strategy': '1',
    #             'Vis/MinInliers': '10',
    #         }],
    #         remappings=[
    #             ('rgb/image',        '/camera/color/image_raw'),
    #             ('depth/image',      '/camera/aligned_depth_to_color/image_raw'),
    #             ('rgb/camera_info',  '/camera/color/camera_info'),
    #             ('odom',             '/odom'),
    #         ]
    #     )
    # ])

    return LaunchDescription([
        gz_sim, rsp, spawn,
        clock_bridge, camera_image_bridge, camera_info_bridge, depth_image_bridge, imu_bridge, odom_bridge
        # rtabmap
    ])