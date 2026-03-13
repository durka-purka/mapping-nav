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

    # with open(urdf_file, 'rb') as f:
    #     robot_desc = f.read().decode('utf-8')
        
    # robot_desc=robot_desc.replace('<?xml version="1.0" encoding="utf-8"?>', '<?xml version="1.0"?>')

    # ── 1. Gazebo with a world that has features for RTAB-Map to track ──
    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
            '/usr/share/gazebo-11/worlds/cafe.world'   # swap world here
        ],
        output='screen',
        additional_env={'LIBGL_ALWAYS_SOFTWARE': '1'}  # VM fix
    )

    # ── 2. Robot State Publisher ──
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # ── 3. Spawn rover (wait 5s for Gazebo to load the world) ──
    spawn = TimerAction(period=5.0, actions=[
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=['-entity', 'rover', '-topic', 'robot_description',
                       '-x', '0', '-y', '0', '-z', '0.4'],
            output='screen'
        )
    ])

    # ── 4. RTAB-Map (wait 8s, starts mapping from simulated depth camera) ──
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
    #             'Reg/Strategy': '1',       # ICP
    #             'Vis/MinInliers': '10',    # lower = easier to track in sim
    #         }],
    #         remappings=[
    #             ('rgb/image',        '/camera/color/image_raw'),
    #             ('depth/image',      '/camera/aligned_depth_to_color/image_raw'),
    #             ('rgb/camera_info',  '/camera/color/camera_info'),
    #             ('odom',             '/rtabmap/odom'),
    #         ]
    #     )
    # ])

    # ── 5. EKF (fuses rtabmap odom + simulated IMU) ──
    # ekf = TimerAction(period=8.0, actions=[
    #     Node(
    #         package='robot_localization', executable='ekf_node',
    #         name='ekf_filter_node',
    #         parameters=[os.path.join(nav_pkg, 'config', 'ekf_sim.yaml')],
    #         output='screen'
    #     )
    # ])

    # ── 6. Nav2 (wait 15s — needs map to exist first) ──
    # nav2 = TimerAction(period=15.0, actions=[
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             PathJoinSubstitution([
    #                 FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
    #             ])
    #         ]),
    #         launch_arguments={
    #             'use_sim_time': 'true',
    #             'params_file': os.path.join(nav_pkg, 'config', 'nav2_params.yaml'),
    #         }.items()
    #     )
    # ])

    # ── 7. RViz ──
    # rviz = TimerAction(period=10.0, actions=[
    #     Node(
    #         package='rviz2', executable='rviz2',
    #         arguments=['-d', os.path.join(nav_pkg, 'rviz', 'navigation.rviz')],
    #         parameters=[{'use_sim_time': True}]
    #     )
    # ])

    return LaunchDescription([
        gazebo, rsp, spawn,
        # rtabmap, ekf,
        # nav2, rviz
    ])