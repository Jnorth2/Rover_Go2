from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_dir = PathJoinSubstitution([FindPackageShare('go2_nav'), 'config'])
    rtabmap_params = [
        PathJoinSubstitution([config_dir, 'rtabmap_mapping.yaml']),
        # Camera overrides on top of the lidar-only base. With use_camera:=false
        # the base file is loaded a second time instead, which changes nothing.
        PathJoinSubstitution([config_dir, PythonExpression([
            "'rtabmap_camera.yaml' if '", LaunchConfiguration('use_camera'),
            "' == 'true' else 'rtabmap_mapping.yaml'",
        ])]),
        # GPS overrides (rtabmap_gps.yaml) the same way, only with with_gps:=true
        PathJoinSubstitution([config_dir, PythonExpression([
            "'rtabmap_gps.yaml' if '", LaunchConfiguration('with_gps'),
            "' == 'true' else 'rtabmap_mapping.yaml'",
        ])]),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    rtabmap_remappings = [
        ('odom', '/odometry/filtered'),
        ('rgbd_image', '/camera/d435/rgbd_image'),
        ('scan_cloud', '/utlidar/cloud_restamped'),
        ('grid_map', '/map'),
        # Only RTK-fixed solutions (gps_fix_filter_node); silent with with_gps:=false
        ('gps/fix', '/gps/fix_rtk'),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            choices=['true', 'false'],
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'scan_cloud_topic', default_value='/utlidar/cloud',
            description='LiDAR point cloud topic. Prefer /utlidar/cloud (sensor frame). '
                        '/utlidar/cloud_deskewed is in Go2 odom frame and breaks height filtering.',
        ),
        DeclareLaunchArgument(
            'lidar_min_range', default_value='0.5',
            description='Discard lidar returns closer than this to the sensor (m); 0 disables',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            choices=['true', 'false'],
            description='Launch RViz (disabled by default; run groundstation_mapping.launch.py instead)',
        ),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='true',
            choices=['true', 'false'],
            description='Delete the RTAB-Map database on startup to begin a fresh map '
                        '(default). false = resume mapping into ~/.ros/rtabmap.db',
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            choices=['true', 'false'],
            description='Start the RealSense D435 driver and add the depth camera to '
                        'RTAB-Map (rtabmap_camera.yaml); false = lidar-only mapping',
        ),
        DeclareLaunchArgument(
            'with_gps', default_value='false',
            choices=['true', 'false'],
            description='Start the RTK GPS rover (go2_odom gps_rover.launch.py) and use '
                        'RTK-fixed solutions as RTAB-Map position priors (rtabmap_gps.yaml)',
        ),
        DeclareLaunchArgument(
            'use_visual_odom', default_value=LaunchConfiguration('use_camera'),
            choices=['true', 'false'],
            description='Run D435 visual odometry and fuse it in the EKF (defaults to use_camera)',
        ),

        # Re-stamp the lidar cloud from Go2 clock to Jetson clock.
        # The Go2's system clock is offset ~113 s from the Jetson's. Without this,
        # RTAB-Map's approx_sync cannot match the cloud against EKF odometry output.
        Node(
            package='go2_nav',
            executable='cloud_restamp_node.py',
            name='cloud_restamp_node',
            output='screen',
            remappings=[
                ('cloud_in',  LaunchConfiguration('scan_cloud_topic')),
                ('cloud_out', '/utlidar/cloud_restamped'),
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # Drop returns this close to the lidar (spurious points ahead of the head)
                'min_range': ParameterValue(
                    LaunchConfiguration('lidar_min_range'), value_type=float),
            }],
        ),

        # RealSense D435 driver (frames d435i_*, topics /camera/d435/...) plus the
        # color video stream to the groundstation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('go2_camera'), 'launch', 'camera_capture_launch.py',
                ]),
            ]),
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),

        # Bundle color + aligned depth + camera info into one rtabmap_msgs/RGBDImage
        # for RTAB-Map. The driver's own /camera/d435/rgbd is realsense2_camera_msgs/RGBD,
        # which RTAB-Map can't read.
        Node(
            package='rtabmap_sync',
            executable='rgbd_sync',
            name='rgbd_sync',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'approx_sync': True,
            }],
            remappings=[
                ('rgb/image',       '/camera/d435/color/image_raw'),
                ('depth/image',     '/camera/d435/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/d435/color/camera_info'),
                ('rgbd_image',      '/camera/d435/rgbd_image'),
            ],
            condition=IfCondition(LaunchConfiguration('use_camera')),
        ),

        # Odometry: EKF fusing /utlidar/robot_odom + UTlidar IMU + D435 visual odometry
        # → /odometry/filtered + odom→base_link TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('go2_odom'), 'launch', 'odom.launch.py',
                ]),
            ]),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'use_visual_odom': LaunchConfiguration('use_visual_odom'),
            }.items(),
        ),

        # RTK GPS rover: u-blox driver (RTCM corrections from /rtcm) → /ublox_gps_node/fix
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('go2_odom'), 'launch', 'gps_rover.launch.py',
                ]),
            ]),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
            condition=IfCondition(LaunchConfiguration('with_gps')),
        ),

        # Pass only RTK-fixed fixes on to RTAB-Map
        Node(
            package='go2_odom',
            executable='gps_fix_filter_node.py',
            name='gps_fix_filter_node',
            output='screen',
            remappings=[
                ('fix_in',  '/ublox_gps_node/fix'),
                ('fix_out', '/gps/fix_rtk'),
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'require_rtk_fixed': True,
                'max_horizontal_std': 0.10,
            }],
            condition=IfCondition(LaunchConfiguration('with_gps')),
        ),

        # Robot description: publishes /robot_description and static TF chain
        # (base_link → trunk → utlidar_lidar, imu_link, front_camera, etc.)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('go2_description'), 'launch', 'load_go2.launch.py',
                ]),
            ]),
            launch_arguments={'use_rviz': 'false'}.items(),
        ),

        # RTAB-Map SLAM (fresh map — deletes existing database on startup)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['--delete_db_on_start'],
            parameters=rtabmap_params,
            remappings=rtabmap_remappings,
            # Saving a large map to the database on Ctrl+C can take well over
            # launch's default 5 s before SIGTERM/SIGKILL, corrupting the .db.
            sigterm_timeout='120',
            sigkill_timeout='120',
            condition=IfCondition(LaunchConfiguration('delete_db_on_start')),
        ),

        # RTAB-Map SLAM (resume existing map from database)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=rtabmap_params,
            remappings=rtabmap_remappings,
            # Saving a large map to the database on Ctrl+C can take well over
            # launch's default 5 s before SIGTERM/SIGKILL, corrupting the .db.
            sigterm_timeout='120',
            sigkill_timeout='120',
            condition=UnlessCondition(LaunchConfiguration('delete_db_on_start')),
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('go2_nav'), 'config', 'mapping.rviz',
            ])],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
