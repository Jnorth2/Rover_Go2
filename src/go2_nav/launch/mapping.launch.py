from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rtabmap_params = [
        PathJoinSubstitution([
            FindPackageShare('go2_nav'), 'config', 'rtabmap_mapping.yaml',
        ]),
        {'use_sim_time': LaunchConfiguration('use_sim_time')},
    ]
    rtabmap_remappings = [
        ('odom', '/odometry/filtered'),
        # ('rgb/image', '/camera/d455/color/image'),
        # ('rgb/camera_info', '/camera/d455/color/camera_info'),
        ('scan_cloud', '/utlidar/cloud_restamped'),
        ('grid_map', '/map'),
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
            'use_rviz', default_value='false',
            choices=['true', 'false'],
            description='Launch RViz (disabled by default; run groundstation_mapping.launch.py instead)',
        ),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='false',
            choices=['true', 'false'],
            description='Delete the RTAB-Map database on startup to begin a fresh map',
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            choices=['true', 'false'],
            description='Start the RealSense D435 driver (go2_camera camera_capture_launch.py)',
        ),
        DeclareLaunchArgument(
            'use_visual_odom', default_value='true',
            choices=['true', 'false'],
            description='Run D435 visual odometry and fuse it in the EKF',
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
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
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
