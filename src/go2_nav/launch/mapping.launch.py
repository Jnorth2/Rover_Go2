from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            choices=['true', 'false'],
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'scan_cloud_topic', default_value='/utlidar/cloud_deskewed',
            description='LiDAR point cloud topic. Use /utlidar/cloud if deskewed is unavailable.',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='false',
            choices=['true', 'false'],
            description='Launch RViz (disabled by default; run groundstation_mapping.launch.py instead)',
        ),

        # Odometry: EKF fusing /utlidar/robot_odom + UTlidar IMU → /odom + odom→base_link TF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('go2_odom'), 'launch', 'odom.launch.py',
                ]),
            ]),
            launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time')}.items(),
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

        # RTAB-Map SLAM: builds 2D occupancy grid (/map) and 3D point-cloud map.
        # Publishes map → odom TF correction.
        # Input: /utlidar/cloud_deskewed (point cloud), /odom (EKF output)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('go2_nav'), 'config', 'rtabmap_mapping.yaml',
                ]),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('scan_cloud', LaunchConfiguration('scan_cloud_topic')),
                ('odom',       '/odom'),
                ('grid_map',   '/map'),
            ],
        ),

        # Optional: RealSense D435 visual odometry via RTAB-Map.
        # Uncomment once camera TF (camera_link → base_link) is added to the URDF.
        # The D435 IMU is non-functional on Jetson Orin Nano (no HID kernel module).
        #
        # Node(
        #     package='rtabmap_odom',
        #     executable='rgbd_odometry',
        #     name='rgbd_odometry',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time'),
        #                  'frame_id': 'base_link',
        #                  'approx_sync': True}],
        #     remappings=[
        #         ('rgb/image',       '/camera/d435/color/image_raw'),
        #         ('depth/image',     '/camera/d435/aligned_depth_to_color/image_raw'),
        #         ('rgb/camera_info', '/camera/d435/color/camera_info'),
        #         ('odom',            '/camera/d435/visual_odom'),
        #     ],
        # ),

        # RViz2
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
