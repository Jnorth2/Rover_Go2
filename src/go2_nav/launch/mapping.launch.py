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
    rtabmap_remaps = [
        ('scan_cloud', '/utlidar/cloud_restamped'),
        ('odom',       '/odom/filtered'),
        ('grid_map',   '/map'),
    ]

    rtabmap_parameters = {
        'frame_id':'rover_base_origin',
        'use_sim_time':True,
        'subscribe_depth':False,
        'subscribe_scan_cloud': True,
        'use_action_for_goal':True,
        'Reg/Force3DoF':'true',
        'Grid/CellSize': "0.2",  # Voxel downsampling
        'Grid/RayTracing':'true', # Fill empty space
        'Grid/Sensor':'0',
        'Grid/FromDepth':'False',
        'Grid/3D': 'true', # Use 2D occupancy
        'Grid/RangeMax':'5',
        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight':'0.5', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight':'1.5',  # All points over 1 meter are ignored
        'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D)
        'RGBD/CreateOccupancyGrid':"True",
        'RGBD/DepthDecimationr': "4",  # Reduces the depth image resolution before generating the point cloud
        'RGBD/DepthMax': "3.0",  # Filter the depth image
        "delete_db_on_start": True,
    }
    rtabmap_remappings = [
        ('odom', '/odometry/filtered'),
        # ('rgb/image', '/camera/d455/color/image'),
        # ('rgb/camera_info', '/camera/d455/color/camera_info'),
        ('scan_cloud', '/utlidar/cloud_restamped')
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            choices=['true', 'false'],
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'scan_cloud_topic', default_value='/utlidar/cloud_deskewed',
            description='LiDAR point cloud topic. Switch to /utlidar/cloud if deskewed is unavailable.',
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

        # RTAB-Map SLAM (fresh map — deletes existing database on startup)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            arguments=['--delete_db_on_start'],
            parameters=rtabmap_params,
            remappings=rtabmap_remaps,
            condition=IfCondition(LaunchConfiguration('delete_db_on_start')),
        ),

        # RTAB-Map SLAM (resume existing map from database)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=rtabmap_parameters,
            remappings=rtabmap_remappings,
            condition=UnlessCondition(LaunchConfiguration('delete_db_on_start')),
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
