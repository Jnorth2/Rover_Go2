from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Nav2 servers brought up by the lifecycle manager. Same set as nav2_bringup's
# navigation_launch.py minus route_server and docking_server, which the Go2 doesn't use.
NAV2_NODES = [
    'controller_server',
    'smoother_server',
    'planner_server',
    'behavior_server',
    'velocity_smoother',
    'collision_monitor',
    'bt_navigator',
    'waypoint_follower',
]


def generate_launch_description():
    """
    Everything that runs on the Go2's Jetson for autonomous navigation:
    camera, odometry (EKF), robot description, RTAB-Map SLAM (via mapping.launch.py),
    lidar obstacle segmentation, Nav2, and the joystick / cmd_vel teleop bridge.

    Pair with groundstation_navigation.launch.py on the laptop (RViz + joystick).
    """
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    # Same remaps as nav2_bringup: controller and behaviors feed the velocity
    # smoother, whose output goes through the collision monitor to /cmd_vel.
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    nav2_remaps = {
        'controller_server': tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
        'behavior_server': tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
        'velocity_smoother': tf_remaps + [('cmd_vel', 'cmd_vel_nav')],
    }
    nav2_packages = {
        'controller_server': 'nav2_controller',
        'smoother_server': 'nav2_smoother',
        'planner_server': 'nav2_planner',
        'behavior_server': 'nav2_behaviors',
        'velocity_smoother': 'nav2_velocity_smoother',
        'collision_monitor': 'nav2_collision_monitor',
        'bt_navigator': 'nav2_bt_navigator',
        'waypoint_follower': 'nav2_waypoint_follower',
    }
    nav2_nodes = [
        Node(
            package=nav2_packages[name],
            executable=name,
            name=name,
            output='screen',
            respawn=True,
            respawn_delay=2.0,
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=nav2_remaps.get(name, tf_remaps),
        )
        for name in NAV2_NODES
    ] + [
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': NAV2_NODES,
            }],
        ),
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            choices=['true', 'false'],
            description='Use simulation clock',
        ),
        DeclareLaunchArgument(
            'delete_db_on_start', default_value='true',
            choices=['true', 'false'],
            description='Start a fresh RTAB-Map database (default). '
                        'false = keep mapping into the existing ~/.ros/rtabmap.db',
        ),
        DeclareLaunchArgument(
            'use_camera', default_value='true',
            choices=['true', 'false'],
            description='Start the RealSense D435 and add it to RTAB-Map and the EKF',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('go2_nav'), 'config', 'nav2_params.yaml',
            ]),
            description='Nav2 parameters file',
        ),
        DeclareLaunchArgument(
            'nav2_start_delay', default_value='5.0',
            description='Seconds to wait before starting Nav2, so the EKF and RTAB-Map '
                        'are publishing odom → base_link and map → odom when the '
                        'costmaps activate',
        ),

        # Camera, cloud restamp, EKF, robot description and RTAB-Map SLAM
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('go2_nav'), 'launch', 'mapping.launch.py',
                ]),
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'use_camera': LaunchConfiguration('use_camera'),
                'delete_db_on_start': LaunchConfiguration('delete_db_on_start'),
                'use_rviz': 'false',
            }.items(),
        ),

        # Split the restamped lidar cloud into ground and obstacles for the Nav2
        # costmaps and collision monitor. Segmentation is by surface normals, using
        # the same Grid/ settings as rtabmap_mapping.yaml. Runs in base_footprint
        # (URDF ground-level frame under the trunk) so the footprint box below
        # spans from the feet up over the body.
        Node(
            package='rtabmap_util',
            executable='obstacles_detection',
            name='obstacles_detection',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'frame_id': 'base_footprint',
                'wait_for_transform': 0.3,
                'Grid/RangeMax': '6',
                'Grid/NormalsSegmentation': 'true',
                'Grid/NormalK': '20',
                'Grid/MaxGroundAngle': '45',
                'Grid/MaxObstacleHeight': '1.5',
                'Grid/ClusterRadius': '0.15',
                # The UTlidar cloud is sparse; the default of 10 drops thin obstacles
                'Grid/MinClusterSize': '5',
                # Drop returns off the Go2's own body and legs
                'Grid/FootprintLength': '0.75',
                'Grid/FootprintWidth': '0.42',
                'Grid/FootprintHeight': '0.8',
            }],
            remappings=[
                ('cloud', '/utlidar/cloud_restamped'),
                ('ground', '/utlidar/ground'),
                ('obstacles', '/utlidar/obstacles'),
            ],
        ),

        TimerAction(
            period=LaunchConfiguration('nav2_start_delay'),
            actions=nav2_nodes,
        ),

        # /joy (from the groundstation) and /cmd_vel (from Nav2) → Go2 sport API.
        # Joystick input overrides Nav2; △ toggles whether Nav2 may drive.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('go2_control'), 'launch', 'go2_joy_launch.py',
                ]),
            ]),
        ),
    ])
