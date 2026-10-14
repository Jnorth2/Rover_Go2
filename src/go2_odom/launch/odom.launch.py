from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
            'use_visual_odom', default_value='true',
            choices=['true', 'false'],
            description='Run D435 visual odometry and feed it to the EKF (odom1). '
                        'If the camera is not running the EKF simply runs without it.',
        ),

        # /utlidar/robot_odom has zero covariances; inject reasonable diagonal values
        # so robot_localization can properly weight this measurement source.
        # Pose diagonal [x, y, z, roll, pitch, yaw] in m² / rad².
        # Twist diagonal [vx, vy, vz, vroll, vpitch, vyaw] in (m/s)² / (rad/s)².
        Node(
            package='go2_odom',
            executable='odom_covariance_node.py',
            name='odom_covariance_node',
            output='screen',
            remappings=[
                ('odom_in',  '/utlidar/robot_odom'),
                ('odom_out', '/utlidar/robot_odom_cov'),
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                # roll/pitch tightened from 0.1 (≈18°) to 0.005 (≈4°) now that the
                # EKF fuses them for 3D SLAM
                'pose_covariance_diagonal':  [0.01, 0.01, 0.05, 0.005, 0.005, 0.05],
                'twist_covariance_diagonal': [0.01, 0.05, 0.1, 0.1, 0.1, 0.02],
            }],
        ),

        # /utlidar/imu has zero covariances; inject values before feeding to EKF.
        # orientation_covariance_diag < 0 signals "orientation not provided" (REP-145).
        Node(
            package='go2_odom',
            executable='imu_covariance_node.py',
            name='imu_covariance_node',
            output='screen',
            remappings=[
                ('imu_in',  '/utlidar/imu'),
                ('imu_out', '/utlidar/imu_with_covariance'),
            ],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'orientation_covariance_diag':        -1.0,
                'angular_velocity_covariance_diag':    0.001,
                'linear_acceleration_covariance_diag': 0.01,
            }],
        ),

        # Visual odometry from the D435 → /camera/d435/visual_odom (EKF odom1).
        # publish_tf is off because the EKF owns odom → base_link. The driver's
        # /camera/d435/rgbd is realsense2_camera_msgs/RGBD, which rtabmap can't
        # read, so the color, aligned depth and camera info topics are used instead.
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'frame_id': 'base_link',
                'odom_frame_id': 'odom',
                'publish_tf': False,
                'approx_sync': True,
                'wait_for_transform': 0.2,
                # Re-initialize on the next frame after tracking is lost; only
                # velocity is fused, so the pose jump on reset doesn't matter.
                'Odom/ResetCountdown': '1',
            }],
            remappings=[
                ('rgb/image',       '/camera/d435/color/image_raw'),
                ('depth/image',     '/camera/d435/aligned_depth_to_color/image_raw'),
                ('rgb/camera_info', '/camera/d435/color/camera_info'),
                ('odom',            '/camera/d435/visual_odom'),
            ],
            condition=IfCondition(LaunchConfiguration('use_visual_odom')),
        ),

        # EKF: fuses /utlidar/robot_odom_cov + /utlidar/imu_with_covariance
        # + /camera/d435/visual_odom (velocities only).
        # Publishes /odom and broadcasts odom → base_link TF.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_node',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('go2_odom'), 'config', 'ekf.yaml',
                ]),
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
