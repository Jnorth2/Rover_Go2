from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
                'pose_covariance_diagonal':  [0.01, 0.01, 0.1, 0.1, 0.1, 0.05],
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

        # EKF: fuses /utlidar/robot_odom_cov + /utlidar/imu_with_covariance.
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
