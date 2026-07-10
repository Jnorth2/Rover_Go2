"""
GPS base station launch (groundstation).

Runs a single self-contained node that opens /dev/tty_rtkgps, configures the
ZED-F9P for survey-in via UBX messages, and publishes RTCM3 correction frames
to /rtcm once survey-in completes.  CycloneDDS carries /rtcm to the Go2 where
the rover's ublox_gps_node receives and forwards them to the rover GPS.

Survey-in progress is logged to the console every second.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('go2_odom'), 'config', 'gps_base.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            choices=['true', 'false'],
        ),

        Node(
            package='go2_odom',
            executable='gps_base_node.py',
            name='gps_base_node',
            output='screen',
            parameters=[
                config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
