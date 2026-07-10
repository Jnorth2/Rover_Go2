"""
GPS launch for the Go2 (rover).

Starts the ublox_gps_node in rover mode.  The node subscribes to the absolute
topic /rtcm (rtcm_msgs/msg/Message) and forwards any RTCM3 correction bytes it
receives to the GPS hardware via USB, enabling an RTK fix.

RTCM corrections are expected to arrive over the CycloneDDS network from the
groundstation's gps_base.launch.py + rtcm_relay_node running on the same domain.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare('go2_odom'), 'config', 'gps_rover.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            choices=['true', 'false'],
        ),

        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[
                config,
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),
    ])
