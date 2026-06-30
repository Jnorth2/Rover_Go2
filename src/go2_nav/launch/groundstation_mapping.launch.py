from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Groundstation-only launch: starts RViz to visualize the mapping stack
    running on the robot. Requires DDS network connectivity to the Jetson.

    Uses groundstation_mapping.rviz which has no RobotModel display (avoids
    needing go2_description meshes installed on the groundstation machine).
    """
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                FindPackageShare('go2_nav'), 'config', 'groundstation_mapping.rviz',
            ]),
            description='Path to RViz config file',
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
        ),
    ])
