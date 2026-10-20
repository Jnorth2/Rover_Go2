from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_control',
            executable='joy_teleop.py',
            name='go2_joy_teleop',
            output='screen',
            parameters=[{
                # Stick axes (joy/SDL GameController layout)
                #   0 = left stick X, 1 = left stick Y
                #   2 = right stick X, 3 = right stick Y
                'axis_vx': 1,       # left stick Y  → forward/backward
                'axis_vy': 0,       # left stick X  → strafe left/right
                'axis_vyaw': 2,     # right stick X → turn left/right
                'axis_pitch': 3,    # right stick Y → look up/down
                # Button (PS layout: 0=X  1=O  2=□  3=△)
                'btn_sit_stand': 2,  # Square = toggle sit/stand
                'btn_nav_toggle': 3,  # Triangle = allow/block Nav2 /cmd_vel
                'nav_enabled': True,
                # Speed limits
                'max_linear': 0.8,   # m/s
                'max_angular': 1.0,  # rad/s
                'max_pitch': 0.3,    # rad, body tilt at full stick
                'deadzone': 0.08,
                # Stale /joy or /cmd_vel counts as zero (link loss stops the robot)
                'joy_timeout': 0.5,
                'cmd_vel_timeout': 0.5,
                # Nav2 commands are raised to at least these (Go2 steps discretely)
                'nav_min_linear': 0.15,   # m/s
                'nav_min_angular': 0.3,   # rad/s
                'nav_deadband_linear': 0.03,
                'nav_deadband_angular': 0.05,
            }],
        ),
    ])
