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
                # Stick axes (joy_linux DS4/DualSense defaults)
                #   0 = left stick X, 1 = left stick Y
                #   3 = right stick X, 4 = right stick Y
                'axis_vx': 1,       # left stick Y  → forward/backward
                'axis_vy': 0,       # left stick X  → strafe left/right
                'axis_vyaw': 3,     # right stick X → turn left/right
                # Button (PS layout: 0=X  1=O  2=△  3=□)
                'btn_sit_stand': 3,  # Square = toggle sit/stand
                # Speed limits
                'max_linear': 0.8,   # m/s
                'max_angular': 1.0,  # rad/s
                'deadzone': 0.08,
            }],
        ),
    ])
