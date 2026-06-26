import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ARM_IP = '239.0.0.1'

def generate_launch_description():
    # Replace these with your camera serial numbers
    # You can find them with `rs-enumerate-devices`

    d435_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435',
        parameters=[{
            "camera_name": "d435",
            "depth_width": 1280,
            "depth_height": 720,
            "color_width": 1280,
            "color_height": 720,
            "pointcloud.enable": True,
            "pointcloud__neon_.enable": True,
            "pointcloud__neon_.stream_filter": 2,
            "align_depth.enable": True,
            #"enable_rgbd": True,
            "decimation_filter": True,
            "decimation_filter.filter_magnitude": 2,
            "enable_sync": True,
            "pointcloud.stream_filter": 2,
            "enable_color": True,
            # "enable_depth": True,
            # "serial_no":"218622273613",
            "depth_fps": 30,
            "rgb_fps": 30,
        }],
        output='screen',
    )

    front_camera_node = Node(
        package='go2_camera',
        namespace='go2_camera',
        executable='camera_capture',
        name='front_camera',
        parameters=[{
            'device': '/dev/go2/camera_front_camera',
            'cap_width': 1920,
            'cap_height': 1080,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 1920,
            'stream_height': 1080,
            'fec_percentage': 30,
            'udp_host': ARM_IP,
            'udp_port': 42069,
            'mux_port': 20002
        }],
        respawn=True
    )

    gripper_view = Node(
        package='go2_camera',
        namespace='go2_camera',
        executable='camera_ros2_conversion',
        name='realsense_conversion',
        parameters=[{
            'image_topic': '/camera/d435/color/image_raw',
            'cap_width': 1280,
            'cap_height': 720,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 1280,
            'stream_height': 720,
            'fec_percentage': 30,
            'udp_host': ARM_IP,
            'udp_port': 42074,
            'mux_port': 20000
        }],
        respawn=True,
    )
    



    return LaunchDescription([
        
        d435_node,
        gripper_view,
    ])

