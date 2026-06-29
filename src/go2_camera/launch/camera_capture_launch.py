import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

ARM_IP = '192.168.123.100'

def generate_launch_description():
    # Replace these with your camera serial numbers
    # You can find them with `rs-enumerate-devices`

    d435_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d435',
        parameters=[{
            "camera_name": "d435",
            "serial_no":"238722072340",

            "depth_module.depth_profile": "848x480x30",
            "rgb_camera.color_profile": "1280x720x30",
            #enable streams
            "enable_color": True,
            "enable_depth": True,
            "enable_accel": True,
            "enable_gyro": True,
            "enable_infra1": False,
            "enable_infra2": False,
            "enable_rgbd": True,

            "enable_sync": True,

            #Point cloud
            "pointcloud.enable": True,
            "pointcloud__neon_.enable": True,
            "pointcloud__neon_.stream_filter": 2,
            "pointcloud.stream_filter": 2,
            "align_depth.enable": True,
            #Decimation Filter
            "decimation_filter": True,
            "decimation_filter.filter_magnitude": 2,
            #spacial Filter
            "spatial_filter.enable": True,
            "spatial_filter.filter_magnitude": 2,
            "spatial_filter.filter_smooth_alpha": 0.55,
            "spatial_filter.filter_smooth_delta": 30,
            "spatial_filter.holes_fill": 1,   # 1 = small/conservative hole fill
            "spatial_filter.frames_queue_size": 16,
            #temporal Filter
            "temporal_filter.enable": True,
            "temporal_filter.filter_smooth_alpha": 0.45,
            "temporal_filter.filter_smooth_delta": 25,
            "temporal_filter.holes_fill": 0,  # low persistence; use 0 if moving fast
            "temporal_filter.frames_queue_size": 16,

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
            'cap_width': 1920,
            'cap_height': 1080,
            'cap_framerate': 30,
            'speed_preset': 'ultrafast',
            'bitrate': 4000,
            'stream_width': 1920,
            'stream_height': 1080,
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

