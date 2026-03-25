# launch/invert_image_launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    return LaunchDescription([
        # 3) Node du driver USB-Cam
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[{
                # configure le périphérique vidéo
                'video_device':    '/dev/video0',
                'image_width':     640,
                'image_height':    480,
                'framerate':       50.0,
                'pixel_format':    'yuyv'
            }]
        ),

        # 4) Node pour lancer votre script Python d'inversion d'image
        Node(
    		executable='qbo_invert_image',
    		name='invert_image_node',
    		output='screen',
	)
    ])

