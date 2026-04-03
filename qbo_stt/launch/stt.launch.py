#!/usr/bin/env python3
"""
Launch — qbo_stt
Vincent Foucault — Avril 2026
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_dir = get_package_share_directory('qbo_stt')
    config  = os.path.join(pkg_dir, 'config', 'params.yaml')

    audio_arg = DeclareLaunchArgument(
        'audio_enabled', default_value='true',
        description='Activer le micro au démarrage'
    )

    node = Node(
        package='qbo_stt',
        executable='stt_node',
        name='stt_node',
        output='screen',
        parameters=[config],
        emulate_tty=True,
    )

    return LaunchDescription([audio_arg, node])
