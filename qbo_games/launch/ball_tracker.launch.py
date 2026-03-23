#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =======================================================================================================
#     LAUNCH FILE — Neo Ball Tracker
#
#     Lance le nœud qbo_ball_tracker_node avec sa configuration YAML.
#
#     USAGE:
#       ros2 launch qbo_games ball_tracker.launch.py
#       ros2 launch qbo_games ball_tracker.launch.py debug:=false
#       ros2 launch qbo_games ball_tracker.launch.py scale:=0.25
#
#     AUTEUR : Vincent FOUCAULT / elpimous12
#     DATE   : 2025
# =======================================================================================================

import os
from ament_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('qbo_games')

    # ---- Fichier de configuration ----
    config_file = os.path.join(pkg_share, 'config', 'ball_tracker_config.yaml')

    # ---- Arguments de lancement overridables ----
    # vince : permet d'ajuster debug et scale sans recompiler
    arg_debug = DeclareLaunchArgument(
        'debug',
        default_value='true',
        description='Publier ou non limage debug /ball_tracker_debug'
    )

    arg_scale = DeclareLaunchArgument(
        'scale',
        default_value='0.5',
        description='Facteur de réduction de limage pour le traitement HSV (0.25 à 1.0)'
    )

    # ---- Nœud principal ----
    ball_tracker_node = Node(
        package    = 'qbo_games',
        executable = 'qbo_ball_tracker',
        name       = 'qbo_ball_tracker_node',
        output     = 'screen',
        emulate_tty=True,
        parameters = [
            config_file,
            # vince : les args CLI surchargent le YAML si précisés
            {
                'perf.publish_debug': LaunchConfiguration('debug'),
                'perf.scale_factor' : LaunchConfiguration('scale'),
            }
        ],
    )

    return LaunchDescription([
        arg_debug,
        arg_scale,
        ball_tracker_node,
    ])
