# File: launch/qbo_tts_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Récupération du répertoire de partage du package
    pkg_share = get_package_share_directory('qbo_tts')

    # Chemin vers le fichier de configuration YAML
    config_file = os.path.join(pkg_share, 'config', 'tts.yaml')
    
    # Fallback vers le dossier src en développement
    if not os.path.exists(config_file):
        config_file = '/home/nvidia/qbo_ws/src/qbo_tts/config/tts.yaml'

    return LaunchDescription([
        Node(
            package='qbo_tts',
            executable='qbo_tts_node',
            name='qbo_tts_node',
            output='screen',
            parameters=[config_file],
        )
    ])
