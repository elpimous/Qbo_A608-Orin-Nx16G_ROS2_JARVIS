# File: launch/qbo_tts_launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Récupération du répertoire de partage du package
    pkg_share = get_package_share_directory('qbo_tts')

    # Chemins du modèle via variables d'environnement ou chemins relatifs
        # Pour développement local, chemin vers le modèle dans src
    default_model = os.getenv('PIPER_MODEL_PATH', os.path.join(get_package_share_directory('qbo_tts'), 'tts_model', 'axel.onnx'))
    # Chemin de secours direct vers le dossier src (en dev) :
    if not os.path.exists(default_model):
        default_model = '/home/nvidia/qbo_ws/src/qbo_tts/tts_model/axel.onnx'
    # Config associée
    default_config = os.getenv('PIPER_CONFIG_PATH', default_model + '.json')

    # Index du périphérique de sortie audio (par défaut 1 USB Audio Device)
    default_output_index = int(os.getenv('AUDIO_DEVICE_INDEX', 1))


    return LaunchDescription([
        Node(
            package='qbo_tts',
            executable='qbo_tts_node',  # tel que défini dans setup.py
            name='tts_',
            output='screen',
            parameters=[
                {'model_path': default_model},
                {'config_path': default_config},
                {'output_device_index': default_output_index},
            ],
        )
    ])