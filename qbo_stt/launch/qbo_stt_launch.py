from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='qbo_stt',                     # Nom du package Python ROS 2
            executable='qbo_stt_node',             # Nom défini dans setup.py (console_scripts)
            name='qbo_stt_node',                   # Nom du noeud ROS 2 dans le graphe
            output='screen',                       # Affiche les logs dans le terminal
            parameters=[
                {'server_uri': 'localhost:50051'},  # URI du serveur ASR
                {'use_ssl': False},                # Connexion SSL ou non
                {'ssl_cert': ''},                  # Chemin vers le certificat si SSL
                {'language_code': 'fr-FR'},        # Langue cible
                {'model_name': 'conformer-fr-FR-asr-streaming-asr-bls-ensemble'},  # Modèle ASR
                {'sample_rate': 16000},            # Fréquence d’échantillonnage
                {'chunk_size': 300},               # Taille des blocs audio
                {'input_device': 0},               # Index du périphérique micro
            ],
        ),
    ])

