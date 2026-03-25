from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'qbo_tts'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        # Fichiers requis pour l'index des packages
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installation du package.xml
        ('share/' + package_name, ['package.xml']),
        # Fichiers de launch
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        # Ajout du modèle TTS si présent
        ('share/' + package_name + '/tts_model', glob('tts_model/*') if os.path.exists('tts_model') else []),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std-msgs',
    ],
    zip_safe=True,
    maintainer='vincent.foucault',
    maintainer_email='elpimous12@gmail.com',
    description='Package de synthèse vocale pour le robot Néo',
    license='Apache-2.0',

    entry_points={
        'console_scripts': [
            'qbo_tts_node = qbo_tts.qbo_tts:main',
        ],
    },
)