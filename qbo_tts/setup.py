from setuptools import setup
import os
from glob import glob

package_name = 'qbo_tts'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files (tts.yaml)
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # TTS model (optionnel - peut être volumineux)
        # (os.path.join('share', package_name, 'tts_model'), glob('tts_model/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent FOUCAULT',
    maintainer_email='vincent@example.com',
    description='ROS2 TTS Node using Piper for robot Néo/QBo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qbo_tts_node = qbo_tts.qbo_tts:main',
        ],
    },
)
