from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'qbo_games'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # vince : registration ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # vince : fichiers launch
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # vince : fichiers config YAML
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent FOUCAULT',
    maintainer_email='elpimous12@gmail.com',
    description='Package de jeux pour le robot Néo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # vince : chifoumi game
            'qbo_chifumi = qbo_games.qbo_chifumi:main',
            # vince : ball tracker
            'qbo_ball_tracker = qbo_games.qbo_ball_tracker:main',
        ],
    },
)
