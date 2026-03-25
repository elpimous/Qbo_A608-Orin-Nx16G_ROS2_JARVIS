from setuptools import setup
import os
from glob import glob

package_name = 'qbo_camera'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # AJOUTÉ : Fichiers requis pour l'index des packages
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Installation du package.xml
        (os.path.join('share', package_name), ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='elpimous12@gmail.com',  # CORRIGÉ : Email sans caractères spéciaux
    description='Camera package for QBO robot Néo',  # AMÉLIORÉ : Description plus claire
    license='Apache-2.0',  # CORRIGÉ : Format standardisé
    # CORRIGÉ : Remplacé tests_require par extras_require
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'qbo_invert_image = qbo_camera.qbo_invert_image:main',
        ],
    },
)