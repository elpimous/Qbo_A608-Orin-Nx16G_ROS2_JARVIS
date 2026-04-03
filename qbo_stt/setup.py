from setuptools import setup
import os
from glob import glob

package_name = 'qbo_stt'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Vincent Foucault',
    maintainer_email='elpimous12@gmail.com',
    description='STT node pour Néo — Parakeet TDT TensorRT FP16',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt_node = qbo_stt.stt_node:main',
        ],
    },
)
