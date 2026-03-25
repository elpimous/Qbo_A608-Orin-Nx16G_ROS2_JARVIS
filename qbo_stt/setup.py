from setuptools import find_packages, setup

package_name = 'qbo_stt'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/qbo_stt_launch.py']),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='elpimous12@gmail.com',
    description='Speech-to-Text node for Neo robot using NVIDIA Riva ASR',
    license='Apache-2.0',  # CORRIGÉ : Format standardisé
    entry_points={
    'console_scripts': [
        'qbo_stt_node = qbo_stt.qbo_stt:main',  # ← très important
        ],
        },
)