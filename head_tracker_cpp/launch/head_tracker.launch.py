from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('head_tracker_cpp'),
        'config', 'head_tracker.yaml'
    )

    return LaunchDescription([
        Node(
            package='head_tracker_cpp',
            executable='head_tracker_node',
            name='head_tracker_node',
            output='screen',
            parameters=[cfg],
            remappings=[
                # uncomment / edit if your topics differ
                # ('/target_roi', '/target_roi'),
                # ('/joint_states', '/joint_states'),
                # ('/cmd_joints',   '/cmd_joints'),
            ],
        )
    ])
