from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('vision_servo_control')
    params_file = os.path.join(pkg_dir, 'config', 'control_params.yaml')

    return LaunchDescription([
        Node(
            package='vision_servo_control',
            executable='control_node',
            name='align_controller',
            output='screen',
            parameters=[params_file],
        ),
    ])