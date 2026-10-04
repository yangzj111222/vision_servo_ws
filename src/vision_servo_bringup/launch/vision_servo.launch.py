from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    vision_pkg = get_package_share_directory('vision_servo_vision')
    control_pkg = get_package_share_directory('vision_servo_control')
    grab_pkg = get_package_share_directory('vision_servo_grab')

    vision_params = os.path.join(vision_pkg, 'config', 'params.yaml')
    control_params = os.path.join(control_pkg, 'config', 'control_params.yaml')
    grab_params = os.path.join(grab_pkg, 'config','grab_params.yaml')

    return LaunchDescription([
        Node(
            package='vision_servo_vision',
            executable='vision_node',
            name='vision_node',
            output='screen',
            parameters=[vision_params],
        ),
        Node(
            package='vision_servo_control',
            executable='control_node',
            name='align_controller',
            output='screen',
            parameters=[control_params],
        ),
        Node(
            package='vision_servo_grab_cpp',
            executable='grab_node',
            name='grab_node',
            output='screen',
            parameters=[grab_params],
        ),
    ])