from setuptools import setup
import os
from glob import glob

package_name = 'vision_servo_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Vision servo alignment controller',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'control_node = vision_servo_control.control_node:main',
        ],
    },
)