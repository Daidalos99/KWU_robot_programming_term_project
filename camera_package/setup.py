#!/usr/bin/env python3

import glob
import os

from setuptools import setup
from setuptools import find_packages

package_name = 'camera_package'
share_dir = 'share/' + package_name

setup(
    name=package_name,
    version='0.6.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (share_dir, ['package.xml']),
        (share_dir + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
        (share_dir + '/param',  glob.glob(os.path.join('param', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='heeyang',
    maintainer_email='huiyang1022@naver.com',
    description='Camera and Database Processing Package',
    license='Apache License, Version 2.0',
    keywords=['ROS'],
    classifiers=[
        'Intendted Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'database = camera_package.robot.main:main_database',
            # 'database = camera_package.robot.database:main',

            'user_interface_input = camera_package.user_interface.get_input_info:main',
            'user_interface_output = camera_package.user_interface.get_output_info:main',
            'user_interface_camera = camera_package.user_interface.get_camera:main',
            'send2gui = camera_package.user_interface.send2gui:main',
        ],
    },
)
