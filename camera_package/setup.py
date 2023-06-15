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
            # 차단기 등록자 노드 하나
            'argument_quick_test = camera_package.circuit_breaker.register:main_00',
            'circuit_breaker_register = camera_package.circuit_breaker.register:main_01',
            # 차단기 해제자 노드 하나
            'circuit_breaker_deleter = camera_package.circuit_breaker.deleter:main_00',
            #* 로봇의 노드 하나 -> 임시로 만들어 주기
            'robot_buffer = camera_package.robot.robot_buffer:main',
            #* 로봇 코어 하나
            'robot_core = camera_package.robot.main:main',
            'database = camera_package.robot.main:main_database',
            # 'database = camera_package.robot.database:main',

            'user_interface_input = camera_package.user_interface.get_input_info:main',
            'user_interface_output = camera_package.user_interface.get_output_info:main',
            #! process의 경우 새롭게 하나 더 만들어서
            #! 카메라 버퍼, 수신기의 enter 버퍼, 수신기의 exit 버퍼 간의 관계를 통하여
            #! 꼬리잡기하는 차량을 검출한다.
            'user_interface_process = camera_package.user_interface.get_process:main',
            'user_interface_camera = camera_package.user_interface.get_camera:main',
            'camera_server = camera_package.user_interface.camera_server:main',
        ],
    },
)
