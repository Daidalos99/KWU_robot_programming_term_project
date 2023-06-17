#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
          get_package_share_directory('camera_package'),
          'param',
          'database_config.yaml'
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Full path of parameter file.'),
        Node(
          package='camera_package',
          executable='database',
          name='database',
          parameters=[param_dir],
          output='screen'
        ),

        Node(
          package='camera_package',
          executable='user_interface_input',
          name='user_interface_input',
          parameters=[param_dir],
          output='screen'
        ),

        Node(
          package='camera_package',
          executable='user_interface_output',
          name='user_interface_output',
          parameters=[param_dir],
          output='screen'
        ),

        Node(
          package='camera_package',
          executable='user_interface_camera',
          name='user_interface_camera',
          parameters=[param_dir],
          output='screen'
        ),
    ])
