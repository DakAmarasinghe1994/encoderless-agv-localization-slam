from launch import LaunchDescription

from launch_ros.actions import Node

import os

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    config = os.path.join(

        get_package_share_directory(

            'bno055_imu'),

        'config',

        'imu.yaml'

    )

    return LaunchDescription([

        Node(

            package='bno055_imu',

            executable='imu_serial_node',

            name='imu_serial_node',

            output='screen',

            parameters=[config]

        )

    ])
