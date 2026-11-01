# Start the complete AGV mapping stack and RViz

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_package_launch(package_name, launch_file):
    
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        'launch',
        launch_file,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path)
    )


def generate_launch_description():
    agv_share = get_package_share_directory('agv_localization')
    default_rviz_config = os.path.join(agv_share, 'config', 'agv.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz2 when true.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Absolute path to the RViz configuration file.',
        ),

        include_package_launch(
            'ydlidar_ros2_driver', 'ydlidar_launch.py'
        ),
        include_package_launch(
            'rf2o_laser_odometry', 'rf2o_laser_odometry.launch.py'
        ),
        include_package_launch('bno055_imu', 'imu.launch.py'),
        include_package_launch('agv_localization', 'imu_tf.launch.py'),
        include_package_launch('agv_localization', 'ekf.launch.py'),
        include_package_launch('agv_localization', 'slam.launch.py'),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}],
            condition=IfCondition(use_rviz),
        ),
    ])

# Use the following command to Save a Slam Map
# ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
