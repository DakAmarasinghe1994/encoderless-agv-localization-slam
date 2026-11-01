from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    imu_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'imu_link'
        ]
    )

    return LaunchDescription([
        imu_tf
    ])