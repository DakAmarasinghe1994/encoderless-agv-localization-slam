from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',

            parameters=[{

                'laser_scan_topic':
                    '/scan',

                'odom_topic':
                    '/odom_rf2o',

                'base_frame_id':
                    'base_link',

                'odom_frame_id':
                    'odom',

                'publish_tf':
                    True,

                'freq':
                    20.0  # May need to change it
            }]
        )

    ])