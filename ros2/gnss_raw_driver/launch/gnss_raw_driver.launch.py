import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gnss_raw_driver'),
        'config', 'default.yaml'
    )

    return LaunchDescription([
        Node(
            package='gnss_raw_driver',
            executable='gnss_raw_driver_node',
            name='gnss_raw_driver',
            parameters=[config],
            output='screen',
        ),
    ])
