import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('gnss_raw_driver'),
        'config', 'default.yaml'
    )

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=config),
        DeclareLaunchArgument('device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('protocol', default_value='auto'),
        DeclareLaunchArgument('frame_id', default_value='gnss'),
        DeclareLaunchArgument('publish_raw_binary', default_value='true'),
        Node(
            package='gnss_raw_driver',
            executable='gnss_raw_driver_node',
            name='gnss_raw_driver',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'device': LaunchConfiguration('device'),
                    'baud_rate': ParameterValue(
                        LaunchConfiguration('baud_rate'), value_type=int),
                    'protocol': LaunchConfiguration('protocol'),
                    'frame_id': LaunchConfiguration('frame_id'),
                    'publish_raw_binary': ParameterValue(
                        LaunchConfiguration('publish_raw_binary'), value_type=bool),
                },
            ],
            output='screen',
        ),
    ])
