#!/usr/bin/python3

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import LogInfo

import os

def generate_launch_description():
    share_dir = get_package_share_directory('nvilidar_ros2')
    parameter_file = LaunchConfiguration('params_file_lidar')

    params_declare = DeclareLaunchArgument('params_file_lidar',
                                           default_value=os.path.join(
                                               share_dir, 'params', 'nvilidar.yaml'),
                                           description='FPath to the ROS2 parameters file to use.')

    driver_node = Node(package='nvilidar_ros2',
                                executable='nvilidar_ros2_node',
                                name='nvilidar_ros2_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[parameter_file],
                                namespace='/',
                                )
    tf2_node = Node(package='tf2_ros',
                    executable='static_transform_publisher',
                    name='static_tf_pub_laser',
                    arguments=['0', '0', '0.33','0', '0', '0', '1','base_link','laser_link'],
                    )

    return LaunchDescription([
        params_declare,
        driver_node,
        tf2_node
    ])
