from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
    
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=[ '0.23', '0', '0.2', '0', '0', '0', '1', 'base_link', 'camera_link']
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/camera/depth/points'),
                        ('scan', '/scan1')],
            parameters=[{
                'target_frame': 'camera_link',
                'transform_tolerance': 0.01,
                'min_height': -0.1,
                'max_height': 2.0,
                'angle_min': -1.5708, 
                'angle_max': 1.5708, 
                'angle_increment': 0.0087,
                'scan_time': 0.3333,
                'range_min': 0.05,
                'range_max': 1.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
