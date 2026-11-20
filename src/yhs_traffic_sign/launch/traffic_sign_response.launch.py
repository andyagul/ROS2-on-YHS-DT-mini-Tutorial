"""交通标识牌识别与响应 - Launch文件

可直接运行: ros2 launch yhs_traffic_sign traffic_sign_response.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # 获取参数文件路径
    pkg_share = get_package_share_directory('yhs_traffic_sign')
    default_params_file = os.path.join(pkg_share, 'param', 'traffic_sign_params.yaml')

    # Launch参数声明
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='参数文件路径',
    )

    enable_control_arg = DeclareLaunchArgument(
        'enable_control',
        default_value='true',
        description='是否启用实际车辆控制 (调试时设为false)',
    )

    # 主节点
    traffic_sign_node = Node(
        package='yhs_traffic_sign',
        executable='traffic_sign_response',
        name='traffic_sign_response',
        parameters=[
            LaunchConfiguration('params_file'),
            {'enable_control': LaunchConfiguration('enable_control')},
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        params_file_arg,
        enable_control_arg,
        traffic_sign_node,
    ])
