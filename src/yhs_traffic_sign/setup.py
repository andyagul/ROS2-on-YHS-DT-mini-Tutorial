from glob import glob
import os

from setuptools import setup, find_packages

package_name = 'yhs_traffic_sign'

setup(
    name=package_name,
    version='0.1.0',
    # 显式声明包位置（避免 ament_python 找不到）
    packages=find_packages(exclude=['test']),
    package_dir={'': '.'},
    # 确保所有 .py 文件被打包
    include_package_data=True,
    package_data={
        package_name: ['*.py'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'param'), glob('param/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YHS Robotics',
    maintainer_email='yhs@todo.todo',
    description='交通标识牌识别与响应功能包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_sign_response = yhs_traffic_sign.traffic_sign_response:main',
            'traffic_sign_detector_node = yhs_traffic_sign.traffic_sign_detector:main',
            'distance_estimator_node = yhs_traffic_sign.distance_estimator:main',
        ],
    },
)
