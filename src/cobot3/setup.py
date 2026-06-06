from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cobot3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 아래 항목이 없으면 추가
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='rokey@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 'pick_and_place_banana_pilz = cobot3.pick_and_place_banana_pilz:main',
            'yolo11_node = cobot3.yolo11_node:main',
            'coord_transform    = cobot3.coord_transform_node:main',
            'yolo_detector = cobot3.yolo_detector:main',
            'depth_to_3d = cobot3.depth_to_3d:main', 
            'tf_transformer = cobot3.tf_transformer:main',
            'pick_and_place_banana = cobot3.pick_and_place_banana:main',
            'pick_and_place_banana_xyz= cobot3.pick_and_place_banana_ros2con_xyz:main',
            'pick_and_place_banana_joint= cobot3.pick_and_place_banana_ros2con_joint:main',
            'pick_and_place_yolo_banana= cobot3.pick_and_place_yolo_banana:main',
            'franka_moveit_control = cobot3.franka_moveit_control:main',
            'm0609_color_detector = cobot3.m0609_color_detector:main',
            'nav_to_pose = cobot3.nav_to_pose:main',
        ],
    },
)
