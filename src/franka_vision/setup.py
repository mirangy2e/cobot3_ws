from setuptools import find_packages, setup

package_name = 'franka_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'franka_ros2_control = franka_vision.franka_ros2_control:main',
            'franka_yolo_banana = franka_vision.franka_yolo_banana:main',
            'franka_depth_to_3d  = franka_vision.franka_depth_to_3d:main',
            'franka_tf_transform = franka_vision.franka_tf_transform:main',
            'pick_and_place_banana_joint  = franka_vision.pick_and_place_banana_joint:main',
            'pick_and_place_banana_xyz  = franka_vision.pick_and_place_banana_xyz:main',
            'pick_and_place_yolo_banana  = franka_vision.pick_and_place_yolo_banana:main',
        ],
    },
)
