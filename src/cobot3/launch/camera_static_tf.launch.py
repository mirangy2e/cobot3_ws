# ~/cobot3_ws/src/cobot3/launch/camera_static_tf.launch.py
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import SetParameter
from launch_ros.actions import Node


def generate_launch_description():

    config_path = os.path.join(
        get_package_share_directory('cobot3'),
        'config',
        'camera_config.yaml'
    )

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)['camera_tf']

    roll  = math.radians(cfg['roll_deg'])
    pitch = math.radians(cfg['pitch_deg'])
    yaw   = math.radians(cfg['yaw_deg'])

    return LaunchDescription([
        SetParameter(name="use_sim_time", value=True),

        # 1. World → camera_frame (Isaac Sim 카메라 로컬 프레임)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=[
                '--x',     str(cfg['x']),
                '--y',     str(cfg['y']),
                '--z',     str(cfg['z']),
                '--roll',  str(roll),
                '--pitch', str(pitch),
                '--yaw',   str(yaw),
                '--frame-id',       cfg['parent_frame'],
                '--child-frame-id', 'camera_frame',
            ]
        ),

        # 2. camera_frame → camera_color_optical_frame
        # Isaac Sim 카메라 로컬: X=오른쪽, Y=위,   Z=뒤(광축=-Z)
        # ROS2 optical frame  : X=오른쪽, Y=아래, Z=앞(광축=+Z)
        # 변환: Rotate X 180도 (Y반전 + Z반전)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_optical_tf',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll',  str(math.pi),  # 180도
                '--pitch', '0',
                '--yaw',   '0',
                '--frame-id',       'camera_frame',
                '--child-frame-id', 'camera_color_optical_frame',
            ]
        ),

    ])