# ~/cobot3_ws/src/cobot3/launch/camera_static_tf.launch.py
"""
카메라 TF 발행 launch 파일
카메라 위치 변경 시 config/camera_config.yaml 만 수정

실행:
  ros2 launch cobot3 camera_static_tf.launch.py
"""

import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # ── yaml 파일 로드 ──────────────────────────────────────
    config_path = os.path.join(
        get_package_share_directory('cobot3'),
        'config',
        'camera_config.yaml'
    )

    with open(config_path, 'r') as f:
        cfg = yaml.safe_load(f)['camera_tf']

    # 도 → 라디안 변환
    roll  = math.radians(cfg['roll_deg'])
    pitch = math.radians(cfg['pitch_deg'])
    yaw   = math.radians(cfg['yaw_deg'])

    return LaunchDescription([

        # 카메라 TF (parent_frame → child_frame)
        # 새로운 방식: --x --y --z --roll --pitch --yaw
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
                '--child-frame-id', cfg['child_frame'],
            ]
        ),

        # # World ↔ world 브릿지 (MoveIt2 planning frame 연결)
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='world_bridge_tf',
        #     arguments=[
        #         '--x', '0', '--y', '0', '--z', '0',
        #         '--roll', '0', '--pitch', '0', '--yaw', '0',
        #         '--frame-id', 'World',
        #         '--child-frame-id', 'world',
        #     ]
        # ),

    ])