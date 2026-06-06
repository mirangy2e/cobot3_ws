# ~/cobot3_ws/src/cobot3/launch/camera_static_tf.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 카메라 TF
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='camera_static_tf',
            arguments=[
                '0.0', '-0.33', '0.46',
                '0.6981', '0.0', '0.0',
                'World', 'camera_color_optical_frame'
            ]
        ),
        # World ↔ world 브릿지 (MoveIt2 planning frame 연결)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='world_bridge_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '0',
                'World', 'world'
            ]
        ),
    ])