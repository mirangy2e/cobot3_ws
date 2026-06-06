# ~/cobot3_ws/src/cobot3/launch/demo_sim.launch.py
"""
panda_moveit_config demo.launch.py 래퍼
use_sim_time:=true 를 모든 노드에 일괄 적용

실행:
  ros2 launch cobot3 demo_sim.launch.py
"""

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch_ros.actions import SetParameter


def generate_launch_description():

    set_sim_time = SetParameter(name='use_sim_time', value=True)

    moveit_config = (
        MoveItConfigsBuilder("panda", package_name="panda_moveit_config")
        .to_moveit_configs()
    )

    demo_launch = generate_demo_launch(moveit_config)
    demo_launch.entities.insert(0, set_sim_time)

    return demo_launch
