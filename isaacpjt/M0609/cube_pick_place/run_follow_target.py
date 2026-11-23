"""[실행] 디버깅용: 빨간 타겟을 끌어 로봇을 움직이고, 그리퍼 각도를 바꿔 큐브를 직접 집어본다.

실행: isaac_python run_follow_target.py  (창이 뜨면 Play)
- 뷰포트에서 /World/TargetCube 를 끌면 TCP(gripper_center)가 IK 로 따라간다 (팔 형태 유지, 속도 제한)
- m0609_settings.yaml 의 gripper_angle 을 바꿔 저장하면 손가락이 그 각도로 움직인다 (0.0 열림 ~ 1.10 닫힘, rad)
- 손가락이 멈추면 TCP, 큐브 위치, TCP - 큐브 를 출력한다
  -> m0609_settings.yaml 의 end_effector_offset, gripper_closed 를 정하는 데 사용
"""

import sys

from isaacsim import SimulationApp

# --headless: 창 없이 실행 (채점 스크립트가 사용). SimulationApp 은 다른 Isaac 모듈보다 먼저 만들어야 한다
simulation_app = SimulationApp({"headless": "--headless" in sys.argv})

import numpy as np
import m0609_settings as settings
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from m0609_motion.ik import IKController
from m0609_robot import home_tcp_position
from pick_place_scenes import FollowTargetTask, use_default_light

GRIPPER_ANGLE_RANGE = (0.0, 1.10)  # 입력 허용 범위 (rad). USD 의 finger_joint 한계는 1.18

# 장면: 바닥 + 빨간 타겟 + 큐브 + 로봇. 타겟은 홈 자세의 TCP 에서 시작해 reset 후에도 로봇과 위치가 맞음
my_world = World(stage_units_in_meters=1.0, physics_dt=settings.PHYSICS_DT, rendering_dt=settings.RENDERING_DT)
my_task = FollowTargetTask(target_position=home_tcp_position())
my_world.add_task(my_task)
my_world.reset()
use_default_light()  # 뷰포트 조명: Lighting > Default
task_params = my_task.get_params()
target_name = task_params["target_name"]["value"]
cube_name = task_params["cube_name"]["value"]
my_robot = my_world.scene.get_object(task_params["robot_name"]["value"])
finger_index = my_robot.get_dof_index(settings.GRIPPER_JOINT)

my_controller = IKController(name="follow_target", robot_articulation=my_robot)
articulation_controller = my_robot.get_articulation_controller()
print(f"/World/TargetCube 를 끌고, {settings.SETTINGS_PATH.name} 의 gripper_angle (0.0 ~ 1.10 rad) 을 바꿔 저장하세요")


def command_gripper():
    """gripper_angle 을 허용 범위로 자른 뒤 손가락 목표로 보낸다."""
    angle = float(np.clip(settings.GRIPPER_ANGLE, *GRIPPER_ANGLE_RANGE))
    if angle != settings.GRIPPER_ANGLE:
        print(f"gripper_angle {settings.GRIPPER_ANGLE} 이 범위 {GRIPPER_ANGLE_RANGE} 밖이라 {angle} 을 씁니다")
    my_robot.gripper.apply_action(ArticulationAction(joint_positions=[angle]))
    print(f"gripper_angle -> {angle:.3f} rad ({np.degrees(angle):.1f} deg)")


reset_needed = False
command_needed = True  # 시작과 reset 직후에는 현재 gripper_angle 을 한 번 보냄
last_finger = None
gripper_moving = False
while simulation_app.is_running():
    my_world.step(render=True)
    # Stop 하면 물리 데이터가 무효화되므로, 다음 Play 의 첫 프레임에서 먼저 reset
    if my_world.is_stopped() and not reset_needed:
        reset_needed = True
    if my_world.is_playing():
        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed = False
            command_needed = True
            last_finger = None

        # m0609_settings.yaml 이 저장돼 gripper_angle 이 바뀌었으면 그리퍼에 보냄
        if "GRIPPER_ANGLE" in settings.reload() or command_needed:
            command_gripper()
            command_needed = False
            gripper_moving = True

        # 타겟을 IK 로 따라감
        observations = my_world.get_observations()
        actions = my_controller.forward(
            target_end_effector_position=observations[target_name]["position"],
            target_end_effector_orientation=observations[target_name]["orientation"],
        )
        articulation_controller.apply_action(actions)

        # 손가락이 멈추면 위치 정보 출력
        finger = my_robot.get_joint_positions()[finger_index]
        if gripper_moving and last_finger is not None and abs(finger - last_finger) < 1e-4:
            tcp, _ = my_controller.compute_end_effector_pose()
            cube = observations[cube_name]["position"]
            print(
                f"finger_joint 멈춤 {finger:.3f} rad ({np.degrees(finger):.1f} deg) | "
                f"TCP {np.round(tcp, 3)} | cube {np.round(cube, 3)} | TCP - cube {np.round(tcp - cube, 3)}"
            )
            gripper_moving = False
        last_finger = finger

simulation_app.close()
