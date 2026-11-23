"""[실행] M0609 큐브 pick and place (ik + linear 이동). Isaac Sim UR10e pick_up_example.py 를 옮긴 것.

실행: isaac_python run_pick_place.py  (창이 뜨면 Play)
- 큐브를 cube_initial_position 에서 집어 place_position 에 놓고 시작 위치로 돌아온다
- 단계별 이동 방식은 m0609_settings.yaml 의 motion_modes
- m0609_settings.yaml 의 [실행 중 반영 영역] 값은 저장하면 바로 반영된다
- Stop 후 Play 하면 처음부터 다시 시작한다
"""

import sys

from isaacsim import SimulationApp

# --headless: 창 없이 실행 (채점 스크립트가 사용). SimulationApp 은 다른 Isaac 모듈보다 먼저 만들어야 한다
simulation_app = SimulationApp({"headless": "--headless" in sys.argv})

import argparse

import m0609_settings as settings
from isaacsim.core.api import World
from pick_place_controller import PickPlaceController
from pick_place_scenes import PickPlaceTask, use_default_light

parser = argparse.ArgumentParser()
parser.add_argument("--test", default=False, action="store_true", help="완료되면 종료")
args, unknown = parser.parse_known_args()

# 장면: 바닥 + 큐브 + 로봇
my_world = World(stage_units_in_meters=1.0, physics_dt=settings.PHYSICS_DT, rendering_dt=settings.RENDERING_DT)
my_task = PickPlaceTask()
my_world.add_task(my_task)
my_world.reset()
use_default_light()  # 뷰포트 조명: Lighting > Default
task_params = my_task.get_params()
cube_name = task_params["cube_name"]["value"]
robot_name = task_params["robot_name"]["value"]
my_robot = my_world.scene.get_object(robot_name)

my_controller = PickPlaceController(name="pick_place", robot_articulation=my_robot, gripper=my_robot.gripper)
articulation_controller = my_robot.get_articulation_controller()

reset_needed = False
task_completed = False
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
            task_completed = False

        # m0609_settings.yaml 이 저장됐으면 다시 읽음 (놓을 위치는 task 에도 알려줌)
        if "PLACE_POSITION" in settings.reload():
            my_task.set_params(target_position=settings.PLACE_POSITION)

        observations = my_world.get_observations()
        actions = my_controller.forward(
            picking_position=observations[cube_name]["position"],
            placing_position=observations[cube_name]["target_position"],
            current_joint_positions=observations[robot_name]["joint_positions"],
            end_effector_offset=settings.END_EFFECTOR_OFFSET,
        )
        if my_controller.is_done() and not task_completed:
            print("done picking and placing")
            print(f"최종 큐브 위치 {observations[cube_name]['position'].round(3).tolist()} / 목표 {settings.PLACE_POSITION.round(3).tolist()}")
            task_completed = True
            if args.test:
                break
        articulation_controller.apply_action(actions)

simulation_app.close()
