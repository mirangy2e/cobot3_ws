"""[L6 연습 정답] 로봇이 큐브를 바라보게 하기"""

from pathlib import Path

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction

ROBOT_USD = Path(__file__).resolve().parents[1] / "robot" / "m0609_rg2_d455.usd"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
CUBE_POSITION = np.array([0.45, 0.2, 0.026])  # 큐브 처음 위치 (m)
CUBE_SIZE = 0.0515                             # 큐브 한 변 (m)


# ------------------------------------------------------------
# (가) Task: 로봇과 큐브가 놓인 환경 (장면 + 시작 자세 + 관측값)
# ------------------------------------------------------------
class CubeEnvTask(BaseTask):
    """장면(바닥 + M0609 + 큐브)을 만들고, 시작 자세를 맞추고, 관측값을 돌려준다"""

    # 언제: 첫 world.reset() 때 한 번
    # 할 일: 장면에 물체 추가
    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        scene.add_default_ground_plane()
        add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path="/World/m0609_rg2_d455")
        self._robot = scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))
        self._task_objects[self._robot.name] = self._robot
        self._cube = scene.add(  # TODO 1
            DynamicCuboid(prim_path="/World/Cube", name="cube", position=CUBE_POSITION,
                          scale=np.array([CUBE_SIZE] * 3), size=1.0, color=np.array([0, 0, 1.0]))
        )
        self._task_objects[self._cube.name] = self._cube

    # 언제: 매 world.reset() 끝에 (Stop -> Play 때도)
    # 할 일: 시작 자세 맞추기 (홈 자세로 순간이동)
    def post_reset(self):
        home = np.zeros(self._robot.num_dof)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle
        self._robot.set_joint_positions(home)

    # 언제: world.get_observations() 를 부를 때
    # 돌려줄 값: {물체 이름: {값 이름: 값}}
    def get_observations(self):
        return {  # TODO 2
            self._robot.name: {"joint_positions": self._robot.get_joint_positions()},
            self._cube.name: {"position": self._cube.get_world_pose()[0]},
        }

    # 언제: 메인 루프가 물체 이름을 알아낼 때 (task.get_params())
    # 돌려줄 값: {파라미터 이름: {"value": 값}}
    def get_params(self):
        return {"robot_name": {"value": self._robot.name}, "cube_name": {"value": self._cube.name}}  # TODO 2


# ------------------------------------------------------------
# (나) Controller: 관측값(큐브 위치)으로 목표를 계산한다
# ------------------------------------------------------------
class FaceCubeController(BaseController):
    """joint_1 을 돌려 로봇이 큐브 쪽을 바라보게 한다"""

    def __init__(self, name, arm_indices):
        super().__init__(name)
        self._arm_indices = arm_indices  # 명령을 보낼 관절이 관절 배열의 몇 번째인지 (팔 6개)

    # 언제: 매 프레임 메인 루프가 부른다
    # 받는 값: 큐브 위치 [x, y, z] (m) / 돌려줄 값: ArticulationAction
    def forward(self, cube_position):
        # TODO 3
        # 1) 목표 각도: 로봇(원점)에서 본 큐브 방향
        facing = np.arctan2(cube_position[1], cube_position[0])
        # 2) 목표 자세: 홈 자세에서 joint_1 만 facing
        goal_arm = np.radians(HOME_DEG)
        goal_arm[0] = facing
        # 3) 명령 (팔 관절 6개에만)
        return ArticulationAction(joint_positions=goal_arm, joint_indices=self._arm_indices)


# ------------------------------------------------------------
# (다) 등록과 준비
# ------------------------------------------------------------
world = World(stage_units_in_meters=1.0)
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
task = CubeEnvTask(name="cube_env")
world.add_task(task)
world.reset()  # 여기서 task.set_up_scene() -> task.post_reset() 이 불린다

# 물체 이름은 Task 에게 물어본다 (TODO 2 의 get_params)
params = task.get_params()
robot_name = params["robot_name"]["value"]  # "m0609"
cube_name = params["cube_name"]["value"]    # "cube": 관측값에서 큐브 위치를 꺼낼 때 쓴다
robot = world.scene.get_object(robot_name)  # 이름으로 장면에서 로봇 객체를 찾는다 (apply_action 용)
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]  # 팔 관절 6개가 관절 배열(12개)의 몇 번째인지. 예) [0, 1, 2, 3, 4, 5]
controller = FaceCubeController("face_cube_controller", arm_indices=arm)  # 명령을 팔 관절 6개에만 보낸다

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1

    # 1) Stop 을 감지해 둔다
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        # 2) Stop 다음 첫 Play 프레임: 장면과 컨트롤러를 함께 처음으로
        if reset_needed:
            world.reset()
            controller.reset()
            reset_needed = False
        # 3) 관측 (Task)
        observations = world.get_observations()
        cube_position = observations[cube_name]["position"]
        # 4) 계산 (Controller)
        action = controller.forward(cube_position)
        # 5) 명령
        robot.apply_action(action)

        if frame % 60 == 0:
            joint_1_deg = np.degrees(observations[robot_name]["joint_positions"][arm[0]])
            print(f"{frame} 프레임 | 큐브 x {cube_position[0]:5.2f} y {cube_position[1]:5.2f} m | joint_1 {joint_1_deg:6.1f} deg")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
