"""[L7 예제] 그리퍼와 pick and place: 큐브 집어 들기

실행: isaac_python L7_gripper_steps.py
"""

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
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

ROBOT_USD = Path(__file__).resolve().parents[1] / "robot" / "m0609_rg2_d455.usd"
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
ABOVE_DEG = [0, 12, 83, 0, 85, 0]   # 큐브 위 자세: 그리퍼가 아래를 보고, 그리퍼 끝이 바닥에서 약 0.2 m
PICK_DEG = [0, 18, 101, 0, 61, 0]   # 큐브를 집는 자세: 그리퍼 끝이 큐브 높이
CUBE_POSITION = np.array([0.45, 0.0, 0.02575])  # 로봇 정면 (m)
CUBE_SIZE = 0.0515                               # 큐브 한 변 (m)
TOLERANCE_DEG = 1.0  # 팔 관절이 모두 목표와 이만큼 안으로 들어오면 "도착"
GRIPPER_FRAMES = 60  # 그리퍼 단계는 이만큼 기다린 뒤 다음 단계로 (60 프레임 = 1 초)

# 단계 목록: (이름, 종류, 값)
#   종류 "arm"     : 값 = 팔 관절 목표 (deg). 도착하면 다음 단계
#   종류 "gripper" : 값 = "open" 또는 "close". GRIPPER_FRAMES 가 지나면 다음 단계
STEPS = [
    ("큐브 위로", "arm", ABOVE_DEG),
    ("내려가기", "arm", PICK_DEG),
    ("그리퍼 닫기", "gripper", "close"),
    ("들어 올리기", "arm", ABOVE_DEG),
]


# ------------------------------------------------------------
# (가) Task: 로봇(팔 + 그리퍼)과 큐브가 놓인 환경
# ------------------------------------------------------------
class PickEnvTask(BaseTask):
    """장면(바닥 + M0609 + RG2 그리퍼 + 큐브)을 만들고, 시작 자세를 맞추고, 관측값을 돌려준다"""

    # 언제: 첫 world.reset() 때 한 번
    # 할 일: 장면에 물체 추가
    def set_up_scene(self, scene):
        super().set_up_scene(scene)                   # 부모 것을 먼저 (self.scene 을 저장한다)
        scene.add_default_ground_plane()              # 바닥

        # 로봇 USD 참조: 파일의 기본 prim(로봇)을 장면에 넣는다
        add_reference_to_stage(
            usd_path=str(ROBOT_USD),                  # 참조할 USD 파일 경로
            prim_path="/World/m0609_rg2_d455",        # 장면에 만들 prim 경로 (GUI 의 Add Reference 처럼 파일 이름)
        )

        # 그리퍼 객체: finger_joint 하나로 열고 닫는다
        gripper = ParallelGripper(
            end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",  # 그리퍼 본체 prim (rigid body 여야 함)
            joint_prim_names=["finger_joint"],        # 구동 관절 이름 (RG2 는 하나)
            joint_opened_positions=np.array([0.0]),   # 열림 각도 (rad)
            joint_closed_positions=np.array([1.0]),   # 닫힘 각도 (rad). 큐브를 잡으면 그 전에 멈춘다
            use_mimic_joints=True,                    # 나머지 손가락은 USD 의 mimic 관절이 따라 움직임
        )

        # 로봇 객체: 팔과 그리퍼를 한 객체로 (SingleArticulation 에 gripper 가 붙은 것)
        self._robot = scene.add(
            SingleManipulator(
                prim_path="/World/m0609_rg2_d455",    # 로봇 prim 경로 (위에서 USD 를 참조해 넣은 곳)
                name="m0609",                         # 장면에서 찾을 때 쓰는 이름 (world.scene.get_object)
                end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",  # 그리퍼 본체 prim (ParallelGripper 와 같게)
                gripper=gripper,                      # 위에서 만든 그리퍼 객체 -> robot.gripper 로 꺼냄
            )
        )

        # 큐브: 중력과 충돌이 있는 상자 (그리퍼로 집을 물체)
        self._cube = scene.add(
            DynamicCuboid(
                prim_path="/World/Cube",              # Stage 에 만들 prim 경로
                name="cube",                          # 장면에서 찾을 때 쓰는 이름
                position=CUBE_POSITION,               # 큐브 중심 위치 [x, y, z] (m)
                scale=np.array([CUBE_SIZE] * 3),      # x, y, z 방향 크기 배율 [0.0515, 0.0515, 0.0515]
                size=1.0,                             # 기본 한 변 1 m. 실제 크기 = size x scale = 0.0515 m
                color=np.array([0, 0, 1.0]),          # 색 [R, G, B] (0 ~ 1). 파란색
            )
        )

        # 이 Task 에 속한 물체로 등록
        self._task_objects[self._robot.name] = self._robot
        self._task_objects[self._cube.name] = self._cube

    # 언제: 매 world.reset() 끝에 (Stop -> Play 때도)
    # 할 일: 시작 자세 맞추기 (팔은 홈 자세, 그리퍼는 열림으로 순간이동)
    def post_reset(self):
        home = np.zeros(self._robot.num_dof)          # 관절 12개 값을 담을 배열 (팔 6 + 그리퍼 6)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle  # 관절 이름으로 몇 번째인지 찾아 홈 각도를 넣는다
        self._robot.set_joint_positions(home)         # 팔을 홈 자세로 순간이동

        # 그리퍼 열기: 그리퍼 객체가 finger_joint 를 열림 각도로 순간이동시킨다
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions,  # 그리퍼를 만들 때 넣은 열림 각도 [0.0]
        )

    # 언제: world.get_observations() 를 부를 때
    # 돌려줄 값: {물체 이름: {값 이름: 값}}
    def get_observations(self):
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),  # 관절 12개의 지금 각도 (rad)
            },
            self._cube.name: {
                "position": self._cube.get_world_pose()[0],           # 큐브 중심 위치 [x, y, z] (m)
            },
        }

    # 언제: 메인 루프가 물체 이름을 알아낼 때 (task.get_params())
    # 돌려줄 값: {파라미터 이름: {"value": 값}}
    def get_params(self):
        return {
            "robot_name": {"value": self._robot.name},  # "m0609"
            "cube_name": {"value": self._cube.name},    # "cube"
        }


# ------------------------------------------------------------
# (나) Controller: 단계 목록(STEPS)을 하나씩 진행한다 (상태 머신)
#      지금 단계 번호를 기억하고, 단계가 끝나면 다음 번호로 넘어간다
# ------------------------------------------------------------
class StepController(BaseController):
    """STEPS 의 단계를 차례로 진행한다. 팔 단계는 관절 목표를, 그리퍼 단계는 그리퍼 명령을 돌려준다"""

    def __init__(self, name, gripper, arm_indices):
        super().__init__(name)                        # 컨트롤러 이름
        self._gripper = gripper          # 그리퍼 객체 (forward("open" / "close") 로 명령을 만들어 준다)
        self._arm_indices = arm_indices  # 명령을 보낼 관절이 관절 배열의 몇 번째인지 (팔 6개)
        self.reset()

    # 언제: 처음 만들 때, 그리고 메인 루프가 world.reset() 할 때
    # 할 일: 기억하는 상태를 처음 값으로
    def reset(self):
        self.step = 0          # 지금 단계 번호 (STEPS 의 몇 번째)
        self._step_frames = 0  # 지금 단계에서 지난 프레임 수

    # 모든 단계를 끝냈는지
    def is_done(self):
        return self.step >= len(STEPS)

    # 언제: 매 프레임 메인 루프가 부른다
    # 받는 값: 팔 관절 6개의 지금 각도 (rad) / 돌려줄 값: ArticulationAction (다 끝났으면 None)
    def forward(self, current_arm):
        if self.is_done():
            return None
        name, kind, value = STEPS[self.step]          # 지금 단계의 (이름, 종류, 값)
        self._step_frames += 1

        # 1) 이번 단계의 명령과 "끝났는지"
        if kind == "arm":
            action = ArticulationAction(
                joint_positions=np.radians(value),    # 팔 관절 6개 목표 (deg -> rad)
                joint_indices=self._arm_indices,      # 6개 값을 관절 배열의 몇 번째에 넣을지 (그리퍼는 그대로)
            )
            finished = np.max(np.abs(np.degrees(current_arm) - value)) < TOLERANCE_DEG  # 6개 모두 도착
        else:
            # 그리퍼 객체가 finger_joint 자리에만 열림/닫힘 각도를 넣은 명령을 만들어 준다
            action = self._gripper.forward(
                action=value,                         # "open" 또는 "close"
            )
            finished = self._step_frames >= GRIPPER_FRAMES
        # 2) 끝났으면 다음 단계로
        if finished:
            self.step += 1
            self._step_frames = 0
        return action


# ------------------------------------------------------------
# (다) 등록과 준비
# ------------------------------------------------------------
world = World(
    stage_units_in_meters=1.0,                        # 길이 단위: 1 = 1 m
    physics_dt=1.0 / 120.0,                           # 물리 계산 주기 (s). 한 프레임에 물리 계산 2 번 (L2 와 같음)
    rendering_dt=1.0 / 60.0,                          # 한 프레임 시간 (s)
)
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute(
    "SetLightingMenuModeCommand",                     # 실행할 명령 이름
    lighting_mode="Default",                          # 조명 모드
)
task = PickEnvTask(name="pick_env")                   # Task 이름
world.add_task(task)                                  # World 에 Task 등록
world.reset()  # 여기서 task.set_up_scene() -> task.post_reset() 이 불린다

params = task.get_params()
robot_name = params["robot_name"]["value"]            # "m0609"
cube_name = params["cube_name"]["value"]              # "cube"
robot = world.scene.get_object(robot_name)            # 이름으로 장면에서 로봇 객체를 찾는다
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]  # 팔 관절 6개가 관절 배열(12개)의 몇 번째인지
controller = StepController(
    "step_controller",                                # 컨트롤러 이름
    gripper=robot.gripper,                            # 로봇에 붙은 그리퍼 객체
    arm_indices=arm,                                  # 팔 관절 6개가 몇 번째인지
)

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False
last_step = 0  # 바로 전 프레임의 단계 번호 (단계가 바뀌면 출력)
while simulation_app.is_running():
    world.step(render=True)                           # 한 프레임 진행 (render=True: 화면도 그린다)

    # 1) Stop 을 감지해 둔다
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if world.is_playing():
        # 2) Stop 다음 첫 Play 프레임: 장면과 컨트롤러를 함께 처음으로
        if reset_needed:
            world.reset()
            controller.reset()
            last_step = 0
            reset_needed = False
            print("reset 완료: 처음부터 다시 시작")
        # 3) 관측 (Task)
        observations = world.get_observations()
        current_arm = observations[robot_name]["joint_positions"][arm]  # 12개 중 팔 6개만
        # 4) 계산 (Controller)
        action = controller.forward(current_arm)
        # 5) 명령 (다 끝났으면 None: 마지막 목표 그대로 유지)
        if action is not None:
            robot.apply_action(action)

        if controller.step != last_step:  # 단계가 끝났을 때 한 번 출력
            finger = observations[robot_name]["joint_positions"][robot.get_dof_index("finger_joint")]  # 그리퍼 각도 (rad)
            x, y, z = observations[cube_name]["position"]                                          # 큐브 위치 (m)
            print(f"시각 {world.current_time:5.2f} s | {STEPS[last_step][0]} 끝 | 그리퍼 {finger:.2f} rad | 큐브 x {x:5.2f} y {y:5.2f} z {z:.3f} m")
            last_step = controller.step
            if controller.is_done():
                print("모든 단계 완료")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
