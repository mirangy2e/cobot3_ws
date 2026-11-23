"""[L9 미션 완성 코드] 타겟 따라가기 컨트롤러: 끌어 옮기고 돌리는 타겟을 그리퍼가 IK 로 따라간다

실행: isaac_python L9_follow_target.py  (창이 뜨면 Play 후 빨간 타겟을 끌고 돌려 본다)
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
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

# 파일 경로
M0609_DIR = Path(__file__).resolve().parents[1]                            # isaacpjt/M0609 (이 파일이 있는 basic 의 상위 폴더)
ROBOT_USD = f"{M0609_DIR}/robot/m0609_rg2_d455.usd"                        # 장면에 넣을 로봇 (M0609 + RG2 + D455)
URDF_PATH = f"{M0609_DIR}/robot/m0609_rg2.urdf"                            # IK 용: 링크 길이, 관절 축, 관절 한계
DESCRIPTION_PATH = f"{M0609_DIR}/descriptor/m0609_robot_description.yaml"  # IK 용: 움직일 관절 (팔 6개)

# 로봇 관절
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
HOME_RAD = np.radians(HOME_DEG)

# 팔 형태 유지
SHAPE_JOINTS = [2, 4]   # 형태를 볼 관절: joint_3 (팔꿈치), joint_5 (손목). 부호가 홈 자세(+90, +90)와 같아야 한다
MAX_JOINT_STEP = 0.02   # 한 프레임에 관절이 움직일 수 있는 최대 각도 (rad). 60 Hz 에서 약 69 deg/s

# IK 목표 설정
TCP_FRAME = "gripper_center"  # IK 로 위치를 맞출 기준점: 그리퍼 손가락 사이 (URDF 의 프레임 이름)
DOWN_ORIENTATION = euler_angles_to_quat(np.radians([0, 180, 0]))  # TCP 자세: 그리퍼가 아래를 향함 (쿼터니언)

# 타겟
TARGET_SIZE = 0.04  # 타겟 한 변 (m)


# ------------------------------------------------------------
# (가) IK 계산기 (L8 과 같다)
# ------------------------------------------------------------
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,          # 로봇 설명 파일
    urdf_path=URDF_PATH,                              # URDF 파일
)


# 타겟 시작 위치: 홈 자세의 TCP 위치 (FK: 관절값 -> TCP 위치)
HOME_TCP_POSITION, _ = ik_solver.compute_forward_kinematics(
    TCP_FRAME,                                        # 위치를 계산할 프레임
    np.radians(HOME_DEG),                             # 홈 자세 관절값 (rad)
)


# ------------------------------------------------------------
# (나) Task: 로봇(팔 + 그리퍼)과 따라갈 타겟이 놓인 환경
# ------------------------------------------------------------
class FollowEnvTask(BaseTask):
    """장면(바닥 + M0609 + RG2 그리퍼 + 빨간 타겟)을 만들고, 시작 자세를 맞추고, 관측값을 돌려준다"""

    # 언제: 첫 world.reset() 때 한 번
    # 할 일: 장면에 물체 추가
    def set_up_scene(self, scene):
        super().set_up_scene(scene)                   # 부모 것을 먼저 (self.scene 을 저장한다)
        scene.add_default_ground_plane()              # 바닥

        # 로봇 USD 참조: 파일의 기본 prim(로봇)을 장면에 넣는다
        add_reference_to_stage(
            usd_path=ROBOT_USD,                       # 참조할 USD 파일 경로
            prim_path="/World/m0609_rg2_d455",        # 장면에 만들 prim 경로 (GUI 의 Add Reference 처럼 파일 이름)
        )

        # 그리퍼 객체: finger_joint 하나로 열고 닫는다
        gripper = ParallelGripper(
            end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",  # 그리퍼 본체 prim (rigid body 여야 함)
            joint_prim_names=["finger_joint"],        # 구동 관절 이름 (RG2 는 하나)
            joint_opened_positions=np.array([0.0]),   # 열림 각도 (rad)
            joint_closed_positions=np.array([1.0]),   # 닫힘 각도 (rad)
            use_mimic_joints=True,                    # 나머지 손가락은 USD 의 mimic 관절이 따라 움직임
        )

        # 로봇 객체: 팔과 그리퍼를 한 객체로
        self._robot = scene.add(
            SingleManipulator(
                prim_path="/World/m0609_rg2_d455",    # 로봇 prim 경로 (위에서 USD 를 참조해 넣은 곳)
                name="m0609",                         # 장면에서 찾을 때 쓰는 이름
                end_effector_prim_path="/World/m0609_rg2_d455/gripper_body",  # 그리퍼 본체 prim (ParallelGripper 와 같게)
                gripper=gripper,                      # 위에서 만든 그리퍼 객체 -> robot.gripper 로 꺼냄
            )
        )

        # 타겟: 눈에만 보이는 상자 (중력·충돌 없음). 뷰포트에서 끌어 옮긴다
        self._target = scene.add(
            VisualCuboid(
                prim_path="/World/Target",            # Stage 에 만들 prim 경로 (뷰포트에서 이것을 선택해 끈다)
                name="target",                        # 장면에서 찾을 때 쓰는 이름
                position=HOME_TCP_POSITION,           # 시작 위치: 홈 자세의 TCP 위치 [x, y, z] (m)
                orientation=DOWN_ORIENTATION,         # 시작 자세: 그리퍼가 아래를 향하는 자세
                scale=np.array([TARGET_SIZE] * 3),    # x, y, z 방향 크기 배율
                size=1.0,                             # 기본 한 변 1 m. 실제 크기 = size x scale = 0.04 m
                color=np.array([1.0, 0, 0]),          # 색 [R, G, B] (0 ~ 1). 빨간색
            )
        )

        # 이 Task 에 속한 물체로 등록
        self._task_objects[self._robot.name] = self._robot
        self._task_objects[self._target.name] = self._target

    # 언제: 매 world.reset() 끝에 (Stop -> Play 때도)
    # 할 일: 시작 자세 맞추기 (팔은 홈 자세, 그리퍼는 열림으로 순간이동). 타겟은 world.reset() 이 시작 위치로 되돌린다
    def post_reset(self):
        home = np.zeros(self._robot.num_dof)          # 관절 12개 값을 담을 배열 (팔 6 + 그리퍼 6)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle  # 관절 이름으로 몇 번째인지 찾아 홈 각도를 넣는다
        self._robot.set_joint_positions(home)         # 팔을 홈 자세로 순간이동
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
            self._target.name: {
                "position": self._target.get_world_pose()[0],         # 타겟 위치 [x, y, z] (m)
                "orientation": self._target.get_world_pose()[1],      # 타겟 자세 (쿼터니언 [w, x, y, z])
            },
        }

    # 언제: 메인 루프가 물체 이름을 알아낼 때 (task.get_params())
    # 돌려줄 값: {파라미터 이름: {"value": 값}}
    def get_params(self):
        return {
            "robot_name": {"value": self._robot.name},    # "m0609"
            "target_name": {"value": self._target.name},  # "target"
        }


# ------------------------------------------------------------
# (다) Controller: 타겟 따라가기 (이 미션에서 만드는 것)
#      매 프레임 타겟 위치·자세로 IK 를 풀어 팔 관절 명령을 만든다
#      홈과 같은 팔 형태의 답만 쓰고, 한 프레임에 조금씩만 움직인다
#      쓸 답이 없으면 마지막 명령을 그대로 보낸다 (그 자리에서 기다림)
# ------------------------------------------------------------
class FollowController(BaseController):
    """타겟 위치·자세 -> IK -> 팔 관절 6개 명령. 단계 없이 매 프레임 같은 일을 한다"""

    def __init__(self, name, arm_indices):
        super().__init__(name)                        # 컨트롤러 이름
        self._arm_indices = arm_indices  # 명령을 보낼 관절이 관절 배열의 몇 번째인지 (팔 6개)
        self.reset()

    # 언제: 처음 만들 때, 그리고 메인 루프가 world.reset() 할 때
    # 할 일: 기억하는 상태를 처음 값으로
    def reset(self):
        self._last_command = None  # 마지막으로 보낸 팔 관절 목표 (rad). 처음엔 없음
        self.reachable = True      # 마지막 프레임에 따라갈 답(홈 형태 IK 해)이 있었는지 (메인 루프가 출력에 쓴다)

    # 언제: 매 프레임 메인 루프가 부른다
    # 받는 값: 타겟 위치 (m), 타겟 자세 (쿼터니언), 팔 관절 6개의 지금 각도 (rad) / 돌려줄 값: ArticulationAction
    def forward(self, target_position, target_orientation, current_arm):
        # 1) reset 후 첫 프레임: 지금 자세를 유지하는 명령으로 시작
        if self._last_command is None:
            self._last_command = current_arm

        # 2) IK 를 두 시작점에서 차례로 풀고, 홈과 같은 팔 형태인 첫 답을 쓴다
        facing_home = HOME_RAD.copy()
        facing_home[0] = np.arctan2(target_position[1], target_position[0])  # 홈 자세에서 joint_1 만 타겟 방향으로
        solution = None
        for seed in (current_arm, facing_home):      # 지금 자세에서 먼저, 안 되면 홈 계열에서
            joints, success = ik_solver.compute_inverse_kinematics(
                TCP_FRAME,                            # 목표에 맞출 프레임
                target_position,                      # 목표 위치 [x, y, z] (m)
                target_orientation,                   # 목표 자세 (쿼터니언 [w, x, y, z])
                warm_start=seed,                      # 계산을 시작할 관절값
            )
            if success and self._same_shape(joints):
                solution = joints
                break

        # 3) 답이 있으면 그쪽으로 한 프레임에 MAX_JOINT_STEP 까지만, 없으면 마지막 목표 그대로 (기다림)
        self.reachable = solution is not None
        if self.reachable:
            step = np.clip(solution - self._last_command, -MAX_JOINT_STEP, MAX_JOINT_STEP)
            self._last_command = self._last_command + step

        # 4) 팔 관절 6개에만 보내는 명령
        return ArticulationAction(
            joint_positions=self._last_command,       # 팔 관절 6개 목표 (rad)
            joint_indices=self._arm_indices,          # 6개 값을 관절 배열의 몇 번째에 넣을지 (그리퍼는 그대로)
        )

    # 홈 자세와 같은 팔 형태인지: joint_3 (팔꿈치), joint_5 (손목) 의 부호가 홈과 같아야 한다
    def _same_shape(self, joints):
        return np.all(np.sign(joints[SHAPE_JOINTS]) == np.sign(HOME_RAD[SHAPE_JOINTS]))


# ------------------------------------------------------------
# (라) 등록과 준비
# ------------------------------------------------------------
world = World(
    stage_units_in_meters=1.0,                        # 길이 단위: 1 = 1 m
    physics_dt=1.0 / 120.0,                           # 물리 계산 주기 (s)
    rendering_dt=1.0 / 60.0,                          # 한 프레임 시간 (s)
)
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute(
    "SetLightingMenuModeCommand",                     # 실행할 명령 이름
    lighting_mode="Default",                          # 조명 모드
)
task = FollowEnvTask(name="follow_env")               # Task 이름
world.add_task(task)                                  # World 에 Task 등록
world.reset()  # 여기서 task.set_up_scene() -> task.post_reset() 이 불린다

params = task.get_params()
robot_name = params["robot_name"]["value"]            # "m0609"
target_name = params["target_name"]["value"]          # "target"
robot = world.scene.get_object(robot_name)            # 이름으로 장면에서 로봇 객체를 찾는다
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]  # 팔 관절 6개가 관절 배열(12개)의 몇 번째인지 -> [0, 1, 2, 3, 4, 5]
controller = FollowController(
    "follow_controller",                              # 컨트롤러 이름
    arm_indices=arm,                                  # 팔 관절 6개가 몇 번째인지
)
print("Play 후 뷰포트에서 빨간 타겟(/World/Target)을 선택해 끌어 보세요")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False
last_reachable = True  # 바로 전 프레임의 IK 성공 여부 (바뀌면 출력)
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
            last_reachable = True
            reset_needed = False
            print("reset 완료: 처음부터 다시 시작")
        # 3) 관측 (Task)
        observations = world.get_observations()
        current_arm = observations[robot_name]["joint_positions"][arm]  # 12개 중 팔 6개만
        target_position = observations[target_name]["position"]
        target_orientation = observations[target_name]["orientation"]
        # 4) 계산 (Controller)
        action = controller.forward(target_position, target_orientation, current_arm)
        # 5) 명령
        robot.apply_action(action)

        if controller.reachable != last_reachable:  # IK 성공 / 실패가 바뀌었을 때 한 번 출력
            x, y, z = target_position
            if controller.reachable:
                print(f"시각 {world.current_time:5.2f} s | 다시 따라감 | 타겟 x {x:5.2f} y {y:5.2f} z {z:.3f} m")
            else:
                print(f"시각 {world.current_time:5.2f} s | 따라갈 IK 해 없음: 마지막 자세 유지 | 타겟 x {x:5.2f} y {y:5.2f} z {z:.3f} m")
            last_reachable = controller.reachable

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
