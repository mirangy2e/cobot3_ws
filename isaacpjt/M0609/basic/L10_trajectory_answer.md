---
notion_parent: https://app.notion.com/p/27b450ef7c5983939fa701761ca13d7d
notion_page: https://app.notion.com/p/407450ef7c59805ca1f9e8fc8198e0c5
---

# L10. 연습 문제 정답

---

## 정답 코드
### L10_trajectory_answer.py
```python
"""[L10 연습 정답] 직선으로 내려가고 올라가는 pick and place: 하강·상승 단계를 trajectory 로 움직이기"""

from pathlib import Path

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import lula
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.rotations import euler_angles_to_quat, rot_matrix_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, LulaTaskSpaceTrajectoryGenerator
from isaacsim.robot_motion.motion_generation.lula.utils import get_pose3
from isaacsim.util.debug_draw import _debug_draw

# 파일 경로
M0609_DIR = Path(__file__).resolve().parents[1]                            # isaacpjt/M0609 (이 파일이 있는 basic 의 상위 폴더)
ROBOT_USD = f"{M0609_DIR}/robot/m0609_rg2_d455.usd"                        # 장면에 넣을 로봇 (M0609 + RG2 + D455)
URDF_PATH = f"{M0609_DIR}/robot/m0609_rg2.urdf"                            # IK 용: 링크 길이, 관절 축, 관절 한계
DESCRIPTION_PATH = f"{M0609_DIR}/descriptor/m0609_robot_description.yaml"  # IK 용: 움직일 관절 (팔 6개)

# 로봇 관절
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]

# IK, trajectory 목표 설정
TCP_FRAME = "gripper_center"  # IK 로 위치를 맞출 기준점: 그리퍼 손가락 사이 (URDF 의 프레임 이름)
DOWN_ORIENTATION = euler_angles_to_quat(np.radians([0, 180, 0]))  # TCP 자세: 그리퍼가 아래를 향함 (쿼터니언)

# 큐브와 놓을 위치
CUBE_POSITION = np.array([0.45, 0.20, 0.02575])  # 로봇 왼쪽 앞 (m). 미션의 큐브 처음 위치
CUBE_SIZE = 0.0515                                # 큐브 한 변 (m)
ABOVE_HEIGHT = 0.20                               # 큐브 위 자세의 TCP 높이 (m)
GRASP_HEIGHT = CUBE_POSITION[2] + 0.015           # 집을 때 TCP 높이: 큐브 중심보다 1.5 cm 위 (손가락 끝이 바닥에 걸리지 않게)
PLACE_POSITION = np.array([0.45, -0.25, 0.02575])  # 큐브를 놓을 위치 (m). 미션의 놓을 위치

# 단계별 TCP 목표 위치 [x, y, z] (m)
ABOVE_POS = np.array([CUBE_POSITION[0], CUBE_POSITION[1], ABOVE_HEIGHT])          # 큐브 위
PICK_POS = np.array([CUBE_POSITION[0], CUBE_POSITION[1], GRASP_HEIGHT])           # 큐브를 집는 위치
PLACE_ABOVE_POS = np.array([PLACE_POSITION[0], PLACE_POSITION[1], ABOVE_HEIGHT])  # 놓을 위치 위
PLACE_POS = np.array([PLACE_POSITION[0], PLACE_POSITION[1], GRASP_HEIGHT])        # 내려놓는 위치

# 단계 진행
TOLERANCE_DEG = 1.0  # 팔 관절이 모두 목표와 이만큼 안으로 들어오면 "도착"
GRIPPER_FRAMES = 60  # 그리퍼 단계는 이만큼 기다린 뒤 다음 단계로 (60 프레임 = 1 초)
SPEED_SCALE = 0.3    # 직선 단계: 관절 속도·가속도 한계에 곱하는 비율
DT = 1.0 / 60.0      # 직선 단계: trajectory 에서 관절값을 꺼내는 간격 = 한 프레임 시간 (s)

# 그리기 (색은 R, G, B, 투명도 / 크기는 화면 픽셀)
PLAN_COLOR = (0.0, 0.8, 0.0, 1.0)    # 직선 단계의 계획 경로: 녹색 선
PLAN_WIDTH = 3
ACTUAL_COLOR = (1.0, 0.0, 0.0, 1.0)  # 실제 TCP: 빨간 점
ACTUAL_SIZE = 10
DRAW_EVERY = 3                       # 실제 TCP 를 3 프레임마다 점 하나로 그린다


# ------------------------------------------------------------
# (가) IK: TCP 목표(위치 + 자세) -> 팔 관절 6개 각도 (L8 과 같음)
# ------------------------------------------------------------
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,          # 로봇 설명 파일
    urdf_path=URDF_PATH,                              # URDF 파일
)


def solve_ik(position):
    """TCP 가 position 에서 아래를 보게 하는 팔 관절 6개 각도 (deg) 를 돌려준다"""
    joints, success = ik_solver.compute_inverse_kinematics(
        TCP_FRAME,                                    # 목표에 맞출 프레임
        position,                                     # 목표 위치 [x, y, z] (m)
        DOWN_ORIENTATION,                             # 목표 자세 (쿼터니언 [w, x, y, z])
        warm_start=np.radians(HOME_DEG),              # 계산을 시작할 관절값 (rad). 여기서 가까운 답을 찾는다
    )
    if not success:
        raise RuntimeError(f"IK 해를 찾지 못했다: {position}")
    return np.degrees(joints)                         # rad -> deg (ARM_JOINTS 순서)


# 직선으로 가지 않는 단계의 자세 2개 (하강·상승은 직선 단계가 trajectory 로 움직인다)
ABOVE_DEG = solve_ik(ABOVE_POS)              # 큐브 위 자세
PLACE_ABOVE_DEG = solve_ik(PLACE_ABOVE_POS)  # 놓을 위치 위 자세


# ------------------------------------------------------------
# (가-2) trajectory: 지금 관절값에서 TCP 목표까지 직선 -> 시간별 관절값 목록 (TODO 1)
# ------------------------------------------------------------
generator = LulaTaskSpaceTrajectoryGenerator(
    robot_description_path=DESCRIPTION_PATH,          # 로봇 설명 파일
    urdf_path=URDF_PATH,                              # URDF 파일
)
# 속도 조절: 원래 한계값(URDF) 에 SPEED_SCALE 을 곱해 다시 넣는다
generator.set_c_space_velocity_limits(np.array(generator.get_c_space_velocity_limits()) * SPEED_SCALE)
generator.set_c_space_acceleration_limits(np.array(generator.get_c_space_acceleration_limits()) * SPEED_SCALE)


def plan_line(start_joints, goal_position):
    """start_joints (rad) 의 TCP 에서 goal_position 까지 직선으로 가는 (관절값 목록, 관절 속도 목록) 을 돌려준다"""
    # ① 시작 TCP 위치·자세 (FK)
    start_pos, start_rot = ik_solver.compute_forward_kinematics(TCP_FRAME, start_joints)
    # ② 직선 경로: 시작 TCP -> 목표
    line = lula.create_task_space_path_spec(get_pose3(start_pos, rot_quat=rot_matrix_to_quat(start_rot)))
    line.add_linear_path(get_pose3(goal_position, rot_quat=DOWN_ORIENTATION))
    # ③ 묶음 경로: 시작 관절값을 정하고 ② 를 넣는다
    path = lula.create_composite_path_spec(start_joints.astype(np.float64))
    path.add_task_space_path_spec(line, lula.CompositePathSpec.TransitionMode.SKIP)
    # ④ trajectory -> DT 간격 관절값, 관절 속도 목록
    trajectory = generator.compute_task_space_trajectory_from_path_spec(path, TCP_FRAME)
    if trajectory is None:
        raise RuntimeError(f"직선 trajectory 를 만들지 못했다: {goal_position}")
    joint_list, velocity_list = [], []
    times = np.append(np.arange(trajectory.start_time, trajectory.end_time, DT), trajectory.end_time)  # 마지막은 끝 시각 (속도 0)
    for t in times:
        joints, velocities = trajectory.get_joint_targets(t)
        joint_list.append(joints)
        velocity_list.append(velocities)
    return joint_list, velocity_list


# 단계 목록: (이름, 종류, 값)
#   종류 "arm"     : 값 = 팔 관절 목표 (deg). 도착하면 다음 단계
#   종류 "line"    : 값 = TCP 목표 위치 [x, y, z] (m). 단계 시작 때 직선 trajectory 를 만들고, 다 보내고 도착하면 다음 단계
#   종류 "gripper" : 값 = "open" 또는 "close". GRIPPER_FRAMES 가 지나면 다음 단계
STEPS = [
    ("큐브 위로", "arm", ABOVE_DEG),
    ("내려가기", "line", PICK_POS),
    ("그리퍼 닫기", "gripper", "close"),
    ("들어 올리기", "line", ABOVE_POS),
    ("옆으로 옮기기", "arm", PLACE_ABOVE_DEG),
    ("내려놓기", "line", PLACE_POS),
    ("그리퍼 열기", "gripper", "open"),
    ("올라가기", "line", PLACE_ABOVE_POS),
]


# ------------------------------------------------------------
# (나) Task: 로봇(팔 + 그리퍼)과 큐브가 놓인 환경
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
            usd_path=ROBOT_USD,                       # 참조할 USD 파일 경로
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
# (다) Controller: 단계 목록(STEPS)을 하나씩 진행한다 (상태 머신)
#      지금 단계 번호를 기억하고, 단계가 끝나면 다음 번호로 넘어간다
# ------------------------------------------------------------
class StepController(BaseController):
    """STEPS 의 단계를 차례로 진행한다. 팔 단계는 관절 목표를, 직선 단계는 trajectory 관절값을, 그리퍼 단계는 그리퍼 명령을 돌려준다"""

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
        self._joint_list = []     # 직선 단계: 보낼 관절값 목록 (rad)
        self._velocity_list = []  # 직선 단계: 보낼 관절 속도 목록 (rad/s)

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
        elif kind == "line":
            # (TODO 2) 단계 첫 프레임: 지금 관절값에서 목표까지 직선 trajectory 를 만든다
            if self._step_frames == 1:
                self._joint_list, self._velocity_list = plan_line(current_arm, value)
                print(f"직선 | {name} | 관절값 {len(self._joint_list)} 개 ({len(self._joint_list) * DT:.2f} s)")
            # 한 프레임에 하나씩 보낸다. 다 보낸 뒤에는 마지막 것(멈춘 자세, 속도 0)을 계속 보낸다
            last = len(self._joint_list) - 1                # 마지막 번호
            index = min(self._step_frames - 1, last)        # 이번 프레임에 보낼 번호
            action = ArticulationAction(
                joint_positions=self._joint_list[index],      # 이 프레임의 관절값 (rad)
                joint_velocities=self._velocity_list[index],  # 이 프레임의 관절 속도 (rad/s)
                joint_indices=self._arm_indices,              # 팔 관절 6개 자리에 넣는다
            )
            arrived = np.max(np.abs(np.degrees(current_arm - self._joint_list[last]))) < TOLERANCE_DEG
            finished = index == last and arrived            # 다 보냈고 마지막 관절값에 도착
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
# (라) 등록과 준비
# ------------------------------------------------------------
world = World(
    stage_units_in_meters=1.0,                        # 길이 단위: 1 = 1 m
    physics_dt=1.0 / 120.0,                           # 물리 계산 주기 (s). 미션의 pick and place 와 같은 값
    rendering_dt=1.0 / 60.0,                          # 한 프레임 시간 (s)
)
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute(
    "SetLightingMenuModeCommand",                     # 실행할 명령 이름
    lighting_mode="Default",                          # 조명 모드
)
set_camera_view(eye=np.array([1.3, -0.1, 0.9]), target=np.array([0.4, 0.0, 0.1]))  # 뷰포트 카메라: 앞쪽 위에서 큐브와 놓을 위치를 본다
task = PickEnvTask(name="pick_env")                   # Task 이름
world.add_task(task)                                  # World 에 Task 등록
world.reset()  # 여기서 task.set_up_scene() -> task.post_reset() 이 불린다

params = task.get_params()
robot_name = params["robot_name"]["value"]            # "m0609"
cube_name = params["cube_name"]["value"]              # "cube"
robot = world.scene.get_object(robot_name)            # 이름으로 장면에서 로봇 객체를 찾는다
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]  # 팔 관절 6개가 관절 배열(12개)의 몇 번째인지 -> [0, 1, 2, 3, 4, 5]
controller = StepController(
    "step_controller",                                # 컨트롤러 이름
    gripper=robot.gripper,                            # 로봇에 붙은 그리퍼 객체
    arm_indices=arm,                                  # 팔 관절 6개가 몇 번째인지
)

# IK 결과 출력: 직선이 아닌 단계의 TCP 목표 위치 -> 팔 관절 6개 각도
for pose_name, position, joints in [
    ("큐브 위", ABOVE_POS, ABOVE_DEG),
    ("놓을 위치 위", PLACE_ABOVE_POS, PLACE_ABOVE_DEG),
]:
    x, y, z = position
    print(f"IK | {pose_name} | 목표 x {x:.2f} y {y:5.2f} z {z:.3f} m -> joint_1 ~ 6 {np.round(joints).astype(int)} deg")

# 그리기: 직선 단계(하강·상승)의 계획 경로를 녹색 선으로, 이동 중 실제 TCP 를 빨간 점으로
draw = _debug_draw.acquire_debug_draw_interface()  # 화면에 점·선을 그리는 도구 (Stage 에 물체를 만들지 않는다)
draw.draw_lines(
    [tuple(ABOVE_POS), tuple(PLACE_ABOVE_POS)],       # 선 시작점: 큐브 위, 놓을 위치 위
    [tuple(PICK_POS), tuple(PLACE_POS)],              # 선 끝점: 집는 위치, 내려놓는 위치
    [PLAN_COLOR] * 2,                                 # 선마다 색
    [PLAN_WIDTH] * 2,                                 # 선마다 굵기 (픽셀)
)


def draw_tcp(current_arm):
    """지금 관절값의 TCP 위치 (FK) 에 빨간 점을 찍는다"""
    tcp, _ = ik_solver.compute_forward_kinematics(TCP_FRAME, current_arm)
    draw.draw_points([tuple(tcp)], [ACTUAL_COLOR], [ACTUAL_SIZE])

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False
last_step = 0  # 바로 전 프레임의 단계 번호 (단계가 바뀌면 출력)
frame = 0      # Play 중에 지난 프레임 수 (빨간 점을 찍는 간격용)
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
            frame = 0
            draw.clear_points()  # 빨간 점만 지운다 (녹색 선은 그대로)
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
            frame += 1
            if frame % DRAW_EVERY == 0:
                draw_tcp(current_arm)  # 실제 TCP 를 빨간 점으로

        if controller.step != last_step:  # 단계가 끝났을 때 한 번 출력
            tcp, _ = ik_solver.compute_forward_kinematics(TCP_FRAME, current_arm)                  # 지금 관절값 -> TCP 위치 (FK)
            finger = observations[robot_name]["joint_positions"][robot.get_dof_index("finger_joint")]  # 그리퍼 각도 (rad)
            x, y, z = observations[cube_name]["position"]                                          # 큐브 위치 (m)
            print(f"시각 {world.current_time:5.2f} s | {STEPS[last_step][0]} 끝"
                  f" | TCP x {tcp[0]:.2f} y {tcp[1]:5.2f} z {tcp[2]:.3f} m -> joint_1 ~ 6 {np.round(np.degrees(current_arm)).astype(int)} deg"
                  f" | 그리퍼 {finger:.2f} rad | 큐브 x {x:5.2f} y {y:5.2f} z {z:.3f} m")
            last_step = controller.step
            if controller.is_done():
                print("모든 단계 완료")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
```

---

## 해설
1. **직선으로 내려가고 올라가는 pick and place**
   - **TODO 1: 생성기와 `plan_line()`**
     ```python
     generator = LulaTaskSpaceTrajectoryGenerator(
         robot_description_path=DESCRIPTION_PATH,          # 로봇 설명 파일
         urdf_path=URDF_PATH,                              # URDF 파일
     )
     # 속도 조절: 원래 한계값(URDF) 에 SPEED_SCALE 을 곱해 다시 넣는다
     generator.set_c_space_velocity_limits(np.array(generator.get_c_space_velocity_limits()) * SPEED_SCALE)
     generator.set_c_space_acceleration_limits(np.array(generator.get_c_space_acceleration_limits()) * SPEED_SCALE)


     def plan_line(start_joints, goal_position):
         """start_joints (rad) 의 TCP 에서 goal_position 까지 직선으로 가는 (관절값 목록, 관절 속도 목록) 을 돌려준다"""
         # ① 시작 TCP 위치·자세 (FK)
         start_pos, start_rot = ik_solver.compute_forward_kinematics(TCP_FRAME, start_joints)
         # ② 직선 경로: 시작 TCP -> 목표
         line = lula.create_task_space_path_spec(get_pose3(start_pos, rot_quat=rot_matrix_to_quat(start_rot)))
         line.add_linear_path(get_pose3(goal_position, rot_quat=DOWN_ORIENTATION))
         # ③ 묶음 경로: 시작 관절값을 정하고 ② 를 넣는다
         path = lula.create_composite_path_spec(start_joints.astype(np.float64))
         path.add_task_space_path_spec(line, lula.CompositePathSpec.TransitionMode.SKIP)
         # ④ trajectory -> DT 간격 관절값, 관절 속도 목록
         trajectory = generator.compute_task_space_trajectory_from_path_spec(path, TCP_FRAME)
         if trajectory is None:
             raise RuntimeError(f"직선 trajectory 를 만들지 못했다: {goal_position}")
         joint_list, velocity_list = [], []
         times = np.append(np.arange(trajectory.start_time, trajectory.end_time, DT), trajectory.end_time)  # 마지막은 끝 시각 (속도 0)
         for t in times:
             joints, velocities = trajectory.get_joint_targets(t)
             joint_list.append(joints)
             velocity_list.append(velocities)
         return joint_list, velocity_list
     ```

     - 예제의 (다), (마) ~ (사) 를 함수로 묶었다. 직선 단계가 4번 있으므로 함수로 만들면 단계마다 한 줄로 부를 수 있다
     - 예제와 다른 점: 예제는 `START_POS` 의 IK 로 시작 관절값을 구했지만, 여기서는 **지금 관절값** (`start_joints`) 이 시작이고 시작점은 그 FK 다. 이전 단계가 끝난 자세에서 바로 이어진다
     - 직선의 시작점은 지금 관절값의 TCP (FK) 다. `SKIP` 이라 시작 관절값의 TCP 와 직선의 시작점이 같아야 한다
     - 생성기는 처음에 한 번만 만든다. 속도 한계도 한 번 설정하면 계속 쓰인다
   - **TODO 2: `"line"` 단계**
     ```python
     # (TODO 2) 단계 첫 프레임: 지금 관절값에서 목표까지 직선 trajectory 를 만든다
     if self._step_frames == 1:
         self._joint_list, self._velocity_list = plan_line(current_arm, value)
         print(f"직선 | {name} | 관절값 {len(self._joint_list)} 개 ({len(self._joint_list) * DT:.2f} s)")
     # 한 프레임에 하나씩 보낸다. 다 보낸 뒤에는 마지막 것(멈춘 자세, 속도 0)을 계속 보낸다
     last = len(self._joint_list) - 1                # 마지막 번호
     index = min(self._step_frames - 1, last)        # 이번 프레임에 보낼 번호
     action = ArticulationAction(
         joint_positions=self._joint_list[index],      # 이 프레임의 관절값 (rad)
         joint_velocities=self._velocity_list[index],  # 이 프레임의 관절 속도 (rad/s)
         joint_indices=self._arm_indices,              # 팔 관절 6개 자리에 넣는다
     )
     arrived = np.max(np.abs(np.degrees(current_arm - self._joint_list[last]))) < TOLERANCE_DEG
     finished = index == last and arrived            # 다 보냈고 마지막 관절값에 도착
     ```

     - trajectory 는 단계 **첫 프레임**에 만든다. 시작할 때 미리 만들지 않는 이유: 직선의 시작 관절값이 이전 단계가 실제로 끝난 자세이기 때문이다 (그리퍼를 닫는 동안 팔이 조금 움직일 수도 있다)
     - 번호를 `min(..., last)` 로 막아서, 다 보낸 뒤에는 마지막 관절값(속도 0)을 계속 보낸다. 마지막 값의 속도가 0 이 아니면 팔이 지나쳐 간다
     - 끝 판정은 "다 보냈다" 와 "도착했다" 둘 다다. 다 보낸 순간에는 팔이 아직 조금 늦어 있을 수 있다
   - **바꾸지 않은 것**
     - `Task`, 메인 루프, `"arm"`, `"gripper"` 단계는 L8 과 같다. `STEPS` 에서 하강·상승 4 단계의 종류만 `"line"` 으로 바꾸고 값을 관절값 대신 TCP 목표 위치로 바꿨다
     - `PICK_DEG`, `PLACE_DEG` 는 필요 없어졌다. 아래 자세는 trajectory 가 경로를 따라 계산한다
   - **실행 결과**
     ```
     IK | 큐브 위 | 목표 x 0.45 y  0.20 z 0.200 m -> joint_1 ~ 6 [23 18 75  0 87 23] deg
     IK | 놓을 위치 위 | 목표 x 0.45 y -0.25 z 0.200 m -> joint_1 ~ 6 [-30  21  71   0  88 -30] deg
     시각  0.30 s | 큐브 위로 끝 | TCP x 0.45 y  0.20 z 0.204 m -> joint_1 ~ 6 [23 17 76  0 87 23] deg | 그리퍼 0.00 rad | 큐브 x  0.45 y  0.20 z 0.026 m
     직선 | 내려가기 | 관절값 59 개 (0.98 s)
     시각  1.28 s | 내려가기 끝 | TCP x 0.45 y  0.20 z 0.036 m -> joint_1 ~ 6 [23 24 93  0 63 23] deg | 그리퍼 0.00 rad | 큐브 x  0.45 y  0.20 z 0.026 m
     시각  2.28 s | 그리퍼 닫기 끝 | TCP x 0.45 y  0.20 z 0.039 m -> joint_1 ~ 6 [23 24 93  0 63 23] deg | 그리퍼 0.63 rad | 큐브 x  0.45 y  0.20 z 0.026 m
     직선 | 들어 올리기 | 관절값 59 개 (0.98 s)
     시각  3.27 s | 들어 올리기 끝 | TCP x 0.45 y  0.20 z 0.196 m -> joint_1 ~ 6 [23 18 75  0 87 23] deg | 그리퍼 0.69 rad | 큐브 x  0.45 y  0.20 z 0.177 m
     시각  3.65 s | 옆으로 옮기기 끝 | TCP x 0.45 y -0.25 z 0.195 m -> joint_1 ~ 6 [-29  22  71   0  88 -30] deg | 그리퍼 0.69 rad | 큐브 x  0.45 y -0.25 z 0.176 m
     직선 | 내려놓기 | 관절값 58 개 (0.97 s)
     시각  4.62 s | 내려놓기 끝 | TCP x 0.45 y -0.25 z 0.043 m -> joint_1 ~ 6 [-30  26  89   0  65 -30] deg | 그리퍼 0.69 rad | 큐브 x  0.45 y -0.25 z 0.026 m
     시각  5.62 s | 그리퍼 열기 끝 | TCP x 0.45 y -0.25 z 0.036 m -> joint_1 ~ 6 [-30  27  89   0  65 -30] deg | 그리퍼 0.00 rad | 큐브 x  0.45 y -0.25 z 0.026 m
     직선 | 올라가기 | 관절값 59 개 (0.98 s)
     시각  6.60 s | 올라가기 끝 | TCP x 0.45 y -0.25 z 0.195 m -> joint_1 ~ 6 [-30  21  71   0  88 -30] deg | 그리퍼 0.00 rad | 큐브 x  0.45 y -0.25 z 0.026 m
     모든 단계 완료
     ```

     - `직선 |` 줄: 단계 첫 프레임에 만든 trajectory 의 관절값 개수. 약 16 cm 직선이 약 1 초 (59 프레임)
     - 직선 단계가 끝났을 때 TCP 의 x, y 가 목표와 같다. L8 에서는 하강 중에 옆으로 15 ~ 24 mm 흔들렸고, 이제는 1 ~ 3 mm 다 (내려놓기는 바로 앞 "옆으로 옮기기" 의 흔들림이 남아 약 1 cm)
     - L8 보다 약 3 초 길다. 하강·상승을 직선 trajectory 의 속도 한계(`SPEED_SCALE` 0.3) 로 움직이기 때문이다
     - 큐브가 놓을 위치 (0.45, -0.25) 에 놓였다
     - 화면: 하강·상승의 빨간 점(실제 TCP)이 녹색 선(계획 경로) 위에 세로로 찍힌다. "큐브 위로", "옆으로 옮기기" 의 빨간 점은 곡선이다 (끝 관절값 하나로 이동)

---

## 확인할 것
- [ ] `plan_line()` 에서 시작 관절값으로 지금 관절값을 쓰는 이유를 설명할 수 있다
- [ ] trajectory 를 시작할 때가 아니라 단계 첫 프레임에 만드는 이유를 설명할 수 있다
- [ ] 다 보낸 뒤 마지막 관절값을 계속 보내는 이유를 설명할 수 있다
