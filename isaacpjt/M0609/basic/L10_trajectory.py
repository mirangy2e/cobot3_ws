"""[L10 예제] trajectory 로 직선 이동: 시작점과 끝점을 주면 그 사이를 직선으로 가는 시간별 관절값 목록이 나온다

실행: isaac_python L10_trajectory.py
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
import lula
import numpy as np
import omni.kit.commands
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver, LulaTaskSpaceTrajectoryGenerator
from isaacsim.robot_motion.motion_generation.lula.utils import get_pose3
from isaacsim.util.debug_draw import _debug_draw

# 파일 경로
M0609_DIR = Path(__file__).resolve().parents[1]                            # isaacpjt/M0609 (이 파일이 있는 basic 의 상위 폴더)
ROBOT_USD = f"{M0609_DIR}/robot/m0609_rg2_d455.usd"                        # 장면에 넣을 로봇 (M0609 + RG2 + D455)
URDF_PATH = f"{M0609_DIR}/robot/m0609_rg2.urdf"                            # 계산용: 링크 길이, 관절 축, 관절 한계
DESCRIPTION_PATH = f"{M0609_DIR}/descriptor/m0609_robot_description.yaml"  # 계산용: 움직일 관절 (팔 6개)

# 로봇 관절
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
TCP_FRAME = "gripper_center"  # 경로를 따라갈 기준점: 그리퍼 손가락 사이

# 그리퍼가 아래를 향하는 자세 (L8 연습 문제와 같음)
DOWN_ORIENTATION = euler_angles_to_quat(np.radians([0, 180, 0]))

# 직선 이동 (실습에서 바꿔 볼 값): 그리퍼가 아래를 향한 채 START_POS -> END_POS 직선
START_POS = np.array([0.40, 0.35, 0.20])   # 시작점: 왼쪽 앞, 높이 20 cm
END_POS = np.array([0.40, -0.35, 0.20])    # 끝점: 오른쪽 앞, 같은 높이 (가로로 70 cm)
SPEED_SCALE = 0.3  # 로봇 최고 속도의 몇 배로 움직일지 (작을수록 느리다)

# 진행
DT = 1.0 / 60.0   # 한 프레임 시간 (s). 관절값을 이 간격으로 꺼낸다
START_FRAME = 60  # Play 후 이 프레임부터 이동 시작 (그 전 1 초는 시작 자세)
PRINT_EVERY = 20  # 관절값 목록과 이동 중 TCP 를 20 개(프레임)마다 한 줄씩 출력
DRAW_EVERY = 3    # 실제 TCP 를 3 프레임마다 점 하나로 그린다

# 그리기 (색은 R, G, B, 투명도 / 크기는 화면 픽셀)
PLAN_COLOR = (0.0, 0.8, 0.0, 1.0)    # 계획 경로: 녹색 선
PLAN_WIDTH = 3
ACTUAL_COLOR = (1.0, 0.0, 0.0, 1.0)  # 실제 TCP: 빨간 점 (계획대로 가면 녹색 선 위에 찍힌다)
ACTUAL_SIZE = 10


# ------------------------------------------------------------
# 출력·그리기용 함수: 결과를 보여 주기만 한다 (로직을 볼 때는 건너뛰어도 된다)
#   아래 본문에서 만드는 ik_solver, robot, arm 을 사용한다
# ------------------------------------------------------------
draw = _debug_draw.acquire_debug_draw_interface()  # 화면에 점·선을 그리는 도구 (Stage 에 물체를 만들지 않는다)


def to_deg(joints):
    """관절값 (rad) -> 정수 deg 배열 (출력용)"""
    return np.round(np.degrees(joints)).astype(int)


def print_joint_list(times, joint_list):
    """관절값 목록을 PRINT_EVERY 개마다 한 줄씩, 그 관절값의 TCP 위치 (FK) 와 함께 출력한다"""
    print(f"trajectory | {times[-1]:.2f} s, 관절값 {len(joint_list)} 개 ({DT:.4f} s 간격)")
    for i in range(0, len(joint_list), PRINT_EVERY):
        tcp, _ = ik_solver.compute_forward_kinematics(TCP_FRAME, joint_list[i])
        print(f"  [{i:3d}] t {times[i]:4.2f} s | joint_1 ~ 6 {to_deg(joint_list[i])} deg"
              f" | TCP x {tcp[0]:.2f} y {tcp[1]:5.2f} z {tcp[2]:.3f} m")


def robot_tcp():
    """로봇의 지금 팔 관절값과 그때의 TCP 위치 (FK)"""
    current = robot.get_joint_positions()[arm]
    tcp, _ = ik_solver.compute_forward_kinematics(TCP_FRAME, current)
    return current, tcp


def print_robot(frame, state):
    """로봇의 지금 관절값과 TCP 위치를 한 줄 출력한다"""
    current, tcp = robot_tcp()
    print(f"{frame} 프레임 | {state} | joint_1 ~ 6 {to_deg(current)} deg"
          f" | TCP x {tcp[0]:.2f} y {tcp[1]:5.2f} z {tcp[2]:.3f} m")


def draw_plan_line():
    """계획 경로: 시작점 -> 끝점을 녹색 선으로 그린다"""
    draw.draw_lines(
        [tuple(START_POS)],                           # 선 시작점 목록 [(x, y, z), ...]
        [tuple(END_POS)],                             # 선 끝점 목록
        [PLAN_COLOR],                                 # 선마다 색
        [PLAN_WIDTH],                                 # 선마다 굵기 (픽셀)
    )


def show_moving(frame, index):
    """이동 중: 실제 TCP 를 DRAW_EVERY 프레임마다 빨간 점으로 그리고, PRINT_EVERY 프레임마다 출력한다"""
    if index % DRAW_EVERY == 0:
        _, tcp = robot_tcp()
        draw.draw_points([tuple(tcp)], [ACTUAL_COLOR], [ACTUAL_SIZE])
    if index % PRINT_EVERY == 0:
        print_robot(frame, "이동")


# (가) 장면: World, 바닥, 조명, 카메라
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")  # Lighting 메뉴의 "Default"
set_camera_view(eye=np.array([1.3, -0.2, 1.1]), target=np.array([0.3, 0.0, 0.15]))  # 앞쪽 위에서 내려다본다

# (나) 로봇
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/m0609_rg2_d455")
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))


# ************************************************************
# ★ trajectory 영역 시작: 시작점 -> 끝점 직선 -> 시간별 팔 관절 6개 각도 목록
# ************************************************************

# (다) 계산기 두 개 (같은 파일 두 개로 만든다)
#   ik_solver : IK, FK 용 (L8 의 IK 계산기)
#   generator : trajectory 생성기
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,
    urdf_path=URDF_PATH,
)
generator = LulaTaskSpaceTrajectoryGenerator(
    robot_description_path=DESCRIPTION_PATH,
    urdf_path=URDF_PATH,
)

# 속도 조절: 원래 한계값에 SPEED_SCALE 을 곱해 다시 넣는다
velocity_limits = np.array(generator.get_c_space_velocity_limits())
acceleration_limits = np.array(generator.get_c_space_acceleration_limits())
generator.set_c_space_velocity_limits(velocity_limits * SPEED_SCALE)
generator.set_c_space_acceleration_limits(acceleration_limits * SPEED_SCALE)


# (라) 시작 관절값: 시작점의 IK (L8 과 같음)
#    start_joints : [41, 23, 67, 0, 89, 41] deg (rad 로 저장)
start_joints, success = ik_solver.compute_inverse_kinematics(
    TCP_FRAME, START_POS, DOWN_ORIENTATION, warm_start=np.radians(HOME_DEG)
)
if not success:
    raise RuntimeError("시작점의 IK 해가 없다")


# (마) 경로 만들기 (예제 값 기준으로 단계마다 만들어지는 것)

# ① 직선 경로 계획: 시작 자세 -> 끝 자세 직선
line = lula.create_task_space_path_spec(get_pose3(START_POS, rot_quat=DOWN_ORIENTATION))
line.add_linear_path(get_pose3(END_POS, rot_quat=DOWN_ORIENTATION))

# ② 묶음 경로 계획: 시작 관절값(팔 형태 유지) + ①
path = lula.create_composite_path_spec(start_joints)
path.add_task_space_path_spec(line, lula.CompositePathSpec.TransitionMode.SKIP)


# (바) trajectory: 직선 경로 계획 -> 시간에 따라 움직이는 관절값
#    trajectory : 0 ~ 2.89 s 동안의 관절값. 시각 t 를 넣으면 그때의 관절값과 속도가 나온다
trajectory = generator.compute_task_space_trajectory_from_path_spec(path, TCP_FRAME)
if trajectory is None:
    raise RuntimeError("trajectory 를 만들지 못했다: 팔이 닿지 않는 곳이 있다")


# (사) 관절값 목록: DT 간격의 시각마다 관절값과 관절 속도를 꺼낸다 (여기서 처음으로 세부 점이 생긴다)
#    times         : [0.000, 0.017, 0.033, ..., 2.89] s                            (175 개)
#    joint_list    : [[41, 23, 67, 0, 89, 41], ..., [-42, 23, 67, 0, 89, -42]] deg  (175 개, rad 로 저장)
#    velocity_list : 같은 시각의 관절 속도 (rad/s). 마지막은 0                       (175 개)
times = np.arange(trajectory.start_time, trajectory.end_time, DT)
times = np.append(times, trajectory.end_time)  # 마지막은 끝 시각 (멈춘 자세, 속도 0)

joint_list = []
velocity_list = []
for t in times:
    joints, velocities = trajectory.get_joint_targets(t)
    joint_list.append(joints)
    velocity_list.append(velocities)

# ************************************************************
# ★ trajectory 영역 끝
# ************************************************************

print_joint_list(times, joint_list)  # 관절값 목록 출력
draw_plan_line()                     # 계획 경로 (녹색 선)


# (아) 시작 자세로 순간이동하고, 멈춘 상태로 기다린다
world.reset()
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]  # 팔 관절 6개가 관절 배열(12개)의 몇 번째인지 -> [0, 1, 2, 3, 4, 5]
start = np.zeros(robot.num_dof)
start[arm] = start_joints
robot.set_joint_positions(start)
world.step(render=True)  # 한 프레임 진행해서 순간이동한 자세를 화면에 반영
world.pause()            # 멈춤. 화면 왼쪽의 Play(▶) 를 누르면 시작
print("Play 를 누르면 이동을 시작한다 (Stop -> Play: 처음부터 다시)")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0             # Play 중에 지난 프레임 수
reset_needed = False
while simulation_app.is_running():
    world.step(render=True)

    # Stop 을 감지해 둔다. Play 중일 때만 아래를 진행한다
    if world.is_stopped():
        reset_needed = True
    if not world.is_playing():
        continue
    # Stop 다음 첫 Play 프레임: 시작 자세와 프레임 수를 처음으로
    if reset_needed:
        world.reset()
        robot.set_joint_positions(start)
        frame = 0
        draw.clear_points()  # 빨간 점만 지운다 (녹색 선은 그대로)
        reset_needed = False

    frame += 1
    index = frame - START_FRAME  # 이번 프레임에 보낼 관절값 번호

    # (자) 목록의 관절값을 한 프레임에 하나씩 보낸다
    if 0 <= index < len(joint_list):
        action = ArticulationAction(
            joint_positions=joint_list[index],      # 이 시각의 관절값 (rad)
            joint_velocities=velocity_list[index],  # 이 시각의 관절 속도 (같이 주면 늦지 않게 따라간다)
            joint_indices=arm,                      # 팔 관절 6개 자리에
        )
        robot.apply_action(action)
        show_moving(frame, index)  # 실제 TCP 를 빨간 점으로, 가끔 출력

    # (차) 다 보내고 1 초 뒤: 도착 자세 출력
    if index == len(joint_list) + 60:
        print_robot(frame, "도착")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
