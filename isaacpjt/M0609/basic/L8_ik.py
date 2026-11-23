"""[L8 예제] IK 로 관절 각도 구하기: 목표 위치와 자세를 넣으면 팔 관절 각도가 나온다

실행: isaac_python L8_ik.py
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
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

# 파일 경로
M0609_DIR = Path(__file__).resolve().parents[1]                            # isaacpjt/M0609 (이 파일이 있는 basic 의 상위 폴더)
ROBOT_USD = f"{M0609_DIR}/robot/m0609_rg2_d455.usd"                        # 장면에 넣을 로봇 (M0609 + RG2 + D455)
URDF_PATH = f"{M0609_DIR}/robot/m0609_rg2.urdf"                            # IK 용: 링크 길이, 관절 축, 관절 한계
DESCRIPTION_PATH = f"{M0609_DIR}/descriptor/m0609_robot_description.yaml"  # IK 용: 움직일 관절 (팔 6개)

# 로봇 관절
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]
TCP_FRAME = "gripper_center"  # IK 로 위치를 맞출 기준점: 그리퍼 손가락 사이 (URDF 의 프레임 이름)

# 목표 위치와 자세 (실습에서 바꿔 볼 값)
TARGET_POS = np.array([0.45, 0.00, 0.45])  # TCP 목표 위치 [x, y, z] (m)
ROLL_DEG = 180   # 접근 방향 (roll, pitch): (0, 0) 하늘 / (90, 0) -y 수평 / (180, 0) 바닥 / (0, 90) +x 수평
PITCH_DEG = 0
YAW_DEG = 0      # 그리퍼 회전: 바닥 방향 (180, 0) 일 때 세로축으로 돌린 각도. 0 / 45 / 90 / 135
TARGET_ORIENT = euler_angles_to_quat(np.radians([ROLL_DEG, PITCH_DEG, YAW_DEG]))  # deg -> rad -> 쿼터니언 [w, x, y, z]

# 진행
MOVE_FRAME = 60    # 이 프레임에 IK 결과를 팔에 보낸다 (그 전 1 초는 홈 자세)
CHECK_FRAME = 240  # 이 프레임에 도착한 자세를 출력한다 (보낸 뒤 3 초)

# (가) 장면: World, 바닥, 조명
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

# (나) 로봇과 목표 표시 상자
add_reference_to_stage(usd_path=ROBOT_USD, prim_path="/World/m0609_rg2_d455")
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))
marker = world.scene.add(
    VisualCuboid(
        prim_path="/World/Marker",                    # Stage 에 만들 prim 경로
        name="marker",                                # 장면에서 찾을 때 쓰는 이름
        position=TARGET_POS,                          # 목표 위치에 둔다 (그리퍼 손가락 사이가 여기로 온다)
        orientation=TARGET_ORIENT,                    # 목표 자세만큼 돌려 둔다
        scale=np.array([0.03] * 3),                   # 한 변 3 cm
        size=1.0,                                     # 기본 한 변 1 m. 실제 크기 = size x scale
        color=np.array([1.0, 0, 0]),                  # 빨간색
    )
)
world.reset()

# ************************************************************
# ★ IK 영역 시작: 목표 위치 [x, y, z] + 자세 -> 팔 관절 6개 각도
# ************************************************************

# 팔 관절 6개가 관절 배열(12개)의 몇 번째인지 -> [0, 1, 2, 3, 4, 5]
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]

# (다) IK 계산기: 파일 두 개로 만든다 (장면의 로봇과 상관없이 계산만 한다)
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,          # 로봇 설명 파일
    urdf_path=URDF_PATH,                              # URDF 파일
)

# (라) IK: 목표 위치 + 자세 -> 팔 관절 6개 각도 (rad)
joints, success = ik_solver.compute_inverse_kinematics(
    TCP_FRAME,                                        # 목표에 맞출 프레임
    TARGET_POS,                                       # 목표 위치 [x, y, z] (m)
    TARGET_ORIENT,                                    # 목표 자세 (쿼터니언 [w, x, y, z])
    warm_start=np.radians(HOME_DEG),                  # 계산을 시작할 관절값 (rad). 여기서 가까운 답을 찾는다
)
x, y, z = TARGET_POS
print(f"IK | 목표 x {x:.2f} y {y:5.2f} z {z:.2f} m, roll {ROLL_DEG} pitch {PITCH_DEG} yaw {YAW_DEG} deg"
      f" -> joint_1 ~ 6 {np.round(np.degrees(joints)).astype(int)} deg | 성공 {success}")
if not success:
    print("IK 해 없음: 팔이 닿지 않는 위치나 자세다. 팔은 홈 자세에 그대로 있는다")

# (마) 홈 자세로 순간이동
home = np.zeros(robot.num_dof)
home[arm] = np.radians(HOME_DEG)
robot.set_joint_positions(home)

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1
    if frame == MOVE_FRAME and success:  # (바) IK 로 구한 관절값을 팔에 보낸다 (답이 없으면 보내지 않음)
        robot.apply_action(ArticulationAction(joint_positions=joints, joint_indices=arm))
        print(f"{frame} 프레임 | IK 결과로 이동 시작")
    if frame == CHECK_FRAME:  # (사) 도착한 자세 확인: 지금 관절값으로 FK 를 계산해 TCP 위치를 본다
        current = robot.get_joint_positions()[arm]
        tcp, _ = ik_solver.compute_forward_kinematics(TCP_FRAME, current)
        print(f"{frame} 프레임 | 도착 | joint_1 ~ 6 {np.round(np.degrees(current)).astype(int)} deg"
              f" | TCP x {tcp[0]:.2f} y {tcp[1]:5.2f} z {tcp[2]:.2f} m")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
