"""[L5 연습 정답] Stop -> Play 해도 처음부터 다시"""

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
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction

ROBOT_USD = Path(__file__).resolve().parents[1] / "robot" / "m0609_rg2_d455.usd"
HOME_DEG = [0, 0, 90, 0, 90, 0]
TOLERANCE_DEG = 1.0  # 목표와 이만큼 안으로 들어오면 "도착"

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path="/World/m0609_rg2_d455")
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))
world.reset()
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]
finger = robot.get_dof_index("finger_joint")

# 홈 자세: world.reset() 때 돌아갈 자세로 등록하고, 지금 순간이동
home = np.zeros(robot.num_dof)
home[arm] = np.radians(HOME_DEG)
robot.set_joints_default_state(positions=home)
robot.set_joint_positions(home)


def send_goal(goal_deg):
    """joint_1 을 goal_deg 로 보내는 모터 목표. -45 deg 로 갈 때 그리퍼 닫기, 45 deg 로 갈 때 열기"""
    target = home.copy()
    target[arm[0]] = np.radians(goal_deg)
    target[finger] = 0.9 if goal_deg < 0 else 0.0
    robot.apply_action(ArticulationAction(joint_positions=target))


goal_deg = -45.0  # 지금 joint_1 이 가고 있는 목표 (deg)
send_goal(goal_deg)

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False  # Stop 을 눌렀는지 기억해 둔다
while simulation_app.is_running():
    world.step(render=True)
    if world.is_stopped() and not reset_needed:  # Stop 을 감지해 둔다
        reset_needed = True
    if world.is_playing():
        if reset_needed:  # Stop 다음 첫 Play 프레임: 먼저 reset, 내 변수도 처음 값으로
            world.reset()
            goal_deg = -45.0
            send_goal(goal_deg)
            reset_needed = False
            print("reset 완료: 홈 자세에서 처음부터 다시 시작")
        joint_1_deg = np.degrees(robot.get_joint_positions()[arm[0]])
        if abs(joint_1_deg - goal_deg) < TOLERANCE_DEG:
            print(f"시각 {world.current_time:5.2f} s | joint_1 {joint_1_deg:6.1f} deg, {goal_deg:.0f} deg 도착")
            goal_deg = -goal_deg
            send_goal(goal_deg)

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
