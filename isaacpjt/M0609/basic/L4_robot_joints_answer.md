---
notion_parent: https://app.notion.com/p/27b450ef7c5983939fa701761ca13d7d
notion_page: https://app.notion.com/p/406450ef7c59807b8600edb41b543bae
---

# L4. 연습 문제 정답

---

## 정답 코드

### L4_robot_joints_answer.py
```python
"""[L4 연습 정답] 로봇 관절 제어"""

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
MOVE_FRAME = 100     # 이 프레임에 모터 목표를 보낸다

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
add_reference_to_stage(usd_path=str(ROBOT_USD), prim_path="/World/m0609_rg2_d455")
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))
world.reset()
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]
finger = robot.get_dof_index("finger_joint")

# 순간이동: 홈 자세, 그리퍼 열기
home = np.zeros(robot.num_dof)
home[arm] = np.radians(HOME_DEG)
home[finger] = 0.0
robot.set_joint_positions(home)


def send_goal(goal_deg):
    """joint_1 을 goal_deg 로 보내는 모터 목표. -45 deg 로 갈 때 그리퍼 닫기, 45 deg 로 갈 때 열기"""
    target = home.copy()
    target[arm[0]] = np.radians(goal_deg)
    target[finger] = 0.9 if goal_deg < 0 else 0.0
    robot.apply_action(ArticulationAction(joint_positions=target))


# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
goal_deg = -45.0  # 지금 joint_1 이 가고 있는 목표 (deg)
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1
    if frame == MOVE_FRAME:  # 첫 모터 목표: joint_1 -45 deg, 그리퍼 닫기
        send_goal(goal_deg)
    # 도착하면 목표를 반대쪽(-45 <-> 45)으로 바꿔 보낸다
    joint_1_deg = np.degrees(robot.get_joint_positions()[arm[0]])
    if frame >= MOVE_FRAME and abs(joint_1_deg - goal_deg) < TOLERANCE_DEG:
        print(f"{frame} 프레임 | joint_1 {joint_1_deg:6.1f} deg, {goal_deg:.0f} deg 도착")
        goal_deg = -goal_deg
        send_goal(goal_deg)

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
```

```bash
cd ~/cobot3_ws/isaacpjt/M0609/basic
isaac_python L4_robot_joints_answer.py
```

---

## 해설
1. **도착하면 반대쪽으로**
   ```python
   joint_1_deg = np.degrees(robot.get_joint_positions()[arm[0]])
   if frame >= MOVE_FRAME and abs(joint_1_deg - goal_deg) < TOLERANCE_DEG:
       print(f"{frame} 프레임 | joint_1 {joint_1_deg:6.1f} deg, {goal_deg:.0f} deg 도착")
       goal_deg = -goal_deg
       send_goal(goal_deg)
   ```
   - **도착 판정**: 모터는 목표에 조금씩 다가가므로 실제 각도가 -45.000 으로 딱 맞는 프레임이 없을 수 있다. 차이의 절댓값이 `TOLERANCE_DEG`(1 deg) 보다 작으면 도착으로 본다
   - **목표 바꾸기**: `goal_deg = -goal_deg` 로 -45 와 45 를 오간다. 목표가 바뀌면 지금 각도와 새 목표의 차이가 90 deg 가 되므로, 다음 프레임에는 조건이 저절로 거짓이 되어 한 번만 출력된다
   - `frame >= MOVE_FRAME`: 첫 목표를 보내기 전에는 홈 자세(0 deg)라서 판정할 필요가 없다
   - `send_goal()` 은 `home.copy()` 에서 joint_1 과 그리퍼만 바꿔 보낸다. `target = home` 이라 쓰면 같은 배열을 가리켜서 `home` 까지 바뀐다
   - **실행 결과**
     ```
     118 프레임 | joint_1  -44.2 deg, -45 deg 도착
     156 프레임 | joint_1   45.0 deg, 45 deg 도착
     193 프레임 | joint_1  -45.0 deg, -45 deg 도착
     230 프레임 | joint_1   45.0 deg, 45 deg 도착
     ...
     ```
     - 처음에는 0 -> -45 deg 라서 18 프레임, 그 뒤로는 90 deg 를 움직이므로 약 37 프레임마다 도착한다
     - 시간(프레임 수)을 정해 두지 않아도, 도착할 때마다 다음 동작으로 넘어간다

---

## 확인할 것
- [ ] `==` 대신 오차 범위로 도착을 판정하는 이유를 설명할 수 있다
- [ ] 도착 출력이 목표마다 한 번씩만 나오는 이유를 설명할 수 있다
