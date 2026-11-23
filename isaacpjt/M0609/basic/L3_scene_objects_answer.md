---
notion_parent: https://app.notion.com/p/27b450ef7c5983939fa701761ca13d7d
notion_page: https://app.notion.com/p/406450ef7c59802a9d8bea4432fac1d7
---

# L3. 연습 문제 정답

---

## 정답 코드

### L3_scene_objects_answer.py
```python
"""[L3 연습 정답] 장면 객체: 위치와 자세"""

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
from isaacsim.core.api.objects import DynamicCone, VisualCone
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

OFFSET = np.array([0.0, 0.0, 0.3])  # yellow 는 red 보다 0.3 m 위에

# TODO 1: 기울어진 채 떨어지는 red, 그것을 따라 할 yellow
red = world.scene.add(
    DynamicCone(
        prim_path="/World/Red", name="red", position=np.array([0.4, 0.0, 0.5]),
        orientation=euler_angles_to_quat(np.array([60, 0, 0]), degrees=True),
        radius=0.05, height=0.15, color=np.array([1.0, 0, 0]),
    )
)
yellow = world.scene.add(
    VisualCone(
        prim_path="/World/Yellow", name="yellow", position=np.array([0.4, 0.0, 0.8]),
        radius=0.05, height=0.15, color=np.array([1.0, 1.0, 0]),
    )
)
world.reset()

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1
    position, orientation = red.get_world_pose()  # TODO 2: red 의 위치, 자세를 읽어
    yellow.set_world_pose(position=position + OFFSET, orientation=orientation)  # yellow 에 복사
    if frame % 60 == 0:  # TODO 3
        angles = quat_to_euler_angles(orientation, degrees=True)
        print(f"{frame} 프레임 | red 위치 {np.round(position, 3)} | 각도(deg) {np.round(angles, 1)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()

```

```bash
cd ~/cobot3_ws/isaacpjt/M0609/basic
isaac_python L3_scene_objects_answer.py
```

---

## 해설
1. **원뿔 두 개 추가**
   ```python
   orientation=euler_angles_to_quat(np.array([60, 0, 0]), degrees=True)
   ```
   - x 축 둘레로 60 deg 기울인 자세를 쿼터니언으로 바꿔 넣는다. `degrees=True` 가 없으면 60 을 rad 로 읽어 엉뚱하게 돈다
   - `red` 는 `DynamicCone` 이라 중력으로 떨어지고, 기울어진 채 바닥에 닿아 옆으로 눕는다
   - `yellow` 는 `VisualCone` 이라 물리가 없다. 코드로 옮겨 주지 않으면 처음 위치에 그대로 떠 있다
   - `yellow` 에는 `orientation` 을 주지 않았다. 주지 않으면 기본 자세(끝이 위)로 만들어진다
2. **위치, 자세 복사**
   ```python
   position, orientation = red.get_world_pose()
   yellow.set_world_pose(position=position + OFFSET, orientation=orientation)
   ```
   - `get_world_pose()` 는 (위치, 자세) 두 값을 돌려주므로 변수 두 개로 나눠 받는다
   - `position + OFFSET`: numpy 배열이라 `[x, y, z] + [0, 0, 0.3]` 이 칸마다 더해져 0.3 m 위가 된다
   - 자세는 쿼터니언 그대로 넘긴다. 각도로 바꿨다가 다시 바꿀 필요가 없다
   - 반대로 `yellow`(Visual) 를 손으로 옮기고 `red`(Dynamic) 가 따라 하게 하면, 물리가 매 프레임 `red` 를 다시 움직여서 떨린다. **물리가 움직이는 물체를 읽고, 물리가 없는 물체를 옮기는** 쪽이 깔끔하다
3. **각도 출력**
   ```python
   if frame % 60 == 0:
       angles = quat_to_euler_angles(orientation, degrees=True)
   ```
   - 실행 결과(드래그하지 않았을 때): `red` 위치 약 `[0.4 -0.069 0.024]`, 각도 약 `[108.4 0 0]` deg
     - 60 deg 기울어진 채 떨어져 더 넘어가 옆으로 누웠다. 그래서 x 축 각도만 커졌다
     - 높이 0.024 m: 원뿔이 누워서 중심이 바닥 가까이 내려왔다
   - `yellow` 는 같은 자세로 높이 약 0.324 m(0.3 m 위)에 떠 있다
   - 원뿔을 자기 축 둘레로 굴리면 모양은 그대로라 눈으로는 안 보이지만, 출력되는 각도는 바뀐다

---

## 확인할 것
- [ ] Dynamic 물체를 읽고 Visual 물체를 옮기는 이유를 설명할 수 있다
- [ ] `euler_angles_to_quat` 에 `degrees=True` 를 빼면 어떻게 되는지 설명할 수 있다
