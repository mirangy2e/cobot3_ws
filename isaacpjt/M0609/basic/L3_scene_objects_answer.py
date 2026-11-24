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

# 기울어진 채 떨어지는 red, 그것을 따라 할 yellow
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
    position, orientation = red.get_world_pose()  # red 의 위치, 자세를 읽어
    yellow.set_world_pose(position=position + OFFSET, orientation=orientation)  # yellow 에 복사
    if frame % 60 == 0:  # 60 프레임마다 각도 출력
        angles = quat_to_euler_angles(orientation, degrees=True)
        print(f"{frame} 프레임 | red 위치 {np.round(position, 3)} | 각도(deg) {np.round(angles, 1)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
