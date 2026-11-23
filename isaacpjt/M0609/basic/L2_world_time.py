"""[L2 예제] World 와 프레임 시간

실행: isaac_python L2_world_time.py   (창을 닫으면 종료)
"""

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
from isaacsim.core.api.objects import DynamicCuboid

RENDERING_DT = 1.0 / 60.0  # 한 프레임의 시간 (초)

# 장면 구성: World 생성 -> 물체 추가 -> reset
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=RENDERING_DT)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/Cube", name="cube", position=np.array([0.0, 0.0, 1.0]), size=0.1)
)
world.reset()  # 물체를 다 넣은 뒤 한 번: 물리를 시작할 준비

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)  # 한 프레임(1/60 초) 진행. 이 안에서 큐브가 중력으로 떨어진다
    frame += 1
    if frame % 30 == 0 and frame <= 180:  # 처음 180 프레임 동안 30 프레임마다 출력
        height = cube.get_world_pose()[0][2]
        print(f"프레임 {frame:3d} | 시간 {frame * RENDERING_DT:.2f} 초 | 큐브 높이 {height:.3f} m")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
