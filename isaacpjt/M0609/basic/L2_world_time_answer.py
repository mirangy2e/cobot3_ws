"""[L2 연습 정답] World 와 프레임 시간

2번 문항 답: world.reset() 은 장면의 모든 물체를 처음 상태(위치, 속도)로 되돌린다. 그래서 바닥에 있던 큐브가
           다시 1 m 높이로 돌아가 떨어진다.
3번 문항 답: set_world_pose 는 그 물체 하나만 옮긴다. reset 과 달리 다른 물체와 시뮬레이션 상태는 그대로다.
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

RENDERING_DT = 1.0 / 60.0

world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=RENDERING_DT)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/Cube", name="cube", position=np.array([0.0, 0.0, 1.0]), size=0.1)
)
world.reset()

FRAMES_2S = round(2.0 / RENDERING_DT)  # 2초 = 120 프레임
print(f"2초는 {FRAMES_2S} 프레임")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
frame = 0
while simulation_app.is_running():
    world.step(render=True)
    frame += 1
    height = cube.get_world_pose()[0][2]
    if frame == FRAMES_2S:  # 2초 뒤 큐브 높이
        print(f"{frame} 프레임 ({frame * RENDERING_DT:.1f} 초): 큐브 높이 {height:.3f} m")
    if frame == 180:  # 루프 도중 reset
        print(f"reset 전 큐브 높이 {height:.3f} m")
        world.reset()
        print(f"reset 후 큐브 높이 {cube.get_world_pose()[0][2]:.3f} m")
    if frame == 300:  # 물체 하나만 옮기기
        cube.set_world_pose(position=np.array([0.3, 0.0, 1.0]))
        print("큐브를 (0.3, 0, 1.0) 으로 옮김")
    if frame == 360:  # 옮긴 뒤 위치 확인
        print(f"옮긴 뒤 60 프레임: 큐브 위치 {np.round(cube.get_world_pose()[0], 3)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
