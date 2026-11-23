"""[L5 예제] 메인 루프와 Play / Stop

실행: isaac_python L5_main_loop.py
      큐브가 떨어져 바닥 가까이 오면 다시 위로 올라간다. 창에서 Pause -> Play, Stop -> Play 를 눌러 본다
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

START_POSITION = np.array([0.0, 0.0, 1.0])
LOW_HEIGHT = 0.15  # 큐브 중심이 이보다 낮으면 "바닥 가까이" (바닥에 닿기 전에 올려서 튕기거나 구르지 않게)

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()
# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
cube = world.scene.add(
    DynamicCuboid(prim_path="/World/Cube", name="cube", position=START_POSITION, size=0.1, color=np.array([0, 0, 1.0]))
)
world.reset()

drop_count = 0  # 내 변수: 떨어진 횟수

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False
while simulation_app.is_running():
    world.step(render=True)

    # 1) Stop 을 감지해 둔다
    if world.is_stopped() and not reset_needed:
        reset_needed = True
        print("Stop 감지: 다음 Play 때 reset 한다")
    # 2) Play 중일 때만 제어 (Pause 중에는 물리도 내 코드도 멈춘다)
    if world.is_playing():
        if reset_needed:  # Stop 다음 첫 Play 프레임: 먼저 reset
            world.reset()   # 장면: 큐브는 처음 위치로, 시각은 0 으로
            drop_count = 0  # 내 변수도 처음 값으로
            reset_needed = False
            print("reset 완료: 처음부터 다시 시작")
        height = cube.get_world_pose()[0][2]  # 관측
        if height < LOW_HEIGHT:  # 계산: 바닥 가까이 왔으면
            drop_count += 1
            print(f"시각 {world.current_time:5.2f} s | {drop_count} 번째 떨어짐 -> 다시 위로")
            cube.set_world_pose(position=START_POSITION)  # 명령: 처음 위치로 올리고 멈춘 상태로
            cube.set_linear_velocity(np.zeros(3))

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
