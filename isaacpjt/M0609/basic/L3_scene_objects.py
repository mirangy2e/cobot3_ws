"""[L3 예제] 장면 객체: 추가, 조회, 이동

실행: isaac_python L3_scene_objects.py
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
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, VisualCuboid
from isaacsim.core.utils.rotations import euler_angles_to_quat, quat_to_euler_angles

world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# 조명: 화면 위 Lighting 메뉴에서 "Default" 를 고른 것과 같다
omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")

# 1) 떨어지는 파란 큐브 (5 cm)
world.scene.add(
    DynamicCuboid(
        prim_path="/World/Blue", name="blue", position=np.array([0.0, 0.0, 1.0]),
        scale=np.array([0.05, 0.05, 0.05]), size=1.0, color=np.array([0, 0, 1.0]),
    )
)
# 2) 움직이지 않는 회색 벽 (30 x 3 x 30 cm)
world.scene.add(
    FixedCuboid(
        prim_path="/World/Wall", name="wall", position=np.array([0.5, 0.0, 0.15]),
        scale=np.array([0.3, 0.03, 0.3]), size=1.0, color=np.array([0.5, 0.5, 0.5]),
    )
)
# 3) 보이기만 하는 빨간 타겟 (파란 큐브 둘레를 돌며 벽을 그대로 통과)
red = world.scene.add(
    VisualCuboid(
        prim_path="/World/Red", name="red", position=np.array([0.5, 0.0, 0.15]),
        size=0.05, color=np.array([1.0, 0, 0]),
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
    angle = frame / 120.0  # 120 프레임(2초)에 1 rad
    # 파란 큐브 (0, 0) 둘레의 타원 (x 방향 0.5 m, y 방향 0.25 m): 한 바퀴에 한 번 벽 중심 (0.5, 0) 을 지난다
    # 자세: 도는 각도만큼 z 축 둘레로 돌린다
    yaw = np.degrees(angle)
    red.set_world_pose(
        position=np.array([0.5 * np.cos(angle), 0.25 * np.sin(angle), 0.15]),
        orientation=euler_angles_to_quat(np.array([0, 0, yaw]), degrees=True),
    )
    if frame % 60 == 0 and frame <= 300:  # 처음 300 프레임 동안 60 프레임마다 출력
        blue = world.scene.get_object("blue")  # 이름으로 찾기
        red_position, red_orientation = red.get_world_pose()
        red_angles = quat_to_euler_angles(red_orientation, degrees=True)
        print(f"{frame} 프레임 | blue {np.round(blue.get_world_pose()[0], 3)} | red {np.round(red_position, 3)}, 각도(deg) {np.round(red_angles, 1)}")

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
