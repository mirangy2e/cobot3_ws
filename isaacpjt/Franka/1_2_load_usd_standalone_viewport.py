
# ── SimulationApp (반드시 모든 import보다 먼저) ───────────────
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ── import ────────────────────────────────────────────────────
from pathlib import Path
import time
import numpy as np

import omni.usd
import omni.ui as ui
import omni.kit.viewport.utility as vp_util
from pxr import Usd, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.examples.franka import Franka


# ============================================================
# 파라미터
# ============================================================
_THIS_DIR = Path(__file__).resolve().parent

USD_PATH        = str(_THIS_DIR / "Collected_no_franka/no_franka.usd")
FRANKA_PRIM     = "/World/franka"
FRANKA_POSITION = np.array([0.0, -0.5, 0.0])
FRANKA_ORIENT   = euler_angles_to_quat(np.array([0.0, 0.0, 90.0]), degrees=True)

# 뷰포트 설정
PERSP_EYE    = [1.5, 1.5, 1.0]   # GUI에서 확인한 값
PERSP_TARGET = [0.0, 0.0, 0.3]   # 로봇 베이스 근처
TOP_CAM_PATH = "/World/Camera"   # Static Camera prim 경로


# ============================================================
# 뷰포트 설정 함수
# ============================================================
def setup_viewport():
    try:
        # Viewport 1: Perspective 줌인
        set_camera_view(
            eye=PERSP_EYE,
            target=PERSP_TARGET,
            camera_prim_path="/OmniverseKit_Persp"
        )
        vp1_api = vp_util.get_viewport_from_window_name("Viewport")
        if vp1_api:
            vp1_api.camera_path = "/OmniverseKit_Persp"
        print(f"[OK] Perspective 줌인  eye={PERSP_EYE}")

        # Viewport 2: Static Camera (5:5 분할)
        #   ┌─────────────────┬─────────────────┐
        #   │  Viewport 1     │  Viewport 2     │
        #   │  Perspective    │  Static Camera  │
        #   │  (로봇 전체)    │  (큐브 하향)    │
        #   └─────────────────┴─────────────────┘
        ui.Workspace.show_window("Viewport 2")
        simulation_app.update()
        simulation_app.update()

        vp2_api = vp_util.get_viewport_from_window_name("Viewport 2")
        if vp2_api:
            vp2_api.camera_path = TOP_CAM_PATH
        print(f"[OK] Viewport 2 → {TOP_CAM_PATH}")

        vp1_handle = ui.Workspace.get_window("Viewport")
        vp2_handle = ui.Workspace.get_window("Viewport 2")
        if vp1_handle and vp2_handle:
            vp2_handle.dock_in(vp1_handle, ui.DockPosition.RIGHT, 0.5)
            print("[OK] 5:5 분할 완료")
        else:
            print("[WARN] WindowHandle 취득 실패 → Viewport 2 별도 창으로 표시")

    except Exception as e:
        print(f"[WARN] 뷰포트 설정 실패: {e}")


# ============================================================
# 메인
# ============================================================
my_world = World(stage_units_in_meters=1.0)

# 1. 환경 USD 로드
stage = omni.usd.get_context().get_stage()
world_prim = stage.GetPrimAtPath("/World")
if not world_prim.IsValid():
    world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
world_prim.GetReferences().AddReference(USD_PATH)

# 2. Franka 생성
franka = my_world.scene.add(
    Franka(
        prim_path=FRANKA_PRIM,
        name="franka",
        position=FRANKA_POSITION,
        orientation=FRANKA_ORIENT,
    )
)

# 3. reset
my_world.reset()

# 4. 뷰포트 설정
setup_viewport()

print("\n[렌더 루프 시작 -- Play 버튼을 눌러 확인]")
print("  확인 포인트:")
print("  - Viewport 1: Franka 전체 시점")
print("  - Viewport 2: Static Camera 시점 (큐브 하향)\n")

while simulation_app.is_running():
    my_world.step(render=True)
    time.sleep(0.016)

simulation_app.close()