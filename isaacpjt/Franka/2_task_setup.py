
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
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.objects import VisualCuboid
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

# Place 목표 위치
PLACE_POSITION  = np.array([0.35, -0.2, 0.0])

# 뷰포트 설정
PERSP_EYE    = [1.5, 1.5, 1.0]
PERSP_TARGET = [0.0, 0.0, 0.3]
TOP_CAM_PATH = "/World/Camera"


# ============================================================
# 뷰포트 설정 함수
# ============================================================
def setup_viewport():
    try:
        set_camera_view(
            eye=PERSP_EYE,
            target=PERSP_TARGET,
            camera_prim_path="/OmniverseKit_Persp"
        )
        vp1_api = vp_util.get_viewport_from_window_name("Viewport")
        if vp1_api:
            vp1_api.camera_path = "/OmniverseKit_Persp"
        print(f"  [OK] Perspective 줌인")

        ui.Workspace.show_window("Viewport 2")
        simulation_app.update()
        simulation_app.update()
        vp2_api = vp_util.get_viewport_from_window_name("Viewport 2")
        if vp2_api:
            vp2_api.camera_path = TOP_CAM_PATH
        print(f"  [OK] Viewport 2 → {TOP_CAM_PATH}")

        vp1_handle = ui.Workspace.get_window("Viewport")
        vp2_handle = ui.Workspace.get_window("Viewport 2")
        if vp1_handle and vp2_handle:
            vp2_handle.dock_in(vp1_handle, ui.DockPosition.RIGHT, 0.5)
            print("  [OK] 5:5 분할 완료")
        else:
            print("  [WARN] WindowHandle 취득 실패 → Viewport 2 별도 창으로 표시")
    except Exception as e:
        print(f"  [WARN] 뷰포트 설정 실패: {e}")


# ============================================================
# Task
# ============================================================
class FrankaTask(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()             # 1. LOAD
        self._discover_links()       # 2. DISCOVER
        self._register_robot(scene)  # 3. REGISTER
        self._setup_physics()        # 4. PHYSICS
        self._create_scene(scene)    # 5. SCENE
        print("\n  [완료] 씬 구성 성공!\n")

    # ── 1. LOAD ──────────────────────────────────────────────
    def _load_usd(self):
        print("\n" + "=" * 60)
        print("[1.LOAD] USD 로드")
        print("=" * 60)

        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)

        for _ in range(15):
            simulation_app.update()

        print(f"  [OK] {USD_PATH}")

     # ── 2. DISCOVER ──────────────────────────────────────────────
    def _discover_links(self):
        pass
    
    # ── 3. REGISTER ──────────────────────────────────────────
    def _register_robot(self, scene):
        print("\n" + "=" * 60)
        print("[4.REGISTER] Franka 등록")
        print("=" * 60)

        self._robot = scene.add(
            Franka(
                prim_path=FRANKA_PRIM,
                name="franka",
                position=FRANKA_POSITION,
                orientation=FRANKA_ORIENT,
            )
        )
        print(f"  [OK] {FRANKA_PRIM}  pos={FRANKA_POSITION}  Z=90°")
    
    # ── 4. PHYSICS ───────────────────────────────────────────
    def _setup_physics(self):
        print("\n" + "=" * 60)
        print("[3.PHYSICS] 물리 설정")
        print("=" * 60)
        # Franka: Drive 설정은 내장 클래스가 처리
        # 큐브: Rigid Body + Collider는 USD에서 설정됨
        # 필요 시 마찰력 등 추가
        print("  [INFO] Franka Drive — 내장 클래스 처리")
        print("  [INFO] 큐브 물리   — USD 설정값 사용")



    # ── 5. SCENE ─────────────────────────────────────────────
    def _create_scene(self, scene):
        print("\n" + "=" * 60)
        print("[5.SCENE] 목표 마커 생성")
        print("=" * 60)

        scene.add(
            VisualCuboid(
                prim_path="/World/goal_marker",
                name="goal_marker",
                position=PLACE_POSITION,
                scale=np.array([0.06, 0.06, 0.001]),
                color=np.array([0.0, 1.0, 0.0]),
            )
        )
        print(f"  [OK] goal_marker @ {PLACE_POSITION}")

    def get_observations(self):
        return {}

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )


# ============================================================
# 메인
# ============================================================
def main():
    my_world = World(stage_units_in_meters=1.0)
    task = FrankaTask(name="franka_task")
    my_world.add_task(task)
    my_world.reset()

    setup_viewport()

    print("[렌더 루프 시작 -- Play 버튼을 눌러 확인]\n")
    print("  확인 포인트:")
    print("  - Franka가 설계한 위치에 생성됐는가")
    print("  - 목표 마커(초록)가 PLACE_POSITION에 표시되는가")
    print("  - Viewport 2에 카메라 시점이 표시되는가\n")

    while simulation_app.is_running():
        my_world.step(render=True)

    simulation_app.close()


if __name__ == "__main__":
    main()