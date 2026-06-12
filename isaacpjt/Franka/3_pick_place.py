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
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController


# ============================================================
# 파라미터
# ============================================================
_THIS_DIR = Path(__file__).resolve().parent

USD_PATH        = str(_THIS_DIR / "Collected_no_franka/no_franka.usd")
FRANKA_PRIM     = "/World/franka"
FRANKA_POSITION = np.array([0.0, -0.5, 0.0])
FRANKA_ORIENT   = euler_angles_to_quat(np.array([0.0, 0.0, 90.0]), degrees=True)

PLACE_POSITION  = np.array([0.35, -0.2, 0.0])

# events_dt 기본값: [0.008, 0.005, 0.1, 0.1, 0.0025, 0.001, 0.0025, 1, 0.008, 0.08]
EVENTS_DT    = [0.008, 0.005, 0.1, 0.1, 0.0025, 0.001, 0.0025, 1, 0.008, 0.08]

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

        ui.Workspace.show_window("Viewport 2")
        simulation_app.update()
        simulation_app.update()
        vp2_api = vp_util.get_viewport_from_window_name("Viewport 2")
        if vp2_api:
            vp2_api.camera_path = TOP_CAM_PATH

        vp1_handle = ui.Workspace.get_window("Viewport")
        vp2_handle = ui.Workspace.get_window("Viewport 2")
        if vp1_handle and vp2_handle:
            vp2_handle.dock_in(vp1_handle, ui.DockPosition.RIGHT, 0.5)
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
        self._load_usd()
        self._register_robot(scene)
        self._setup_physics()
        self._create_scene(scene)
        print("\n  [완료] 씬 구성 성공!\n")

    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

    def _register_robot(self, scene):
        self._robot = scene.add(
            Franka(
                prim_path=FRANKA_PRIM,
                name="franka",
                position=FRANKA_POSITION,
                orientation=FRANKA_ORIENT,
            )
        )

    def _setup_physics(self):
        pass

    def _create_scene(self, scene):
        # 큐브 래핑 (실시간 위치 읽기)
        self._cube = scene.add(
            SingleRigidPrim(prim_path="/World/Cube", name="cube")
        )
        # 목표 마커
        scene.add(
            VisualCuboid(
                prim_path="/World/goal_marker",
                name="goal_marker",
                position=PLACE_POSITION,
                scale=np.array([0.06, 0.06, 0.001]),
                color=np.array([0.0, 1.0, 0.0]),
            )
        )

    def get_observations(self):
        cube_pos, _ = self._cube.get_world_pose()
        return {"cube": {"position": cube_pos}}

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

    robot = my_world.scene.get_object("franka")
    controller = PickPlaceController(
        name="pick_place_controller",
        gripper=robot.gripper,
        robot_articulation=robot,
        events_dt=EVENTS_DT,
    )

    was_playing = False
    task_done   = False

    print("[Play 버튼을 누르면 Pick & Place가 시작됩니다]\n")

    while simulation_app.is_running():
        my_world.step(render=True)
        is_playing = my_world.is_playing()

        if is_playing and not was_playing:
            my_world.reset()
            robot.gripper.set_joint_positions(
                robot.gripper.joint_opened_positions
            )
            controller.reset()
            task_done = False

        if is_playing and not task_done:
            obs      = my_world.get_observations()
            cube_pos = obs["cube"]["position"]

            actions = controller.forward(
                picking_position=cube_pos,
                placing_position=PLACE_POSITION,
                current_joint_positions=robot.get_joint_positions(),
                end_effector_offset=np.array([0.0, 0.0, 0.02]),
            )
            robot.apply_action(actions)

            if controller.is_done():
                print("[완료] Pick & Place 완료")
                task_done = True

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()