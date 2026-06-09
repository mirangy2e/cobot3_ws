from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import sys
import time
import math

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent

RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller_ori import PickPlaceController_ORI


# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터                                             ║
# ╚══════════════════════════════════════════════════════════════╝
USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera/m0609_camera.usd")
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
GRIPPER_JOINTS  = ["finger_joint", "right_inner_knuckle_joint"]


DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

GRIPPER_OPEN    = [0.0, 0.0]
GRIPPER_CLOSE   = [0.5, 0.5]
GRIPPER_DELTA   = [-0.5, -0.5]

FINGER_STATIC   = 1.8
FINGER_DYNAMIC  = 1.4
CUBE_STATIC     = 1.2
CUBE_DYNAMIC    = 1.0


# ╔══════════════════════════════════════════════════════════════╗
# ║  B. Controller 파라미터                                       ║
# ╚══════════════════════════════════════════════════════════════╝
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

EE_OFFSET     = np.array([0.0, 0.0, 0.2])
EVENTS_DT     = [0.008, 0.005, 0.02, 0.1, 0.0025, 0.01, 0.0025, 1, 0.008, 0.08]

# ── 직육면체 큐브 설정 ────────────────────────────────────────
CUBE_SCALE     = np.array([0.08, 0.04, 0.04])  # 직육면체 (긴 방향 X)
CUBE_INIT_POS  = np.array([0.45, 0.00, 0.04 / 2.0])
CUBE_YAW_DEG   = 30.0   # 큐브 회전 각도 (Z축 기준)

# Place 위치 (사용하지 않지만 controller.forward에 필요)
GOAL_POS       = np.array([0.55, -0.35, 0.0])

# Pick + Lift 후 중단할 event 번호
STOP_AT_EVENT  = 5   # event 5 = Place 이동 시작 전


# ============================================================
# 유틸
# ============================================================
def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(np.zeros(robot.num_dof))


def yaw_from_quat(quat_wxyz):
    """쿼터니언(w,x,y,z)에서 Z축 yaw 각도(rad) 추출"""
    w, x, y, z = quat_wxyz
    r = Rotation.from_quat([x, y, z, w])
    euler = r.as_euler('xyz')
    return euler[2]


def quat_from_yaw(yaw_deg):
    """Z축 yaw 각도(deg) → 쿼터니언(w,x,y,z)"""
    r = Rotation.from_euler('z', yaw_deg, degrees=True)
    x, y, z, w = r.as_quat()
    return np.array([w, x, y, z])


# ============================================================
# Task
# ============================================================
class M0609Task(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
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
        print(f"[1.LOAD] {USD_PATH}")

    def _discover_links(self):
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        print(f"[2.DISCOVER] EE = {self._ee_path}")
        for jn in GRIPPER_JOINTS:
            find_prim_path_by_name(ROBOT_PRIM_PATH, jn)


    def _setup_physics(self):
        stage = omni.usd.get_context().get_stage()

        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
        print("[3.PHYSICS] 완료")

    def _register_robot(self, scene):
        gripper = ParallelGripper(
            end_effector_prim_path=self._ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array(GRIPPER_OPEN),
            joint_closed_positions=np.array(GRIPPER_CLOSE),
            action_deltas=np.array(GRIPPER_DELTA),
        )
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )
        print(f"[4.REGISTER] {ROBOT_PRIM_PATH}")

    def _create_scene(self, scene):
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC, dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )

        cube_orient = quat_from_yaw(CUBE_YAW_DEG)
        self._cube = scene.add(
            DynamicCuboid(
                prim_path="/World/target_cube",
                name="target_cube",
                position=CUBE_INIT_POS,
                orientation=cube_orient,
                scale=CUBE_SCALE,
                color=np.array([0.0, 0.0, 1.0]),
                mass=0.05,
                physics_material=cube_material,
            )
        )
        print(f"[5.SCENE] 직육면체 @ {CUBE_INIT_POS}, yaw={CUBE_YAW_DEG}°")
        print(f"          scale={CUBE_SCALE}")

        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC, dynamic_friction=FINGER_DYNAMIC,
            restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if link_path:
                SingleGeometryPrim(
                    prim_path=link_path, name=f"{link_name}_geom",
                ).apply_physics_material(finger_material)

    def get_observations(self):
        cube_pos, cube_ori = self._cube.get_world_pose()
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            self._cube.name: {
                "position": cube_pos,
                "orientation": cube_ori,
            },
        }

    def pre_step(self, control_index, simulation_time):
        pass

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인 — 그리퍼 회전 + Pick + Lift + 중단                    ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    for _ in range(30):
        my_world.step(render=True)

    # ── Controller 생성 ──────────────────────────────────────
    controller = PickPlaceController_ORI(
        name="m0609_pick_place_controller",
        gripper=robot.gripper,
        robot_articulation=robot,
        end_effector_initial_height=0.30,
        events_dt=EVENTS_DT,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )

    print("\n[Pick + Lift 시작]\n")
    was_playing  = False
    pick_done    = False

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        is_playing = my_world.is_playing()

        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            pick_done = False

            # 월드 리셋 후 초기화 대기 안정화
            for _ in range(30):
                my_world.step(render=True)

        if is_playing and not pick_done:
            event = controller.get_current_event()

            # ★ event 5 도달 = Lift 완료 → 중단
            if event >= STOP_AT_EVENT:
                print(f"\n[중단] event={event} — Pick + Lift 완료!")
                print("       큐브를 정확한 방향으로 들고 있는 상태에서 정지합니다.")
                ee_pos, _ = robot.end_effector.get_world_pose()
                print(f"       EE 위치: {ee_pos}")
                pick_done = True
                continue

            obs = task.get_observations()
            cube_position  = obs["target_cube"]["position"]
            cube_orientation = obs["target_cube"]["orientation"]
            current_joints = obs["m0609_robot"]["joint_positions"]

            # ★ 핵심: 단순 라디안 실수값이 아닌 계산된 4차원 쿼터니언 배열을 매 스텝 주입합니다.
            actions = controller.forward(
                picking_position=cube_position,
                placing_position=GOAL_POS,
                current_joint_positions=current_joints,
                end_effector_offset=EE_OFFSET,
                end_effector_orientation=cube_orientation  
            )
            robot.apply_action(actions)

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()