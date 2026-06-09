from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

from pathlib import Path
import sys
import os
import time
import threading
import random

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent

RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)

from m0609_pick_place_controller import PickPlaceController


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


# ╔══════════════════════════════════════════════════════════════╗
# ║  B-COLOR. 색상 시나리오 파라미터 (★ 이번 장 핵심)               ║
# ╚══════════════════════════════════════════════════════════════╝
ROS_DOMAIN_ID    = int(os.environ.get("ROS_DOMAIN_ID", 50))
COLOR_ID_TOPIC   = "/color_id"

# 큐브 대기 위치 (카메라 시야 밖, 바닥)
HOLD_POS_BLUE  = np.array([3.0, 0.2, 0.0515 / 2.0])
HOLD_POS_GREEN = np.array([3.0, -0.2, 0.0515 / 2.0])

# 랜덤 스폰 범위 (pick 가능 영역)
SPAWN_X_RANGE  = (0.25, 0.45)
SPAWN_Y_RANGE  = (-0.25, 0.25)
SPAWN_Z        = 0.0515 / 2.0

# 색상별 Place 위치
PLACE_POSITIONS = {
    1: np.array([0.45, -0.45, 0.0]),    # 파랑 → Place 1
    2: np.array([0.45,  0.45, 0.0]),    # 초록 → Place 2
}
DEFAULT_PLACE_POS = np.array([0.55, 0.0, 0.0])

# 큐브 정보
CUBE_INFO = {
    1: {"name": "blue_cube",  "color": np.array([0.0, 0.0, 1.0]), "hold": HOLD_POS_BLUE},
    2: {"name": "green_cube", "color": np.array([0.0, 1.0, 0.0]), "hold": HOLD_POS_GREEN},
}

MARKER_COLORS = {
    1: np.array([0.0, 0.0, 1.0]),
    2: np.array([0.0, 1.0, 0.0]),
}


# ============================================================
# ROS2 Subscriber — /color_id만 구독
# ============================================================
class ColorIdSubscriber(Node):

    def __init__(self):
        super().__init__("color_id_subscriber")
        self._color_id = 0
        self._lock = threading.Lock()

        self.create_subscription(Int32, COLOR_ID_TOPIC, self._callback, 10)
        self.get_logger().info(f"{COLOR_ID_TOPIC} 대기 중...")

    def _callback(self, msg: Int32):
        with self._lock:
            self._color_id = msg.data

    def get_color_id(self):
        with self._lock:
            return self._color_id

    def clear(self):
        with self._lock:
            self._color_id = 0


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


def random_spawn_pos():
    """pick 가능 영역 내 랜덤 위치 생성"""
    x = random.uniform(*SPAWN_X_RANGE)
    y = random.uniform(*SPAWN_Y_RANGE)
    return np.array([x, y, SPAWN_Z])


# ============================================================
# Task — M0609Task (큐브 2개 + 마커 2개)
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

        # ★ 큐브 2개 (파랑 + 초록) — 시야 밖 대기 위치에 생성
        self._cubes = {}
        for cid, info in CUBE_INFO.items():
            cube = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/{info['name']}",
                    name=info["name"],
                    position=info["hold"],
                    scale=np.array([0.05, 0.05, 0.05]),
                    color=info["color"],
                    mass=0.05,
                    physics_material=cube_material,
                )
            )
            self._cubes[cid] = cube
            print(f"[5.SCENE] {info['name']} @ 대기 위치 {info['hold']}")

        # ★ Place 마커 2개 (파랑 + 초록)
        for cid, pos in PLACE_POSITIONS.items():
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/goal_marker_{cid}",
                    name=f"goal_marker_{cid}",
                    position=pos,
                    scale=np.array([0.06, 0.06, 0.001]),
                    color=MARKER_COLORS[cid],
                )
            )
            print(f"[5.SCENE] goal_{cid} @ {pos}")

        # 손가락 마찰
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

    def spawn_random_cube(self):
        """큐브 하나를 랜덤 선택 → 랜덤 위치로 이동"""
        # 모든 큐브를 대기 위치로 복귀
        for cid, cube in self._cubes.items():
            cube.set_world_pose(position=CUBE_INFO[cid]["hold"])
            cube.set_linear_velocity(np.zeros(3))
            cube.set_angular_velocity(np.zeros(3))
       
        # 랜덤 선택
        selected_id = random.choice([1, 2])
        spawn_pos = random_spawn_pos()

        cube = self._cubes[selected_id]
        cube.set_world_pose(position=spawn_pos)
        cube.set_linear_velocity(np.zeros(3))
        cube.set_angular_velocity(np.zeros(3))

        label = {1: "파랑", 2: "초록"}
        print(f"[스폰] {label[selected_id]} 큐브 → {spawn_pos}")
        return selected_id, spawn_pos

    def get_observations(self):
        positions = {}
        for cid, cube in self._cubes.items():
            pos, _ = cube.get_world_pose()
            positions[cube.name] = {"position": pos}
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            **positions,
        }

    def pre_step(self, control_index, simulation_time):
        pass

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인 — 랜덤 스폰 + 색상 감지 + 분기 Place (★ 핵심)        ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    # ── ROS2 초기화 ───────────────────────────────────────────
    rclpy.init()
    ros_node = ColorIdSubscriber()
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True
    )
    spin_thread.start()

    # ── World + Task ──────────────────────────────────────────
    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    for _ in range(30):
        my_world.step(render=True)

    # ── Controller 생성 ──────────────────────────────────────
    controller = PickPlaceController(
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

    # ── 랜덤 스폰 + 색상 감지 루프 ───────────────────────────
    was_playing   = False
    pick_started  = False
    task_done     = False
    cube_pos      = None
    spawned_id    = 0
    wait_frames   = 0

    print("\n[Play 버튼을 누르면 랜덤 큐브가 스폰됩니다]\n")

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        is_playing = my_world.is_playing()

        # Play 시작 → 리셋 + 첫 스폰
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            ros_node.clear()
            pick_started = False
            task_done    = False
            wait_frames  = 0

            # 안정화 대기
            for _ in range(30):
                my_world.step(render=True)

            # ★ 랜덤 큐브 스폰
            spawned_id, cube_pos = task.spawn_random_cube()

            # 큐브 안정화 + 카메라 인식 대기
            wait_frames = 60
            print(f"[대기] 카메라 인식 대기 중... ({wait_frames} 프레임)")

        # 카메라 인식 대기
        if is_playing and wait_frames > 0:
            wait_frames -= 1
            if wait_frames == 0:
                pick_started = True
                print("[시작] Pick & Place 시작")

        # Pick & Place 실행
        if is_playing and pick_started and not task_done:
            # /color_id로 Place 위치 결정
            color_id = ros_node.get_color_id()
            if color_id in PLACE_POSITIONS:
                place_pos = PLACE_POSITIONS[color_id]
            else:
                place_pos = DEFAULT_PLACE_POS

            obs = task.get_observations()
            cube_name = CUBE_INFO[spawned_id]["name"]
            current_cube_pos = obs[cube_name]["position"]

            actions = controller.forward(
                picking_position=current_cube_pos,
                placing_position=place_pos,
                current_joint_positions=obs["m0609_robot"]["joint_positions"],
                end_effector_offset=EE_OFFSET,
            )
            robot.apply_action(actions)

            if controller.is_done():
                label = {1: "파랑", 2: "초록", 0: "미감지"}
                print(f"[완료] color_id={color_id} ({label.get(color_id, '?')}) → {place_pos}")
                task_done = True
                ros_node.clear()
                my_world.pause()

        was_playing = is_playing

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    print(f"[ROS_DOMAIN_ID = {ROS_DOMAIN_ID}]")
    main()