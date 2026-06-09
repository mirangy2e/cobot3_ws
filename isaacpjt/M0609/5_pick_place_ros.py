
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ── rclpy — Bridge 활성화 후에만 import 가능 ─────────────────
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from pathlib import Path
import sys
import os
import time
import threading

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
# ║  A. Task 파라미터 (이전 장과 동일)                              ║
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
# ║  B. Controller 파라미터 (이전 장과 동일)                        ║
# ╚══════════════════════════════════════════════════════════════╝

# ── B-1. 인프라 파일 경로 ─────────────────────────────────────
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ── B-2. Pick & Place 동작 파라미터 ───────────────────────────
CUBE_INIT_POS = np.array([0.30, 0.4, 0.0515 / 2.0])
GOAL_POS      = np.array([0.55, -0.35, 0.0])
EE_OFFSET     = np.array([0.0, 0.0, 0.2])

# ── B-3. events_dt ────────────────────────────────────────────
EVENTS_DT = [0.008, 0.005, 0.02, 0.1, 0.0025, 0.01, 0.0025, 1, 0.008, 0.08]


# ╔══════════════════════════════════════════════════════════════╗
# ║  B-ROS. ROS2 파라미터 (★ 이번 장에서 새로 추가)                ║
# ╚══════════════════════════════════════════════════════════════╝
ROS_DOMAIN_ID  = int(os.environ.get("ROS_DOMAIN_ID", 50))
ROS_TOPIC_NAME = "/cube_pose"


# ============================================================
# ROS2 Subscriber (★ 이번 장 핵심)
# ============================================================
class CubePoseSubscriber(Node):
    """
    /cube_pose (PointStamped) 토픽을 수신한다.
    - rclpy.spin()은 별도 스레드에서 실행
    - 공유 변수는 threading.Lock()으로 보호
    - 모션 실행 중(_is_busy)에는 새 좌표 무시
    """

    def __init__(self):
        super().__init__("cube_pose_subscriber")
        self._position = None
        self._lock = threading.Lock()
        self._is_busy = False

        self.create_subscription(PointStamped, ROS_TOPIC_NAME, self._callback, 10)
        self.get_logger().info(f"{ROS_TOPIC_NAME} 대기 중...")

    def _callback(self, msg: PointStamped):
        with self._lock:
            if self._is_busy:
                return
            self._position = np.array([msg.point.x, msg.point.y, msg.point.z])
            self.get_logger().info(f"[수신] 큐브 위치: {self._position}")

    def get_position(self):
        with self._lock:
            return self._position.copy() if self._position is not None else None

    def clear_position(self):
        with self._lock:
            self._position = None

    def set_busy(self, busy: bool):
        with self._lock:
            self._is_busy = busy


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


# ============================================================
# Task — 이전 장과 동일 (M0609Task)
# ============================================================
class M0609Task(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._task_achieved = False

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        print("\n  [완료] 씬 구성 성공!\n")

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

    def _discover_links(self):
        print("\n" + "=" * 60)
        print("[2.DISCOVER] 링크 경로 탐색")
        print("=" * 60)
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        print(f"  EE ({EE_LINK_NAME}) = {self._ee_path}")
        for jn in GRIPPER_JOINTS:
            print(f"  {jn:<35} = {find_prim_path_by_name(ROBOT_PRIM_PATH, jn)}")


    def _setup_physics(self):
        print("\n" + "=" * 60)
        print("[3.PHYSICS] 물리 설정")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()

        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    drive_count += 1
        print(f"  [OK] drive updated: {drive_count}")

    def _register_robot(self, scene):
        print("\n" + "=" * 60)
        print("[4.REGISTER] 로봇 등록")
        print("=" * 60)
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
        print(f"  [OK] SingleManipulator: {ROBOT_PRIM_PATH}")

    def _create_scene(self, scene):
        print("\n" + "=" * 60)
        print("[5.SCENE] 작업 환경 구성")
        print("=" * 60)
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )
        self._cube = scene.add(
            DynamicCuboid(
                prim_path="/World/target_cube",
                name="target_cube",
                position=CUBE_INIT_POS,
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([0.0, 0.0, 1.0]),
                mass=0.05,
                physics_material=cube_material,
            )
        )
        scene.add(
            VisualCuboid(
                prim_path="/World/goal_marker",
                name="goal_marker",
                position=GOAL_POS,
                scale=np.array([0.06, 0.06, 0.001]),
                color=np.array([0.0, 1.0, 0.0]),
            )
        )
        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC,
            dynamic_friction=FINGER_DYNAMIC,
            restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if link_path:
                SingleGeometryPrim(
                    prim_path=link_path,
                    name=f"{link_name}_geom",
                ).apply_physics_material(finger_material)

    def get_observations(self):
        cube_pos, _ = self._cube.get_world_pose()
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            self._cube.name: {
                "position": cube_pos,
                "goal_position": GOAL_POS,
            },
        }

    def pre_step(self, control_index, simulation_time):
        cube_pos, _ = self._cube.get_world_pose()
        if not self._task_achieved and np.mean(np.abs(GOAL_POS - cube_pos)) < 0.02:
            self._cube.get_applied_visual_material().set_color(np.array([0.0, 1.0, 0.0]))
            self._task_achieved = True

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )
        self._cube.get_applied_visual_material().set_color(np.array([0.0, 0.0, 1.0]))
        self._task_achieved = False


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인 — ROS2 + Controller 실행 (★ 이번 장 핵심)            ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    # ── C-ROS1. ROS2 초기화 + 별도 스레드 ────────────────────
    rclpy.init()
    ros_node = CubePoseSubscriber()

    spin_thread = threading.Thread(
        target=rclpy.spin, args=(ros_node,), daemon=True
    )
    spin_thread.start()

    # ── C-1. World + Task ─────────────────────────────────────
    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    for _ in range(30):
        my_world.step(render=True)

    # ── C-2. Controller 생성 ──────────────────────────────────
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

    print(f"\n[대기 중] {ROS_TOPIC_NAME} 수신을 기다립니다...\n")

    # ── C-ROS2. ROS2 수신 + Controller 실행 루프 ─────────────
    was_playing = False
    task_done   = False
    cube_placed = False

    while simulation_app.is_running():
        my_world.step(render=True)
        time.sleep(0.01)
        is_playing = my_world.is_playing()

        # Play 시작 감지 → 리셋
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            ros_node.clear_position()
            ros_node.set_busy(False)
            task_done   = False
            cube_placed = False
            print(f"[리셋] {ROS_TOPIC_NAME} 수신 대기 중...")

        if is_playing and not task_done:
            # /cube_pose 수신 대기
            cube_pos = ros_node.get_position()

            if cube_pos is None:
                was_playing = is_playing
                continue

            # 처음 수신 시 한 번만 큐브 이동
            if not cube_placed:
                task._cube.set_world_pose(position=cube_pos)
                task._cube.set_linear_velocity(np.zeros(3))
                task._cube.set_angular_velocity(np.zeros(3))
                cube_placed = True
                print(f"[픽업 시작] 큐브 위치: {cube_pos}")

            ros_node.set_busy(True)

            # Controller 실행 (7장과 동일)
            obs = task.get_observations()
            actions = controller.forward(
                picking_position=cube_pos,
                placing_position=GOAL_POS,
                current_joint_positions=obs["m0609_robot"]["joint_positions"],
                end_effector_offset=EE_OFFSET,
            )
            robot.apply_action(actions)

            if controller.is_done():
                print("[완료] Pick & Place 성공!")
                task_done = True
                ros_node.set_busy(False)
                ros_node.clear_position()
                my_world.pause()

        was_playing = is_playing

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    print(f"[ROS_DOMAIN_ID = {ROS_DOMAIN_ID}]")
    main()