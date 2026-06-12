"""
5장. Pick & Place + ROS2 (바나나)
    /cube_pose (PointStamped) 구독
    트리거 수신 → 실시간 바나나 위치로 Pick & Place
"""

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
from pxr import Usd, UsdGeom, UsdPhysics

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

from isaacsim.core.api import World
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim, SingleRigidPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers import PickPlaceController


# ============================================================
# 파라미터
# ============================================================
_THIS_DIR = Path(__file__).resolve().parent

USD_PATH        = str(_THIS_DIR / "Collected_no_franka_banana/no_franka_banana.usd")
FRANKA_PRIM     = "/World/franka"
FRANKA_POSITION = np.array([0.0, -0.5, 0.0])
FRANKA_ORIENT   = euler_angles_to_quat(np.array([0.0, 0.0, 90.0]), degrees=True)

PLACE_POSITION  = np.array([0.35, -0.2, 0.0])

CUBE_POSE_TOPIC      = "/banana/point_world"
OBJ_STATIC_FRICTION = 1.8
OBJ_DYNAMIC_FRICTION= 1.4
FINGER_STATIC_FRICTION = 1.8
FINGER_DYNAMIC_FRICTION= 1.4

EVENTS_DT = [0.008, 0.005, 0.1, 0.1, 0.05, 0.008, 0.008, 1, 0.008, 0.08]

PERSP_EYE    = [1.5, 1.5, 1.0]
PERSP_TARGET = [0.0, 0.0, 0.3]
TOP_CAM_PATH = "/World/Camera"


# ============================================================
# ROS2 Subscriber — /cube_pose 구독
# ============================================================
class CubePoseSubscriber(Node):

    def __init__(self):
        super().__init__("cube_pose_subscriber")
        self._pose = None
        self.create_subscription(PointStamped, CUBE_POSE_TOPIC, self._callback, 10)
        self.get_logger().info(f"{CUBE_POSE_TOPIC} 대기 중...")

    def _callback(self, msg: PointStamped):
        self._pose = np.array([msg.point.x, msg.point.y, msg.point.z])

    def get_pose(self):
        return self._pose.copy() if self._pose is not None else None

    def clear(self):
        self._pose = None


# ============================================================
# 뷰포트 설정
# ============================================================
def setup_viewport():
    try:
        set_camera_view(eye=PERSP_EYE, target=PERSP_TARGET,
                        camera_prim_path="/OmniverseKit_Persp")
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
        for _ in range(30):
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
        stage = omni.usd.get_context().get_stage()

        # 바나나 prim 경로 확인
        banana_xform = stage.GetPrimAtPath("/World/banana")
        if banana_xform.IsValid():
            for prim in Usd.PrimRange(banana_xform):
                print(f"  [DISCOVER] {prim.GetPath()}  [{prim.GetTypeName()}]")

        obj_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/obj_material",
            static_friction=OBJ_STATIC_FRICTION,
            dynamic_friction=OBJ_DYNAMIC_FRICTION,
            restitution=0.0,
        )
        banana_mesh = stage.GetPrimAtPath("/World/banana/_11_banana")
        if banana_mesh.IsValid():
            SingleGeometryPrim(
                prim_path="/World/banana/_11_banana", name="banana_geom",
            ).apply_physics_material(obj_material)
            print("  [OK] 바나나 마찰력 적용")
        else:
            print("  [WARN] /World/banana/_11_banana 없음 → 마찰력 생략")

        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC_FRICTION,
            dynamic_friction=FINGER_DYNAMIC_FRICTION,
            restitution=0.0,
        )
        for finger in ["panda_leftfinger", "panda_rightfinger"]:
            SingleGeometryPrim(
                prim_path=f"/World/franka/{finger}",
                name=f"{finger}_geom",
            ).apply_physics_material(finger_material)

        stage = omni.usd.get_context().get_stage()

        # # 7개 관절 Drive 강화
        gripper_joints = {
            f"{FRANKA_PRIM}/panda_hand/panda_finger_joint1",
            f"{FRANKA_PRIM}/panda_hand/panda_finger_joint2",
        }
        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(FRANKA_PRIM)):
            prim_path = str(prim.GetPath())
            if prim_path in gripper_joints:
                continue  # 그리퍼는 별도 처리
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(1e3)
                    drive.GetDampingAttr().Set(1e2)
                    drive.GetMaxForceAttr().Set(1e2)
                    drive_count += 1
        print(f"  [OK] 관절 Drive 강화: {drive_count} drives")

    def _create_scene(self, scene):
        self._cube = scene.add(
            SingleRigidPrim(prim_path="/World/banana", name="banana")
        )
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
        return {"banana": {"position": cube_pos}}

    def post_reset(self):
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )


# ============================================================
# 메인
# ============================================================
def main():
    # ROS2 초기화
    rclpy.init()
    ros_node = CubePoseSubscriber()
    # World + Task
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

    was_playing      = False
    task_done        = False
    pick_started     = False
    cube_pos_from_topic = None

    print("[Play 버튼을 누르면 /cube_pose 대기 후 Pick & Place 시작]\n")

    while simulation_app.is_running():
        my_world.step(render=True)
        rclpy.spin_once(ros_node, timeout_sec=0)
        
        is_playing = my_world.is_playing()

        # Play 시작 → 리셋
        if is_playing and not was_playing:
            my_world.reset()
            # robot.gripper.set_joint_positions(
            #     robot.gripper.joint_opened_positions
            # )
            controller.reset()
            ros_node.clear()
            pick_started = False
            cube_pos_from_topic = None
            task_done = False
            print("[대기] /banana_pose 수신 대기 중...")

        if is_playing and not task_done:
            # 트리거 대기 — /banana_pose 수신 시 Pick 시작
            if not pick_started:
                pose = ros_node.get_pose()
                if pose is not None:
                    pick_started = True
                    cube_pos_from_topic = pose.copy()
                    cube_pos_from_topic[2] = 0.025   # Z 보정 (depth는 표면, pick은 바닥 기준)
                    print(f"[트리거] /banana/point_world 수신 → Pick 위치: {cube_pos_from_topic}")
            else:
                actions = controller.forward(
                    picking_position=cube_pos_from_topic,   # 토픽 좌표 사용
                    placing_position=PLACE_POSITION,
                    current_joint_positions=robot.get_joint_positions(),
                    end_effector_offset=np.array([0.0, 0.0, 0.02]),
                )
                robot.apply_action(actions)

                if controller.is_done():
                    print(f"[완료] Pick & Place 완료 → {PLACE_POSITION}")
                    task_done = True
                    my_world.pause()

        was_playing = is_playing

    ros_node.destroy_node()
    rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()