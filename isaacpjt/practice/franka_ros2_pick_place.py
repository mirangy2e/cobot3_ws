# franka_ros2_pick_place.py  (v2 — clean rebuild)
#
# 커스텀 USD 없이 Isaac Sim 내장 에셋으로 씬을 직접 구성한다.
#   - Franka       : Franka() 클래스로 직접 생성 (삭제/재생성 불필요)
#   - Simple Room  : Nucleus URL 참조
#   - 바나나        : YCB 에셋 + 물리 코드로 추가
#   - KLT 바구니   : Nucleus URL 참조
#
# 실행:
#   ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/franka_ros2_pick_place.py

import os, sys
os.environ["ROS_DOMAIN_ID"] = "50"

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.usd
from pxr import UsdPhysics, UsdShade, UsdGeom, Gf, Sdf, PhysxSchema
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils import extensions
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import RMPFlowController

# ── ROS2 bridge ───────────────────────────────────────────────────────
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
simulation_app.update()

_RCLPY_PATH = "/home/rokey/isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy"
if _RCLPY_PATH not in sys.path:
    sys.path.insert(0, _RCLPY_PATH)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

# ══════════════════════════════════════════════════════════════════════
# Nucleus 에셋 URL  (Isaac Sim 5.1 공식 경로)
# ══════════════════════════════════════════════════════════════════════
_BASE           = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1"
SIMPLE_ROOM_URL = f"{_BASE}/Isaac/Environments/Simple_Room/simple_room.usd"
BANANA_URL      = f"{_BASE}/Isaac/Props/YCB/Axis_Aligned/011_banana.usd"
KLT_BIN_URL     = f"{_BASE}/Isaac/Props/KLT_Bin/small_KLT_visual.usd"

# ══════════════════════════════════════════════════════════════════════
# 파라미터
# ══════════════════════════════════════════════════════════════════════
ROBOT_PRIM  = "/World/franka"
BANANA_PRIM = "/World/banana"
TABLE_PRIM  = "/World/banana_table"
KLT_PRIM    = "/World/klt_bin"

PHYSICS_DT   = 1.0 / 200.0
RENDERING_DT = 20.0 / 200.0

SPEED_SCALE      = 1.0
LIFT_SPEED_SCALE = 0.3
HAND_TO_RG_Z     = 0.10

# 바나나 위치 (실측값)
BANANA_XY      = np.array([-0.016, 0.057])
BANANA_Z_FIXED = 0.116   # 바나나 중심 world z

# 높이 (panda_hand 기준)
PRE_PICK_Z  = 0.55
PICK_Z      = 0.135   # stall≈z0.225, 손끝≈z0.125
LIFT_Z      = 0.50
PRE_PLACE_Z = 0.50
PLACE_Z     = 0.30
RETREAT_Z   = 0.50

BASKET_XY = np.array([0.394, -0.107])

REACH_THRESH       = 0.04
STATE_TIMEOUT      = 2000
STALL_FRAMES       = 150
STALL_DELTA        = 0.003
OPEN_GRIPPER_WAIT  = 80
CLOSE_GRIPPER_WAIT = 500

GRIPPER_JOINT_INDICES = np.array([7, 8])
GRIPPER_OPEN   = np.array([0.04, 0.04])
GRIPPER_CLOSED = np.array([0.01, 0.01])   # 0.00 → 0.01: 바나나 두께 맞춤 (0.00은 관통 시도)

# ══════════════════════════════════════════════════════════════════════
# ROS2 구독 노드
# ══════════════════════════════════════════════════════════════════════
class BananaPosSubscriber(Node):
    def __init__(self):
        super().__init__("franka_pick_place_node")
        self.banana_pos = None
        self.new_pos    = False
        self.create_subscription(PointStamped, "/banana/point_world", self._cb, 10)
        self.get_logger().info("초기화 완료 — /banana/point_world 대기 중")

    def _cb(self, msg):
        pos = np.array([msg.point.x, msg.point.y, msg.point.z])
        if not self.new_pos:
            self.get_logger().info(f"바나나 수신: {np.round(pos, 3)}")
        self.banana_pos = pos
        self.new_pos    = True

rclpy.init()
ros_node = BananaPosSubscriber()

# ══════════════════════════════════════════════════════════════════════
# World 생성
# ══════════════════════════════════════════════════════════════════════
my_world = World(
    stage_units_in_meters=1.0,
    physics_dt=PHYSICS_DT,
    rendering_dt=RENDERING_DT,
)
stage = omni.usd.get_context().get_stage()

# ══════════════════════════════════════════════════════════════════════
# 씬 구성 — 코드로 직접 조립 (커스텀 USD 없음)
# ══════════════════════════════════════════════════════════════════════

# ── 1. 배경: Simple Room (바닥+벽 충돌 포함) ─────────────────────────
room = stage.DefinePrim("/World/SimpleRoom", "Xform")
room.GetReferences().AddReference(SIMPLE_ROOM_URL)

# ── 2. 기본 지면 (SimpleRoom 바닥 백업) ─────────────────────────────
my_world.scene.add_default_ground_plane()

# ── 3. Franka 로봇  ★핵심: 직접 생성 → 삭제/재생성 불필요 ────────────
my_franka = my_world.scene.add(
    Franka(
        prim_path=ROBOT_PRIM,
        name="my_franka",
        end_effector_prim_name="panda_hand",
        position=np.array([0.0, -0.5, 0.0]),
        orientation=np.array([0.7071067811865476, 0.0, 0.0, 0.7071067811865475]),
    )
)

# ── 4. 바나나 받침대 (정적 박스) ─────────────────────────────────────
#    중심 z=0.04m → 상면 z=0.08m → 바나나 중심 z≈0.116m (반경 3.6cm)
table = my_world.scene.add(
    FixedCuboid(
        prim_path=TABLE_PRIM,
        name="banana_table",
        position=np.array([BANANA_XY[0], BANANA_XY[1], 0.04]),
        scale=np.array([0.30, 0.30, 0.08]),
    )
)

# ── 5. 바나나 (RigidBody + YCB 에셋) ─────────────────────────────────
banana_ref = stage.DefinePrim(BANANA_PRIM, "Xform")
banana_ref.GetReferences().AddReference(BANANA_URL)
UsdGeom.XformCommonAPI(banana_ref).SetTranslate(
    Gf.Vec3d(BANANA_XY[0], BANANA_XY[1], BANANA_Z_FIXED)
)
UsdPhysics.RigidBodyAPI.Apply(banana_ref)                # 물리 강체
banana_sim = my_world.scene.add(SingleRigidPrim(prim_path=BANANA_PRIM, name="banana"))

# ── 6. KLT 바구니 (정적) ─────────────────────────────────────────────
klt_ref = stage.DefinePrim(KLT_PRIM, "Xform")
klt_ref.GetReferences().AddReference(KLT_BIN_URL)
UsdGeom.XformCommonAPI(klt_ref).SetTranslate(
    Gf.Vec3d(BASKET_XY[0], BASKET_XY[1], 0.0)
)

# 참조 에셋 로드 대기 (원격 S3 에셋 다운로드 시간 확보)
for _ in range(8):
    simulation_app.update()

my_world.reset()

# 리셋 후 추가 대기 (USD 계층 확정)
for _ in range(5):
    simulation_app.update()

# ══════════════════════════════════════════════════════════════════════
# 물리 설정  (reset 후 / 시뮬 시작 전)
# ══════════════════════════════════════════════════════════════════════
_banana_physics_done  = False
_gripper_drive_done   = False


def _apply_physics_to_children(prim, mat_path):
    """바나나 하위 Mesh/Xform 모두에 충돌+마찰+contactOffset 적용 (재귀)."""
    for child in prim.GetAllChildren():
        if child.GetTypeName() in ("Mesh", "Xform"):
            # CollisionAPI + convexDecomposition
            UsdPhysics.CollisionAPI.Apply(child)
            mesh_col = UsdPhysics.MeshCollisionAPI.Apply(child)
            mesh_col.GetApproximationAttr().Set("convexDecomposition")
            # PhysX contactOffset=0.002 → 그리퍼와 2mm 간격에서도 접촉 인식
            physx_col = PhysxSchema.PhysxCollisionAPI.Apply(child)
            physx_col.GetContactOffsetAttr().Set(0.002)
            physx_col.GetRestOffsetAttr().Set(0.0)
            # Material 바인딩 (relationship 방식 — articulation 링크 안전)
            child.GetRelationship("physics:material:binding").AddTarget(mat_path)
        _apply_physics_to_children(child, mat_path)


def setup_banana_physics():
    """바나나 RigidBody + 하위 메시 충돌+마찰 설정.
    step05 방식 적용: PhysxCollisionAPI contactOffset + 재귀 traversal.
    """
    global _banana_physics_done
    if _banana_physics_done:
        return True
    banana_prim = stage.GetPrimAtPath(BANANA_PRIM)
    if not banana_prim.IsValid():
        print(f"⚠️  바나나 Prim 없음: {BANANA_PRIM}  (다음 프레임 재시도)")
        return False
    # 하위에 메시가 하나라도 있는지 확인
    children = list(banana_prim.GetAllChildren())
    if not children:
        print(f"⚠️  바나나 하위 Prim 없음 (아직 로드 중)  (다음 프레임 재시도)")
        return False
    # PhysicsMaterial 생성
    mat_path = f"{BANANA_PRIM}/PhysicsMaterial"
    mat_prim = stage.DefinePrim(mat_path, "Material")
    phys_mat = UsdPhysics.MaterialAPI.Apply(mat_prim)
    phys_mat.CreateStaticFrictionAttr().Set(1.5)
    phys_mat.CreateDynamicFrictionAttr().Set(1.0)
    phys_mat.CreateRestitutionAttr().Set(0.0)
    # 모든 하위 메시에 적용
    _apply_physics_to_children(banana_prim, mat_path)
    print("✅ 바나나 물리 설정 완료  (convexDecomposition, contactOffset=0.002, static=1.5)")
    _banana_physics_done = True
    return True


def setup_gripper_drive():
    """그리퍼 Drive 설정 — step05 방식: Stiffness=1e6 (파지력 극대화)."""
    global _gripper_drive_done
    if _gripper_drive_done:
        return True
    ok = False
    for jn in ["panda_finger_joint1", "panda_finger_joint2"]:
        p = stage.GetPrimAtPath(f"{ROBOT_PRIM}/panda_hand/{jn}")
        if not p.IsValid():
            print(f"⚠️  {jn} 없음")
            continue
        # Get 먼저 시도, 없으면 Apply
        d = UsdPhysics.DriveAPI.Get(p, "linear")
        if not d:
            d = UsdPhysics.DriveAPI.Apply(p, "linear")
        d.GetStiffnessAttr().Set(1e6)   # 10000 → 1,000,000 (step05와 동일)
        d.GetDampingAttr().Set(1e4)     # 1000  → 10,000
        ok = True
    if ok:
        print("✅ 그리퍼 Drive 완료  (Stiffness=1e6, Damping=1e4 — step05 방식)")
        _gripper_drive_done = True
    return ok


setup_banana_physics()
setup_gripper_drive()

# ── RMPFlow 컨트롤러 ─────────────────────────────────────────────────
my_controller = RMPFlowController(
    name="rmpflow_controller",
    robot_articulation=my_franka,
    physics_dt=PHYSICS_DT,
)
articulation_controller = my_franka.get_articulation_controller()

# ══════════════════════════════════════════════════════════════════════
# 헬퍼 함수
# ══════════════════════════════════════════════════════════════════════
_gripper_target = GRIPPER_OPEN.copy()


def set_gripper(pos):
    global _gripper_target
    _gripper_target = pos.copy()


def apply_arm(target_ee_pos, speed_scale=SPEED_SCALE):
    action = my_controller.forward(
        target_end_effector_position=target_ee_pos,
        target_end_effector_orientation=None,
    )
    if action is not None:
        if action.joint_velocities is not None:
            action.joint_velocities *= speed_scale
        articulation_controller.apply_action(action)


def apply_gripper():
    my_franka.apply_action(ArticulationAction(
        joint_positions=_gripper_target,
        joint_indices=GRIPPER_JOINT_INDICES,
    ))


def apply_step(target_ee_pos=None, speed_scale=SPEED_SCALE):
    if target_ee_pos is not None:
        apply_arm(target_ee_pos, speed_scale)
    apply_gripper()


def get_hand_pos():
    pos, _ = my_franka.end_effector.get_world_pose()
    return pos


_stall_pos   = None
_stall_count = 0


def is_stalled():
    global _stall_pos, _stall_count
    try:
        hand = get_hand_pos()
        if _stall_pos is None or np.linalg.norm(hand - _stall_pos) > STALL_DELTA:
            _stall_pos = hand.copy()
            _stall_count = 0
            return False
        _stall_count += 1
        return _stall_count >= STALL_FRAMES
    except Exception:
        return False


def is_reached(target):
    try:
        return float(np.linalg.norm(get_hand_pos() - target)) < REACH_THRESH
    except Exception:
        return False


def reinit_rmpflow():
    robot_pos, robot_ori = my_franka.get_world_pose()
    my_controller.rmp_flow.set_robot_base_pose(robot_pos, robot_ori)


# ══════════════════════════════════════════════════════════════════════
# State machine
# ══════════════════════════════════════════════════════════════════════
STATES = [
    "IDLE",               # /banana/point_world 수신 대기
    "OPEN_GRIPPER",       # 그리퍼 열기
    "PRE_PICK",           # 바나나 위 접근
    "PICK",               # 바나나 높이로 하강
    "CLOSE_GRIPPER",      # 그리퍼 닫기 + 대기
    "LIFT",               # 천천히 들어올리기
    "PRE_PLACE",          # 바구니 위 접근
    "PLACE",              # 바구니 안으로 하강
    "OPEN_GRIPPER_PLACE", # 그리퍼 열기
    "RETREAT",            # 물러나기 → IDLE
]

state       = "IDLE"
wait_count  = 0
target_pos  = None
pick_target = None


def next_state():
    global state, wait_count, target_pos, _stall_pos, _stall_count
    state        = STATES[(STATES.index(state) + 1) % len(STATES)]
    wait_count   = 0
    target_pos   = None
    _stall_pos   = None
    _stall_count = 0
    print(f"  → [{state}]")


def go_idle():
    global state, wait_count, target_pos, pick_target
    state            = "IDLE"
    wait_count       = 0
    target_pos       = None
    pick_target      = None
    ros_node.new_pos = False
    print("\n=== 완료! 다음 /banana/point_world 대기 중... ===\n")


def reset_sm():
    global state, wait_count, target_pos, pick_target, _stall_pos, _stall_count
    state        = "IDLE"
    wait_count   = 0
    target_pos   = None
    pick_target  = None
    _stall_pos   = None
    _stall_count = 0
    set_gripper(GRIPPER_OPEN)
    reinit_rmpflow()


# ══════════════════════════════════════════════════════════════════════
# 메인 루프
# ══════════════════════════════════════════════════════════════════════
reset_needed = False

print("=" * 60)
print("  Franka ROS2 Pick & Place  (v2 — clean scene)")
print(f"  ROS_DOMAIN_ID = {os.environ['ROS_DOMAIN_ID']}")
print(f"  바나나 위치   = [{BANANA_XY[0]}, {BANANA_XY[1]}, {BANANA_Z_FIXED}]")
print(f"  바구니 위치   = {BASKET_XY}")
print("=" * 60)
print("  Play 버튼을 눌러 시작하세요.\n")

while simulation_app.is_running():
    my_world.step(render=True)
    rclpy.spin_once(ros_node, timeout_sec=0)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():

        if reset_needed:
            my_world.reset()
            reset_sm()
            reset_needed = False
            print("[Reset] 재시작\n")

        if my_world.current_time_step_index == 0:
            reset_sm()

        # 초기화 지연 재시도 (원격 에셋 로드가 늦을 경우)
        if not _banana_physics_done:
            setup_banana_physics()
        if not _gripper_drive_done:
            setup_gripper_drive()

        # ── IDLE ────────────────────────────────────────────────────
        if state == "IDLE":
            apply_step()
            if ros_node.new_pos and ros_node.banana_pos is not None:
                ros_node.new_pos = False
                pick_target = ros_node.banana_pos.copy()
                print(f"\n[IDLE] 바나나 수신: {np.round(pick_target, 3)}")
                next_state()

        # ── OPEN_GRIPPER ─────────────────────────────────────────────
        elif state == "OPEN_GRIPPER":
            set_gripper(GRIPPER_OPEN)
            apply_step()
            wait_count += 1
            if wait_count == 1:
                print("[OPEN_GRIPPER] 그리퍼 열기")
            if wait_count >= OPEN_GRIPPER_WAIT:
                next_state()

        # ── PRE_PICK ─────────────────────────────────────────────────
        elif state == "PRE_PICK":
            if target_pos is None:
                target_pos = np.array([pick_target[0], pick_target[1], PRE_PICK_Z])
                print(f"[PRE_PICK]  panda_hand 목표: {np.round(target_pos, 3)}")
            apply_step(target_pos)
            wait_count += 1
            if wait_count % 100 == 0:
                print(f"           현재: {np.round(get_hand_pos(), 3)}  (f={wait_count})")
            if is_reached(target_pos) or is_stalled() or wait_count >= STATE_TIMEOUT:
                next_state()

        # ── PICK ─────────────────────────────────────────────────────
        elif state == "PICK":
            if target_pos is None:
                target_pos = np.array([pick_target[0], pick_target[1], PICK_Z])
                print(f"[PICK]      panda_hand 목표: {np.round(target_pos, 3)}")
                print(f"            stall≈z{PICK_Z+0.09:.3f}  손끝≈z{PICK_Z+0.09-HAND_TO_RG_Z:.3f}")
            apply_step(target_pos)
            wait_count += 1
            if wait_count % 100 == 0:
                print(f"           현재: {np.round(get_hand_pos(), 3)}  (f={wait_count})")
            if is_reached(target_pos) or is_stalled() or wait_count >= STATE_TIMEOUT:
                next_state()

        # ── CLOSE_GRIPPER ────────────────────────────────────────────
        elif state == "CLOSE_GRIPPER":
            set_gripper(GRIPPER_CLOSED)
            # ★ apply_arm 호출 없음 — 팔 정지 상태에서 그리퍼만 닫음 (step05 방식)
            apply_gripper()
            wait_count += 1
            if wait_count == 1:
                print(f"[CLOSE_GRIPPER] 그리퍼 닫기 — 팔 정지, {CLOSE_GRIPPER_WAIT}f 대기")
            if wait_count % 100 == 0:
                print(f"   [{wait_count:4d}f] 닫는 중... hand={np.round(get_hand_pos(), 3)}")
            if wait_count >= CLOSE_GRIPPER_WAIT:
                print(f"[CLOSE_GRIPPER] 완료  hand={np.round(get_hand_pos(), 3)}")
                next_state()

        # ── LIFT ─────────────────────────────────────────────────────
        elif state == "LIFT":
            if target_pos is None:
                target_pos = np.array([pick_target[0], pick_target[1], LIFT_Z])
                print(f"[LIFT]      목표: {np.round(target_pos, 3)}  (속도 {int(LIFT_SPEED_SCALE*100)}%)")
            apply_step(target_pos, LIFT_SPEED_SCALE)
            wait_count += 1
            if wait_count % 100 == 0:
                print(f"           현재: {np.round(get_hand_pos(), 3)}  (f={wait_count})")
            if is_reached(target_pos) or is_stalled() or wait_count >= STATE_TIMEOUT:
                next_state()

        # ── PRE_PLACE ────────────────────────────────────────────────
        elif state == "PRE_PLACE":
            if target_pos is None:
                target_pos = np.array([BASKET_XY[0], BASKET_XY[1], PRE_PLACE_Z])
                print(f"[PRE_PLACE] 목표: {np.round(target_pos, 3)}")
            apply_step(target_pos)
            wait_count += 1
            if wait_count % 100 == 0:
                print(f"           현재: {np.round(get_hand_pos(), 3)}  (f={wait_count})")
            if is_reached(target_pos) or is_stalled() or wait_count >= STATE_TIMEOUT:
                next_state()

        # ── PLACE ────────────────────────────────────────────────────
        elif state == "PLACE":
            if target_pos is None:
                target_pos = np.array([BASKET_XY[0], BASKET_XY[1], PLACE_Z])
                print(f"[PLACE]     목표: {np.round(target_pos, 3)}")
            apply_step(target_pos)
            wait_count += 1
            if wait_count % 100 == 0:
                print(f"           현재: {np.round(get_hand_pos(), 3)}  (f={wait_count})")
            if is_reached(target_pos) or is_stalled() or wait_count >= STATE_TIMEOUT:
                next_state()

        # ── OPEN_GRIPPER_PLACE ───────────────────────────────────────
        elif state == "OPEN_GRIPPER_PLACE":
            set_gripper(GRIPPER_OPEN)
            apply_step()
            wait_count += 1
            if wait_count == 1:
                print("[OPEN_GRIPPER_PLACE] 그리퍼 열기 (내려놓기)")
            if wait_count >= OPEN_GRIPPER_WAIT:
                next_state()

        # ── RETREAT ──────────────────────────────────────────────────
        elif state == "RETREAT":
            if target_pos is None:
                target_pos = np.array([BASKET_XY[0], BASKET_XY[1], RETREAT_Z])
                print(f"[RETREAT]   목표: {np.round(target_pos, 3)}")
            apply_step(target_pos)
            wait_count += 1
            if wait_count % 100 == 0:
                print(f"           현재: {np.round(get_hand_pos(), 3)}  (f={wait_count})")
            if is_reached(target_pos) or is_stalled() or wait_count >= STATE_TIMEOUT:
                go_idle()

rclpy.shutdown()
simulation_app.close()
