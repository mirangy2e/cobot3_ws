# -*- coding: utf-8 -*-
"""
[step05] 바나나 Pick & Place
=============================
Franka Panda 로봇이 테이블 위 바나나를 집어 목표 지점에 내려놓는 시뮬레이션.

주요 구성
  - 컨트롤러 : RMPflow (직교좌표 목표 → 관절속도 변환)
  - 물체     : YCB 011_banana + 보이지 않는 직육면체 collision box
  - 그리퍼   : Stiffness=1e6 (강한 파지력)

FSM 상태 순서
  WARMUP → OPEN_GRIPPER → PRE_PICK → PICK
  → CLOSE_GRIPPER → LIFT → PRE_PLACE → PLACE
  → OPEN_PLACE → RETURN → DONE

실행:
    ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/step05_banana_pick_place.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics, UsdLux, PhysxSchema, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import RMPFlowController

# ================================================================
# 파라미터
# ================================================================
PHYSICS_DT   = 1.0 / 200.0
RENDERING_DT = 20.0 / 200.0

# ── 로봇 ──────────────────────────────────────────────────────────
# (0, -0.5, 0) 위치에서 +Y 방향(바나나 쪽)을 바라봄
ROBOT_POS    = np.array([0.0, -0.5, 0.0])
ROBOT_ORIENT = np.array([0.7071067811865476, 0.0, 0.0, 0.7071067811865475])  # 90° Z 회전

# ── 바나나 ─────────────────────────────────────────────────────────
# 테이블 상면 z=0.08 위에 놓인 바나나 중심 z≈0.116 (반경 ~0.036m)
BANANA_POS        = np.array([0.0, -0.06, 0.098])   # 바나나 실제 물리 위치
BANANA_ORIENT_DEG = Gf.Vec3f(96.0, 0.0, 0.0)      # X축 90° → 옆으로 눕힘

# PRE_PICK 접근 목표: 수렴 후 팔 실제 XY를 캡처하여 PICK에서 사용
APPROACH_XY = np.array([BANANA_POS[0], BANANA_POS[1] + 0.05])

# ── Pick / Place 높이 ──────────────────────────────────────────────
PRE_PICK_Z  = 0.50
PICK_Z      = 0.10   # ★ 튜닝 포인트: 로그 "수렴: panda_hand z=..." 참고
LIFT_Z      = 0.50
PRE_PLACE_Z = 0.50
PLACE_POS   = np.array([0.3, 0.0, 0.14])   # 내려놓을 위치

# ── 그리퍼 ────────────────────────────────────────────────────────
GRIPPER_JOINT_INDICES = np.array([7, 8])
GRIPPER_OPEN   = np.array([0.04, 0.04])
GRIPPER_CLOSED = np.array([0.005, 0.005])  # 바나나 반경보다 작게 → 파지력 확보

QUAT_DOWN = np.array([0.0, 1.0, 0.0, 0.0])   # 그리퍼 수직 하향

# ── 초기 자세 ─────────────────────────────────────────────────────
JOINT_HOME = np.array([0.0, -1.5708, 0.0, -1.5708, 0.0, 1.5708, 0.7854, 0.04, 0.04])

# ── FSM 타이밍 ────────────────────────────────────────────────────
STALL_FRAMES       = 150
STALL_DELTA        = 0.003
STATE_TIMEOUT      = 2000
OPEN_GRIPPER_WAIT  = 80
CLOSE_GRIPPER_WAIT = 500
OPEN_PLACE_WAIT    = 150
RETURN_WAIT        = 2000

# ================================================================
# World / 씬 구성
# ================================================================
my_world = World(
    stage_units_in_meters=1.0,
    physics_dt=PHYSICS_DT,
    rendering_dt=RENDERING_DT,
)
stage = omni.usd.get_context().get_stage()

my_world.scene.add_default_ground_plane()

# ── 기본 조명 ──────────────────────────────────────────────────────
dome = stage.DefinePrim("/World/defaultLight", "DomeLight")
UsdLux.DomeLight(dome).GetIntensityAttr().Set(1000.0)

# ── Franka (명시적 위치·방향) ──────────────────────────────────────
my_franka = my_world.scene.add(
    Franka(
        prim_path="/World/Franka",
        name="my_franka",
        position=ROBOT_POS,
        orientation=ROBOT_ORIENT,
    )
)

# ── 바나나 받침대 (FixedCuboid) ────────────────────────────────────
# z 중심=0.04 → 상면=0.08m, 바나나 중심=0.08+0.036=0.116
my_world.scene.add(
    FixedCuboid(
        prim_path="/World/banana_table",
        name="banana_table",
        position=np.array([BANANA_POS[0], BANANA_POS[1], 0.04]),
        scale=np.array([0.25, 0.25, 0.08]),
    )
)

# ── 바나나 (Nucleus YCB 에셋 + RigidBody) ─────────────────────────
assets_root = get_assets_root_path()
banana_usd  = assets_root + "/Isaac/Props/YCB/Axis_Aligned/011_banana.usd"

banana_prim = stage.DefinePrim("/World/banana", "Xform")
banana_prim.GetReferences().AddReference(banana_usd)

xform = UsdGeom.XformCommonAPI(banana_prim)
xform.SetTranslate(Gf.Vec3d(float(BANANA_POS[0]),
                            float(BANANA_POS[1]),
                            float(BANANA_POS[2])))
xform.SetRotate(BANANA_ORIENT_DEG)
xform.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))

UsdPhysics.RigidBodyAPI.Apply(banana_prim)

# 에셋 로드 대기
for _ in range(5):
    simulation_app.update()

my_world.reset()

# ================================================================
# 바나나 물리 설정  (reset 이후)
# ================================================================
mat_path = "/World/BananaMaterial"
mat_prim = stage.DefinePrim(mat_path, "Material")
UsdPhysics.MaterialAPI.Apply(mat_prim)
stage.GetPrimAtPath(mat_path).GetAttribute("physics:staticFriction").Set(3.0)
stage.GetPrimAtPath(mat_path).GetAttribute("physics:dynamicFriction").Set(2.0)
stage.GetPrimAtPath(mat_path).GetAttribute("physics:restitution").Set(0.0)


# ── Collision Box (보이지 않는 직육면체) ─────────────────────────
# PhysX dynamic body는 triangle mesh collision을 지원하지 않으므로
# 바나나 시각 메시 대신 단순 직육면체를 collision shape로 사용.
# 크기를 바나나 단면에 맞추면 그리퍼가 실제 껍질을 집는 것처럼 보임.
#   BOX_X : 바나나 좌우 폭
#   BOX_Y : 그리핑 방향 깊이 (이 값으로 파지 타이트함 조정)
#   BOX_Z : 수직 높이
BOX_X      = 0.060
BOX_Y      = 0.032
BOX_Z      = 0.034
BOX_OFFSET = Gf.Vec3d(0.0, 0.0, -0.01706)  # 바나나 로컬 좌표 오프셋 [m]

col_prim = stage.DefinePrim("/World/banana/grip_box", "Cube")
UsdGeom.Cube(col_prim).GetSizeAttr().Set(1.0)
box_xform = UsdGeom.XformCommonAPI(col_prim)
box_xform.SetTranslate(BOX_OFFSET)
box_xform.SetScale(Gf.Vec3f(BOX_X, BOX_Y, BOX_Z))
UsdGeom.Imageable(col_prim).MakeInvisible()

UsdPhysics.CollisionAPI.Apply(col_prim)
physx_col = PhysxSchema.PhysxCollisionAPI.Apply(col_prim)
physx_col.GetContactOffsetAttr().Set(0.001)
physx_col.GetRestOffsetAttr().Set(0.0)
col_prim.GetRelationship("physics:material:binding").AddTarget(mat_path)

UsdPhysics.MassAPI.Apply(banana_prim).GetMassAttr().Set(0.1)  # 100 g

print("✅ 바나나 물리 설정 완료  (보이지 않는 큐브 collision, mass=0.1 kg)")

# ================================================================
# 그리퍼 Drive  (Stiffness=1e6 — 파지력 강화)
# ================================================================
for jpath in [
    "/World/Franka/panda_hand/panda_finger_joint1",
    "/World/Franka/panda_hand/panda_finger_joint2",
]:
    jprim = stage.GetPrimAtPath(jpath)
    if jprim.IsValid():
        d = UsdPhysics.DriveAPI.Get(jprim, "linear")
        if not d:
            d = UsdPhysics.DriveAPI.Apply(jprim, "linear")
        d.GetStiffnessAttr().Set(1e6)
        d.GetDampingAttr().Set(1e4)
print("✅ 그리퍼 Drive 완료  (Stiffness=1e6, Damping=1e4)")

# ================================================================
# 컨트롤러
# ================================================================
my_controller = RMPFlowController(
    name="rmp_ctrl",
    robot_articulation=my_franka,
    physics_dt=PHYSICS_DT,
)
articulation_ctrl = my_franka.get_articulation_controller()
print("✅ RMPflow 컨트롤러 생성 완료")

# reset() 호출 시 JOINT_HOME을 복원 기준 자세로 등록
my_franka.set_joints_default_state(
    positions=JOINT_HOME,
    velocities=np.zeros(len(JOINT_HOME)),
)
print("✅ 관절 기본 자세 등록 완료  (JOINT_HOME)")

# ================================================================
# 헬퍼
# ================================================================
_stall_pos   = None
_stall_count = 0


def reset_stall():
    global _stall_pos, _stall_count
    _stall_pos = None
    _stall_count = 0


def is_stalled():
    global _stall_pos, _stall_count
    try:
        pos, _ = my_franka.end_effector.get_world_pose()
        if _stall_pos is None or np.linalg.norm(pos - _stall_pos) > STALL_DELTA:
            _stall_pos = pos.copy(); _stall_count = 0; return False
        _stall_count += 1
        return _stall_count >= STALL_FRAMES
    except Exception:
        return False


def get_hand_pos():
    pos, _ = my_franka.end_effector.get_world_pose()
    return pos


def apply_arm(target):
    """RMPflow 속도 명령을 그대로 전달 — 스케일링 없음 (중력 보상 유지)."""
    action = my_controller.forward(
        target_end_effector_position=target,
        target_end_effector_orientation=QUAT_DOWN,
    )
    if action is not None:
        articulation_ctrl.apply_action(action)


def open_gripper():
    my_franka.apply_action(ArticulationAction(
        joint_positions=GRIPPER_OPEN,
        joint_indices=GRIPPER_JOINT_INDICES,
    ))


def close_gripper():
    my_franka.apply_action(ArticulationAction(
        joint_positions=GRIPPER_CLOSED,
        joint_indices=GRIPPER_JOINT_INDICES,
    ))


def reinit_rmpflow():
    robot_pos, robot_ori = my_franka.get_world_pose()
    my_controller.rmp_flow.set_robot_base_pose(robot_pos, robot_ori)


def scene_reset():
    my_world.reset()
    my_controller.reset()
    reinit_rmpflow()
    my_franka.set_joint_positions(JOINT_HOME)


# ================================================================
# FSM 상태 정의
# ================================================================
STATE_WARMUP        = "WARMUP"       # 물리 정착 대기 (첫 실행 폭발 방지)
STATE_OPEN_GRIPPER  = "OPEN_GRIPPER"
STATE_PRE_PICK      = "PRE_PICK"
STATE_PICK          = "PICK"
STATE_CLOSE_GRIPPER = "CLOSE_GRIPPER"
STATE_LIFT          = "LIFT"
STATE_PRE_PLACE     = "PRE_PLACE"
STATE_PLACE         = "PLACE"
STATE_OPEN_PLACE    = "OPEN_PLACE"
STATE_RETURN        = "RETURN"
STATE_DONE          = "DONE"

WARMUP_FRAMES = 80   # 물리 정착 대기 프레임 (200Hz → ~0.4초)

current_state = STATE_WARMUP
state_counter = 0
_pick_x = BANANA_POS[0]
_pick_y = BANANA_POS[1]

print("\n" + "=" * 60)
print("  ▶  step05 v2 : 바나나 Pick & Place (명시적 에셋)")
print(f"  ROBOT_POS  = {ROBOT_POS}  (방향: +Y 정면)")
print(f"  BANANA_POS  = {BANANA_POS}  (물리 위치)")
print(f"  APPROACH_XY = {APPROACH_XY}")
print(f"  PICK_Z      = {PICK_Z}  ← 튜닝 포인트")
print(f"  PLACE_POS  = {PLACE_POS}")
print("=" * 60 + "\n")

print("✅ 초기화 완료 — 메인 루프 시작\n")

# ================================================================
# 메인 루프
# ================================================================
reset_needed = False

while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():

        if my_world.current_time_step_index == 0:
            scene_reset()
            current_state = STATE_WARMUP   # ← 항상 WARMUP부터
            state_counter = 0
            _pick_x = APPROACH_XY[0]; _pick_y = APPROACH_XY[1]
            reset_stall()

        if reset_needed:
            scene_reset()
            current_state = STATE_WARMUP   # ← 항상 WARMUP부터
            state_counter = 0
            _pick_x = APPROACH_XY[0]; _pick_y = APPROACH_XY[1]
            reset_stall()
            reset_needed = False
            print("🔄 리셋\n")

        state_counter += 1

        # ── 0. WARMUP — 물리 정착 대기 (관절 위치 능동 유지) ──────────
        if current_state == STATE_WARMUP:
            # RMPflow 속도 명령 없이 관절 위치만 직접 유지
            # → 첫 프레임 충돌로 인한 PhysX 폭발 방지
            my_franka.apply_action(ArticulationAction(
                joint_positions=JOINT_HOME,
                joint_indices=np.arange(len(JOINT_HOME)),
            ))
            if state_counter == 1:
                print(f"[WARMUP] 물리 정착 대기 ({WARMUP_FRAMES}f)...")
            if state_counter >= WARMUP_FRAMES:
                current_state = STATE_OPEN_GRIPPER
                state_counter = 0; reset_stall()
                print("  → OPEN_GRIPPER")

        # ── 1. OPEN_GRIPPER ────────────────────────────────────────
        elif current_state == STATE_OPEN_GRIPPER:
            open_gripper()
            apply_arm(np.array([APPROACH_XY[0], APPROACH_XY[1], LIFT_Z]))
            if state_counter == 1:
                print("[OPEN_GRIPPER] 그리퍼 열기")
            if state_counter >= OPEN_GRIPPER_WAIT:
                current_state = STATE_PRE_PICK
                state_counter = 0; reset_stall()
                print("  → PRE_PICK")

        # ── 2. PRE_PICK — 바나나 위 수렴, 실제 XY 캡처 ─────────────
        elif current_state == STATE_PRE_PICK:
            target = np.array([APPROACH_XY[0], APPROACH_XY[1], PRE_PICK_Z])
            apply_arm(target); open_gripper()
            if state_counter == 1:
                print(f"[PRE_PICK] 목표: {np.round(target, 3)}")
            if state_counter % 100 == 0:
                hand    = get_hand_pos()
                xy_err  = np.linalg.norm(hand[:2] - target[:2])
                print(f"   [{state_counter:4d}f] panda_hand z={hand[2]:.3f}  XY오차={xy_err:.4f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                hand   = get_hand_pos()
                _pick_x = hand[0]
                _pick_y = hand[1]
                current_state = STATE_PICK
                state_counter = 0; reset_stall()
                print(f"  → PICK  (XY고정: {_pick_x:.4f}, {_pick_y:.4f})")

        # ── 3. PICK — XY 고정, Z 하강 ─────────────────────────────
        elif current_state == STATE_PICK:
            target = np.array([_pick_x, _pick_y, PICK_Z])
            apply_arm(target); open_gripper()
            if state_counter == 1:
                print(f"[PICK] 목표 z={PICK_Z}  XY=({_pick_x:.4f}, {_pick_y:.4f}) 고정")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] panda_hand z={get_hand_pos()[2]:.3f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                hand = get_hand_pos()
                print(f"   수렴: panda_hand z={hand[2]:.4f}  ← PICK_Z 튜닝 참고")
                current_state = STATE_CLOSE_GRIPPER
                state_counter = 0; reset_stall()
                print("  → CLOSE_GRIPPER")

        # ── 4. CLOSE_GRIPPER ──────────────────────────────────────
        elif current_state == STATE_CLOSE_GRIPPER:
            # 팔은 현재 위치를 유지하고 그리퍼만 닫아 바나나를 파지
            close_gripper()
            if state_counter == 1:
                z = get_hand_pos()[2]
                print(f"[CLOSE_GRIPPER] 그리퍼 닫기  (panda_hand z={z:.4f}, {CLOSE_GRIPPER_WAIT}f 대기)")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] 닫는 중...")
            if state_counter >= CLOSE_GRIPPER_WAIT:
                current_state = STATE_LIFT
                state_counter = 0; reset_stall()
                print("  → LIFT")

        # ── 5. LIFT ────────────────────────────────────────────────
        elif current_state == STATE_LIFT:
            target = np.array([BANANA_POS[0], BANANA_POS[1], LIFT_Z])
            apply_arm(target); close_gripper()
            if state_counter == 1:
                print(f"[LIFT] 목표 z={LIFT_Z}")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] panda_hand z={get_hand_pos()[2]:.3f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                print(f"   LIFT 완료: panda_hand z={get_hand_pos()[2]:.4f}")
                current_state = STATE_PRE_PLACE
                state_counter = 0; reset_stall()
                print("  → PRE_PLACE")

        # ── 6. PRE_PLACE ───────────────────────────────────────────
        elif current_state == STATE_PRE_PLACE:
            target = np.array([PLACE_POS[0], PLACE_POS[1], PRE_PLACE_Z])
            apply_arm(target); close_gripper()
            if state_counter == 1:
                print(f"[PRE_PLACE] 목표 xy={PLACE_POS[:2]}  z={PRE_PLACE_Z}")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] panda_hand z={get_hand_pos()[2]:.3f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                current_state = STATE_PLACE
                state_counter = 0; reset_stall()
                print("  → PLACE")

        # ── 7. PLACE ───────────────────────────────────────────────
        elif current_state == STATE_PLACE:
            apply_arm(PLACE_POS); close_gripper()
            if state_counter == 1:
                print(f"[PLACE] 목표: {PLACE_POS}")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] panda_hand z={get_hand_pos()[2]:.3f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                print(f"   수렴: panda_hand z={get_hand_pos()[2]:.4f}")
                current_state = STATE_OPEN_PLACE
                state_counter = 0; reset_stall()
                print("  → OPEN_PLACE")

        # ── 8. OPEN_PLACE ──────────────────────────────────────────
        elif current_state == STATE_OPEN_PLACE:
            hand = get_hand_pos()
            apply_arm(np.array([PLACE_POS[0], PLACE_POS[1], hand[2]]))
            open_gripper()
            if state_counter == 1:
                print("[OPEN_PLACE] 그리퍼 열어 내려놓기")
            if state_counter >= OPEN_PLACE_WAIT:
                current_state = STATE_RETURN
                state_counter = 0; reset_stall()
                print("  → RETURN")

        # ── 9. RETURN ──────────────────────────────────────────────
        elif current_state == STATE_RETURN:
            target = np.array([BANANA_POS[0], BANANA_POS[1], LIFT_Z])
            apply_arm(target); open_gripper()
            if state_counter == 1:
                print("[RETURN] 홈 복귀")
            if is_stalled() or state_counter >= RETURN_WAIT:
                current_state = STATE_DONE
                state_counter = 0
                print("\n🎉 Pick & Place 완료!\n")

        # ── 10. DONE ───────────────────────────────────────────────
        elif current_state == STATE_DONE:
            open_gripper()

print("\n⏹  종료")
simulation_app.close()
