# -*- coding: utf-8 -*-
"""
Franka 바나나 Pick & Place  —  RMPflow + FSM
=============================================
Franka Panda 로봇이 테이블 위 바나나를 집어 목표 지점에 내려놓는 시뮬레이션.

주요 구성
  - 컨트롤러 : RMPflow (직교좌표 목표 → 관절속도 변환)
  - 물체     : YCB 011_banana + 보이지 않는 직육면체 collision box
  - 뷰포트   : Perspective(로봇 줌인) + Top-View 카메라(바나나 하향) 5:5 분할

FSM 상태 순서
  WARMUP → OPEN_GRIPPER → PRE_PICK → PICK
  → CLOSE_GRIPPER → LIFT → PRE_PLACE → PLACE
  → OPEN_PLACE → RETURN → DONE

실행
  ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/step06_ros2_actiongraph.py
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from pxr import UsdGeom, UsdPhysics, UsdLux, PhysxSchema, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
import omni.ui as ui
import omni.kit.viewport.utility as vp_util
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import RMPFlowController

# ================================================================
# 파라미터
# ================================================================
PHYSICS_DT   = 1.0 / 200.0   # 물리 스텝 주기 (200 Hz)
RENDERING_DT = 20.0 / 200.0  # 렌더링 주기 (10 Hz)

# ── 로봇 ──────────────────────────────────────────────────────────
ROBOT_POS    = np.array([0.0, -0.5, 0.0])
ROBOT_ORIENT = np.array([0.7071067811865476, 0.0, 0.0, 0.7071067811865475])  # Z축 90° 회전 → +Y 정면

# ── 바나나 ─────────────────────────────────────────────────────────
BANANA_POS        = np.array([0.0, -0.06, 0.098])  # 테이블 상면 위 바나나 중심
BANANA_ORIENT_DEG = Gf.Vec3f(96.0, 0.0, 0.0)       # X축 회전 → 옆으로 눕힘

# PRE_PICK 접근 목표: 실제 XY는 수렴 후 팔 위치에서 캡처하여 PICK에서 사용
APPROACH_XY = np.array([BANANA_POS[0], BANANA_POS[1] + 0.05])

# ── Pick / Place 높이 ──────────────────────────────────────────────
PRE_PICK_Z  = 0.50           # 바나나 위 접근 높이
PICK_Z      = 0.10           # ★ 튜닝: 로그 "수렴: panda_hand z=..." 참고
LIFT_Z      = 0.50           # 집은 후 이동 높이
PRE_PLACE_Z = 0.50           # 내려놓기 전 접근 높이
PLACE_POS   = np.array([0.3, 0.0, 0.14])  # 최종 내려놓기 위치

# ── 그리퍼 ────────────────────────────────────────────────────────
GRIPPER_JOINT_INDICES = np.array([7, 8])
GRIPPER_OPEN   = np.array([0.04, 0.04])
GRIPPER_CLOSED = np.array([0.005, 0.005])  # collision box 반폭보다 작게 → 파지력 확보

QUAT_DOWN = np.array([0.0, 1.0, 0.0, 0.0])  # 엔드이펙터 수직 하향 자세

# ── 초기 자세 (90/90 형상) ────────────────────────────────────────
# 어깨(-45°)와 팔꿈치(-135°)를 꺾어 손목이 바나나 위에 오도록 배치
#   q1= 0°, q2=-45°, q3= 0°, q4=-135°, q5= 0°, q6=90°, q7=45°
JOINT_HOME = np.array([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854, 0.04, 0.04])

# ── Top-View 카메라 ────────────────────────────────────────────────
# 바나나 정위에서 수직 하향, 장축(Y)이 뷰포트 가로 방향으로 정렬
#   Rz=-90° : 카메라 기본 방향(-Z 하향)을 유지하며 좌우 방향만 회전
#   FL=35mm : 카메라-바나나 거리 ~0.33m에서 바나나가 화면의 약 85% 차지
TOP_CAM_PATH   = "/World/Camera"
CAM_TRANSLATE  = Gf.Vec3d(0.0, -0.06, 0.80)   # 바나나 정위 수직 [m]
CAM_ROTATE_DEG = Gf.Vec3f(0.0, 0.0, -90.0)    # 수직 하향 + 바나나 가로 정렬
CAM_FOCAL_LEN  = 35.0    # Focal Length  [mm]
CAM_FOCUS_DIST = 34.0    # Focus Distance [cm]
CAM_CLIP_NEAR  = 0.01    # Near clip [m]
CAM_CLIP_FAR   = 10.0    # Far  clip [m]

# ── Perspective 카메라 (Viewport 1 — 로봇 작업 줌인) ─────────────
PERSP_EYE    = np.array([0.8, -1.1, 0.6])   # 로봇 우측 전방 상공
PERSP_TARGET = np.array([0.0, -0.3, 0.25])  # 팔·바나나 작업 공간 중심

# ── FSM 타이밍 ────────────────────────────────────────────────────
STALL_FRAMES       = 150   # 수렴 판정: 연속 정지 프레임 수
STALL_DELTA        = 0.003 # 수렴 판정: 위치 변화 임계값 [m]
STATE_TIMEOUT      = 2000  # 상태별 최대 프레임 수 (타임아웃 안전장치)
OPEN_GRIPPER_WAIT  = 80
CLOSE_GRIPPER_WAIT = 500   # 파지력이 충분히 작용할 때까지 대기
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

dome = stage.DefinePrim("/World/defaultLight", "DomeLight")
UsdLux.DomeLight(dome).GetIntensityAttr().Set(1000.0)

# ── Franka ────────────────────────────────────────────────────────
my_franka = my_world.scene.add(
    Franka(
        prim_path="/World/Franka",
        name="my_franka",
        position=ROBOT_POS,
        orientation=ROBOT_ORIENT,
    )
)

# ── 바나나 받침대 ─────────────────────────────────────────────────
my_world.scene.add(
    FixedCuboid(
        prim_path="/World/banana_table",
        name="banana_table",
        position=np.array([BANANA_POS[0], BANANA_POS[1], 0.04]),
        scale=np.array([0.25, 0.25, 0.08]),
    )
)

# ── 바나나 (YCB 011_banana, RigidBody) ───────────────────────────
assets_root = get_assets_root_path()
banana_usd  = assets_root + "/Isaac/Props/YCB/Axis_Aligned/011_banana.usd"

banana_prim = stage.DefinePrim("/World/banana", "Xform")
banana_prim.GetReferences().AddReference(banana_usd)

xform = UsdGeom.XformCommonAPI(banana_prim)
xform.SetTranslate(Gf.Vec3d(float(BANANA_POS[0]), float(BANANA_POS[1]), float(BANANA_POS[2])))
xform.SetRotate(BANANA_ORIENT_DEG)
xform.SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
UsdPhysics.RigidBodyAPI.Apply(banana_prim)

# ── Top-View 카메라 프림 ──────────────────────────────────────────
cam_prim = stage.DefinePrim(TOP_CAM_PATH, "Camera")
cam      = UsdGeom.Camera(cam_prim)
cam.GetFocalLengthAttr().Set(CAM_FOCAL_LEN)
cam.GetFocusDistanceAttr().Set(CAM_FOCUS_DIST)
cam.GetClippingRangeAttr().Set(Gf.Vec2f(CAM_CLIP_NEAR, CAM_CLIP_FAR))

for _ in range(5):
    simulation_app.update()

my_world.reset()

# ================================================================
# 바나나 물리 설정  (reset 이후 적용)
# ================================================================

# ── 마찰 재질 ─────────────────────────────────────────────────────
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

print("✅ 바나나 물리 설정 완료")

# ================================================================
# 그리퍼 Drive
# ================================================================
# Stiffness=1e6으로 설정해 그리퍼가 목표 위치까지 강한 힘으로 닫히도록 함.
# 파지력 = Stiffness × (target_pos - actual_pos)
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
print("✅ 그리퍼 Drive 완료")

# ================================================================
# 카메라 / 뷰포트 설정
# ================================================================
cam_xform = UsdGeom.XformCommonAPI(cam_prim)
cam_xform.SetTranslate(CAM_TRANSLATE)
cam_xform.SetRotate(CAM_ROTATE_DEG)

# 5:5 분할 뷰포트
#   ┌─────────────────┬─────────────────┐
#   │  Viewport 1     │  Viewport 2     │
#   │  Perspective    │  Top-View Cam   │
#   │  (로봇 줌인)    │  (바나나 하향)  │
#   └─────────────────┴─────────────────┘
try:
    # Viewport 1: Perspective 카메라 줌인
    set_camera_view(eye=PERSP_EYE, target=PERSP_TARGET, camera_prim_path="/OmniverseKit_Persp")
    vp1_api = vp_util.get_viewport_from_window_name("Viewport")
    if vp1_api:
        vp1_api.camera_path = "/OmniverseKit_Persp"
    print(f"✅ Perspective 줌인  eye={PERSP_EYE}  target={PERSP_TARGET}")

    # Viewport 2: Top-View 카메라
    ui.Workspace.show_window("Viewport 2")
    simulation_app.update()
    simulation_app.update()
    vp2_api = vp_util.get_viewport_from_window_name("Viewport 2")
    if vp2_api:
        vp2_api.camera_path = TOP_CAM_PATH
    print(f"✅ Viewport 2 카메라 연결  →  {TOP_CAM_PATH}")

    # dock_in: WindowHandle 타입으로 5:5 분할
    vp1_handle = ui.Workspace.get_window("Viewport")
    vp2_handle = ui.Workspace.get_window("Viewport 2")
    if vp1_handle and vp2_handle:
        vp2_handle.dock_in(vp1_handle, ui.DockPosition.RIGHT, 0.5)
        print("✅ 5:5 분할 완료  (Left: Perspective | Right: Top-View)")
    else:
        print("⚠  WindowHandle 취득 실패 → Viewport 2 는 별도 창으로 표시")

except Exception as e:
    print(f"⚠  뷰포트 설정 실패: {e}")

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
# PhysX 사전 워밍업
# ================================================================
# render=False로 물리 프레임을 미리 누적해 PhysX 내부 캐시를 초기화.
# 워밍업 없이 시작하면 첫 그리퍼 접촉 시 constraint force가 폭발적으로 커짐.
PRE_WARMUP_FRAMES = 200
print(f"PhysX 워밍업 중... ({PRE_WARMUP_FRAMES} steps, render=False)")
for _ in range(PRE_WARMUP_FRAMES):
    my_world.step(render=False)
my_world.reset()  # 캐시 유지 + JOINT_HOME 위치 복원
print("✅ PhysX 워밍업 완료")

# ================================================================
# 헬퍼 함수
# ================================================================
_stall_pos   = None
_stall_count = 0


def reset_stall():
    global _stall_pos, _stall_count
    _stall_pos = None
    _stall_count = 0


def is_stalled():
    """엔드이펙터가 STALL_FRAMES 동안 STALL_DELTA 이내로 정지 시 True."""
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
    """RMPflow로 엔드이펙터를 target(XYZ)으로 이동. 속도 스케일 없음."""
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
    """reset 후 RMPflow에 현재 로봇 베이스 위치를 재등록."""
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
STATE_WARMUP        = "WARMUP"
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

WARMUP_FRAMES = 80  # 물리 정착 대기 (200Hz 기준 약 0.4초)

current_state = STATE_WARMUP
state_counter = 0
_pick_x = BANANA_POS[0]
_pick_y = BANANA_POS[1]

print("\n" + "=" * 60)
print("  ▶  Franka 바나나 Pick & Place")
print(f"  ROBOT_POS   = {ROBOT_POS}")
print(f"  BANANA_POS  = {BANANA_POS}")
print(f"  PICK_Z      = {PICK_Z}  ← 튜닝 포인트")
print(f"  PLACE_POS   = {PLACE_POS}")
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
            current_state = STATE_WARMUP
            state_counter = 0
            _pick_x = APPROACH_XY[0]; _pick_y = APPROACH_XY[1]
            reset_stall()

        if reset_needed:
            scene_reset()
            current_state = STATE_WARMUP
            state_counter = 0
            _pick_x = APPROACH_XY[0]; _pick_y = APPROACH_XY[1]
            reset_stall()
            reset_needed = False
            print("🔄 리셋\n")

        state_counter += 1

        # ── WARMUP ─────────────────────────────────────────────────
        # RMPflow 대신 관절 위치를 직접 지정해 JOINT_HOME을 능동 유지.
        # 물리 안정화 후 다음 상태로 진행.
        if current_state == STATE_WARMUP:
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

        # ── OPEN_GRIPPER ───────────────────────────────────────────
        elif current_state == STATE_OPEN_GRIPPER:
            open_gripper()
            apply_arm(np.array([APPROACH_XY[0], APPROACH_XY[1], LIFT_Z]))
            if state_counter == 1:
                print("[OPEN_GRIPPER] 그리퍼 열기")
            if state_counter >= OPEN_GRIPPER_WAIT:
                current_state = STATE_PRE_PICK
                state_counter = 0; reset_stall()
                print("  → PRE_PICK")

        # ── PRE_PICK ───────────────────────────────────────────────
        # 바나나 위 접근 후 수렴 시점의 실제 팔 XY를 캡처 → PICK에서 사용.
        elif current_state == STATE_PRE_PICK:
            target = np.array([APPROACH_XY[0], APPROACH_XY[1], PRE_PICK_Z])
            apply_arm(target); open_gripper()
            if state_counter == 1:
                print(f"[PRE_PICK] 목표: {np.round(target, 3)}")
            if state_counter % 100 == 0:
                hand   = get_hand_pos()
                xy_err = np.linalg.norm(hand[:2] - target[:2])
                print(f"   [{state_counter:4d}f] panda_hand z={hand[2]:.3f}  XY오차={xy_err:.4f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                hand   = get_hand_pos()
                _pick_x = hand[0]
                _pick_y = hand[1]
                current_state = STATE_PICK
                state_counter = 0; reset_stall()
                print(f"  → PICK  (XY고정: {_pick_x:.4f}, {_pick_y:.4f})")

        # ── PICK ───────────────────────────────────────────────────
        # XY는 PRE_PICK 수렴값으로 고정, Z만 하강.
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

        # ── CLOSE_GRIPPER ──────────────────────────────────────────
        # 팔은 현재 위치를 유지하고 그리퍼만 닫아 바나나를 파지.
        elif current_state == STATE_CLOSE_GRIPPER:
            close_gripper()
            if state_counter == 1:
                print(f"[CLOSE_GRIPPER] 그리퍼 닫기  (panda_hand z={get_hand_pos()[2]:.4f}, {CLOSE_GRIPPER_WAIT}f 대기)")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] 닫는 중...")
            if state_counter >= CLOSE_GRIPPER_WAIT:
                current_state = STATE_LIFT
                state_counter = 0; reset_stall()
                print("  → LIFT")

        # ── LIFT ───────────────────────────────────────────────────
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

        # ── PRE_PLACE ──────────────────────────────────────────────
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

        # ── PLACE ──────────────────────────────────────────────────
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

        # ── OPEN_PLACE ─────────────────────────────────────────────
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

        # ── RETURN ─────────────────────────────────────────────────
        elif current_state == STATE_RETURN:
            target = np.array([BANANA_POS[0], BANANA_POS[1], LIFT_Z])
            apply_arm(target); open_gripper()
            if state_counter == 1:
                print("[RETURN] 홈 복귀")
            if is_stalled() or state_counter >= RETURN_WAIT:
                current_state = STATE_DONE
                state_counter = 0
                print("\n🎉 Pick & Place 완료!\n")

        # ── DONE ───────────────────────────────────────────────────
        elif current_state == STATE_DONE:
            open_gripper()

print("\n⏹  종료")
simulation_app.close()
