# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# franka_pick_up.py 스타일 기반 — 바나나 Pick & Place
# =====================================================
# franka_pick_up.py 와 동일한 구조 유지:
#   - SingleManipulator + ParallelGripper
#   - PickPlaceController
#   - 단순 메인 루프 (FSM 없음)
#
# step05_banana_pick_place.py 에서 검증된 바나나 물리 설정 적용:
#   - YCB 011_banana.usd (Nucleus)
#   - 보이지 않는 직육면체 collision box (PhysX dynamic body 제약 회피)
#   - 마찰 재질 (static=3.0, dynamic=2.0)
#   - 그리퍼 Drive Stiffness=1e6 (파지력 강화)
#   - WARMUP 카운터 (첫 실행 물리 폭발 방지)
#
# 실행:
#   ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/franka_banana_pick_place.py

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys

import carb
import numpy as np
import omni.usd
from pxr import Gf, PhysxSchema, UsdGeom, UsdLux, UsdPhysics
from isaacsim.core.api import World
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators import SingleManipulator
from isaacsim.robot.manipulators.examples.franka.controllers.pick_place_controller import PickPlaceController
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.storage.native import get_assets_root_path

# ================================================================
# 파라미터
# ================================================================

# ── 바나나 ──────────────────────────────────────────────────────────
BANANA_POS        = np.array([0.40, 0.0, 0.10])    # 로봇 정면 40cm, 테이블 위
BANANA_ORIENT_DEG = Gf.Vec3f(90.0, 0.0, 0.0)       # X축 90° → 옆으로 눕힘

# ── Place 목표 ────────────────────────────────────────────────────
PLACE_POS = np.array([0.10, -0.40, 0.10])           # 내려놓을 위치

# ── 충돌 박스 (step05에서 검증된 값) ──────────────────────────────
#   BOX_X : 바나나 좌우 폭
#   BOX_Y : 그리핑 방향 깊이 ★ 시각적 타이트함 결정
#   BOX_Z : 높이 (PICK 높이 오차 흡수)
BOX_X      = 0.060
BOX_Y      = 0.032
BOX_Z      = 0.034
BOX_OFFSET = Gf.Vec3d(0.0, 0.0, -0.017)            # 바나나 로컬 좌표 오프셋

# ── 워밍업 ────────────────────────────────────────────────────────
WARMUP_FRAMES = 80  # 물리 정착 대기 (첫 실행 충돌 폭발 방지)

# ================================================================
# 에셋 경로 확인
# ================================================================
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# ================================================================
# World / 씬 구성
# ================================================================
my_world = World(stage_units_in_meters=1.0)
stage    = omni.usd.get_context().get_stage()

my_world.scene.add_default_ground_plane()

# ── 기본 조명 ──────────────────────────────────────────────────────
dome_prim = stage.DefinePrim("/World/defaultLight", "DomeLight")
UsdLux.DomeLight(dome_prim).GetIntensityAttr().Set(1000.0)

# ── Franka 로봇 (franka_pick_up.py 동일) ───────────────────────────
robot_usd = assets_root_path + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
robot_ref = add_reference_to_stage(usd_path=robot_usd, prim_path="/World/Franka")
robot_ref.GetVariantSet("Gripper").SetVariantSelection("AlternateFinger")
robot_ref.GetVariantSet("Mesh").SetVariantSelection("Quality")

gripper = ParallelGripper(
    end_effector_prim_path="/World/Franka/panda_rightfinger",
    joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
    joint_opened_positions=np.array([0.04, 0.04]),
    joint_closed_positions=np.array([0.005, 0.005]),  # BOX_Y/2 보다 작게 → 파지력 확보
    action_deltas=np.array([0.005, 0.005]),
)
my_franka = my_world.scene.add(
    SingleManipulator(
        prim_path="/World/Franka",
        name="my_franka",
        end_effector_prim_path="/World/Franka/panda_rightfinger",
        gripper=gripper,
    )
)

# ── 바나나 받침대 ─────────────────────────────────────────────────
# 바나나 중심 z=0.10, 반경 ~0.017 → 테이블 상면 z≈0.083
my_world.scene.add(
    FixedCuboid(
        prim_path="/World/banana_table",
        name="banana_table",
        position=np.array([BANANA_POS[0], BANANA_POS[1], 0.04]),
        scale=np.array([0.25, 0.25, 0.08]),
    )
)

# ── 바나나 (Nucleus YCB 에셋) ────────────────────────────────────
banana_usd  = assets_root_path + "/Isaac/Props/YCB/Axis_Aligned/011_banana.usd"
banana_prim = stage.DefinePrim("/World/banana", "Xform")
banana_prim.GetReferences().AddReference(banana_usd)

UsdGeom.XformCommonAPI(banana_prim).SetTranslate(
    Gf.Vec3d(float(BANANA_POS[0]), float(BANANA_POS[1]), float(BANANA_POS[2]))
)
UsdGeom.XformCommonAPI(banana_prim).SetRotate(BANANA_ORIENT_DEG)
UsdGeom.XformCommonAPI(banana_prim).SetScale(Gf.Vec3f(1.0, 1.0, 1.0))
UsdPhysics.RigidBodyAPI.Apply(banana_prim)

# 바나나 위치 추적 (physics 갱신 후 실시간 위치 읽기)
banana_obj = my_world.scene.add(XFormPrim("/World/banana", "banana"))

# ── 초기화 ────────────────────────────────────────────────────────
my_franka.gripper.set_default_state(my_franka.gripper.joint_opened_positions)

for _ in range(5):          # 에셋 로드 대기
    simulation_app.update()

my_world.reset()

# ================================================================
# 바나나 물리 설정  (reset 이후 적용)
# ================================================================

# ── 마찰 재질 ──────────────────────────────────────────────────────
mat_path = "/World/BananaMaterial"
mat_prim = stage.DefinePrim(mat_path, "Material")
UsdPhysics.MaterialAPI.Apply(mat_prim)
stage.GetPrimAtPath(mat_path).GetAttribute("physics:staticFriction").Set(3.0)
stage.GetPrimAtPath(mat_path).GetAttribute("physics:dynamicFriction").Set(2.0)
stage.GetPrimAtPath(mat_path).GetAttribute("physics:restitution").Set(0.0)

# ── 보이지 않는 직육면체 collision box ───────────────────────────────
# PhysX dynamic body 제약: triangle mesh 불가 → 단순 box로 대체
# 크기를 바나나 시각 표면에 맞춰 손가락이 껍질에 닿아 보이게 함
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

# ── 질량 명시 ─────────────────────────────────────────────────────
UsdPhysics.MassAPI.Apply(banana_prim).GetMassAttr().Set(0.1)   # 100 g

# ── 그리퍼 Drive (Stiffness=1e6 → 파지력 강화) ──────────────────────
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

# ================================================================
# 컨트롤러 (franka_pick_up.py 동일)
# ================================================================
my_controller = PickPlaceController(
    name="pick_place_controller",
    gripper=my_franka.gripper,
    robot_articulation=my_franka,
)
articulation_controller = my_franka.get_articulation_controller()

print("\n" + "=" * 55)
print("  ▶  franka_banana_pick_place")
print(f"  BANANA_POS = {BANANA_POS}")
print(f"  PLACE_POS  = {PLACE_POS}")
print(f"  WARMUP     = {WARMUP_FRAMES} frames")
print("=" * 55 + "\n")

# ================================================================
# 메인 루프  (franka_pick_up.py 스타일 유지)
# ================================================================
reset_needed   = False
task_completed = False
warmup_counter = 0

while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed   = True
        task_completed = False

    if my_world.is_playing():

        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()
            warmup_counter = 0
            task_completed = False

        if reset_needed:
            my_world.reset()
            my_controller.reset()
            reset_needed   = False
            task_completed = False
            warmup_counter = 0

        # ── WARMUP: 물리 정착 대기 ─────────────────────────────────
        # 첫 실행 시 기본 자세가 바나나 box와 겹쳐 물리 폭발 방지
        warmup_counter += 1
        if warmup_counter <= WARMUP_FRAMES:
            continue

        # ── Pick & Place ───────────────────────────────────────────
        if not task_completed:
            banana_pos = banana_obj.get_world_poses()[0][0]   # 물리 갱신 후 실시간 위치

            actions = my_controller.forward(
                picking_position=banana_pos,
                placing_position=PLACE_POS,
                current_joint_positions=my_franka.get_joint_positions(),
                end_effector_offset=np.array([0.0, 0.005, 0.0]),
            )
            articulation_controller.apply_action(actions)

            if my_controller.is_done():
                print("✅ 바나나 집어서 이동 완료!")
                task_completed = True

simulation_app.close()
