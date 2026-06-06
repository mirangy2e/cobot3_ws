# -*- coding: utf-8 -*-
"""
Franka 바나나 Pick & Place  —  RMPflow + FSM + ROS2 Camera + Clock + TF
========================================================================
step07a 에 Clock / TF 토픽 발행을 추가한 버전.
세 그래프 모두 graph_dump.txt 에서 확인한 노드/설정값 그대로 재현.

ROS2 발행 토픽
  Camera Graph  (/World/Graph/camera_graph)
    /camera/rgb   — RGB 이미지  (640×640)
    /camera/depth — 깊이 이미지 (640×640)
    camera_info   — 카메라 내부 파라미터

  Clock Graph  (/World/Graph/clock_graph)
    clock         — 시뮬레이션 시간

  TF Graph  (/World/Graph/tf_graph)
    /tf           — 로봇 링크 변환 트리 (/World/Franka, /World/Camera)

주요 구성
  - 컨트롤러 : RMPflow
  - 물체     : YCB 011_banana + 보이지 않는 직육면체 collision box
  - 카메라   : Top-View (바나나 수직 하향)
  - 뷰포트   : Perspective(로봇 줌인) + Top-View 5:5 분할

FSM 상태 순서
  WARMUP → OPEN_GRIPPER → PRE_PICK → PICK
  → CLOSE_GRIPPER → LIFT → PRE_PLACE → PLACE
  → OPEN_PLACE → RETURN → DONE

실행
  ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/step07b_ros2_actiongraph_clock_tf.py

ROS2 토픽 확인
  export ROS_DOMAIN_ID=50
  ros2 topic list
  ros2 topic hz /clock
  ros2 topic hz /camera/rgb
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# step02에서 검증한 방식: ros2.bridge 단독 활성화
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

import traceback
import numpy as np
import omni.usd
import omni.graph.core as og
from pxr import UsdGeom, UsdPhysics, UsdLux, PhysxSchema, Gf
from isaacsim.core.api import World
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.utils.nucleus import get_assets_root_path
from isaacsim.core.utils.prims import set_targets
import omni.ui as ui
import omni.kit.viewport.utility as vp_util
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.core.api.objects import FixedCuboid
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import RMPFlowController

# ================================================================
# 파라미터
# ================================================================
PHYSICS_DT   = 1.0 / 200.0
RENDERING_DT = 20.0 / 200.0

# ── 로봇 ──────────────────────────────────────────────────────────
ROBOT_POS    = np.array([0.0, -0.5, 0.0])
ROBOT_ORIENT = np.array([0.7071067811865476, 0.0, 0.0, 0.7071067811865475])  # Z축 90° 회전 → +Y 정면

# ── 바나나 ─────────────────────────────────────────────────────────
BANANA_POS        = np.array([0.0, -0.06, 0.098])
BANANA_ORIENT_DEG = Gf.Vec3f(96.0, 0.0, 0.0)

# PRE_PICK 접근 목표: 수렴 후 팔 실제 XY를 캡처하여 PICK에서 사용
APPROACH_XY = np.array([BANANA_POS[0], BANANA_POS[1] + 0.05])

# ── Pick / Place 높이 ──────────────────────────────────────────────
PRE_PICK_Z  = 0.50
PICK_Z      = 0.10   # ★ 튜닝: 로그 "수렴: panda_hand z=..." 참고
LIFT_Z      = 0.50
PRE_PLACE_Z = 0.50
PLACE_POS   = np.array([0.3, 0.0, 0.14])

# ── 그리퍼 ────────────────────────────────────────────────────────
GRIPPER_JOINT_INDICES = np.array([7, 8])
GRIPPER_OPEN   = np.array([0.04, 0.04])
GRIPPER_CLOSED = np.array([0.005, 0.005])
QUAT_DOWN      = np.array([0.0, 1.0, 0.0, 0.0])

# ── 초기 자세 ─────────────────────────────────────────────────────
JOINT_HOME = np.array([0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854, 0.04, 0.04])

# ── Top-View 카메라 ────────────────────────────────────────────────
TOP_CAM_PATH   = "/World/Camera"
CAM_TRANSLATE  = Gf.Vec3d(0.0, -0.06, 0.80)
CAM_ROTATE_DEG = Gf.Vec3f(0.0, 0.0, -90.0)
CAM_FOCAL_LEN  = 35.0
CAM_FOCUS_DIST = 34.0
CAM_CLIP_NEAR  = 0.01
CAM_CLIP_FAR   = 10.0

# ── Perspective 카메라 (Viewport 1) ──────────────────────────────
PERSP_EYE    = np.array([0.8, -1.1, 0.6])
PERSP_TARGET = np.array([0.0, -0.3, 0.25])

# ── ROS2 Graph 경로 (graph_dump.txt 기준) ─────────────────────────
CAM_GRAPH_PATH   = "/World/Graph/camera_graph"
CLOCK_GRAPH_PATH = "/World/Graph/clock_graph"
TF_GRAPH_PATH    = "/World/Graph/tf_graph"

# ── ROS2 공통 ─────────────────────────────────────────────────────
CAM_VIEWPORT_NAME = "Viewport 2"
CAM_DOMAIN_ID     = 50     # Context.inputs:domain_id
CAM_FRAME_ID      = "Camera"
CAM_WIDTH         = 640    # RenderProduct.inputs:width
CAM_HEIGHT        = 640    # RenderProduct.inputs:height
DISPLAY_WIDTH     = 1280   # 뷰포트 표시 해상도 (발행 해상도와 독립)
DISPLAY_HEIGHT    = 720
RGB_TOPIC         = "/camera/rgb"
DEPTH_TOPIC       = "/camera/depth"
INFO_TOPIC        = "camera_info"   # ※ 슬래시 없음 (dump 확인값)
CLOCK_TOPIC       = "clock"         # ※ 슬래시 없음 (dump 확인값)
TF_TOPIC          = "/tf"

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
#   BOX_X : 바나나 좌우 폭
#   BOX_Y : 그리핑 방향 깊이 (이 값으로 파지 타이트함 조정)
#   BOX_Z : 수직 높이
BOX_X      = 0.060
BOX_Y      = 0.032
BOX_Z      = 0.034
BOX_OFFSET = Gf.Vec3d(0.0, 0.0, -0.01706)

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

UsdPhysics.MassAPI.Apply(banana_prim).GetMassAttr().Set(0.1)
print("✅ 바나나 물리 설정 완료")

# ================================================================
# 그리퍼 Drive
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
print("✅ 그리퍼 Drive 완료")

# ================================================================
# 카메라 위치·방향
# ================================================================
cam_xform = UsdGeom.XformCommonAPI(cam_prim)
cam_xform.SetTranslate(CAM_TRANSLATE)
cam_xform.SetRotate(CAM_ROTATE_DEG)

# ================================================================
# ROS2 Action Graphs  — 워밍업 이후 생성  (graph_dump.txt 기준)
# ================================================================
# ※ 세 그래프 모두 PhysX 워밍업(render=False) 이후에 생성해야 함.
#    RunOnce는 시뮬레이션 첫 스텝에 발화하는데, render=False 구간에서
#    발화하면 렌더 시스템이 비활성 상태라 renderProductPath가 빈 값이 됨.
#    → 워밍업 완료 후 그래프를 생성해야 RunOnce가 render=True 프레임에서 발화.

def _build_camera_graph():
    """
    graph_dump.txt camera_graph 재현.

    exec 체인:
      OnPlaybackTick.outputs:tick   → RunOnce.inputs:execIn
      RunOnce.outputs:step          → RenderProduct.inputs:execIn      (첫 프레임 1회)
      RenderProduct.outputs:execOut → RGBPublish.inputs:execIn         (매 프레임)
      RenderProduct.outputs:execOut → DepthPublish.inputs:execIn
      RenderProduct.outputs:execOut → CameraInfoPublish.inputs:execIn
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": CAM_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick",    "omni.graph.action.OnPlaybackTick"),
                ("RunOnce",           "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
                ("Context",           "isaacsim.ros2.bridge.ROS2Context"),
                ("RenderProduct",     "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("RGBPublish",        "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublish",      "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
            ],
            keys.SET_VALUES: [
                # ── Context  (dump: domain_id=50, useDomainIDEnvVar=True) ─────
                ("Context.inputs:domain_id",                          CAM_DOMAIN_ID),
                ("Context.inputs:useDomainIDEnvVar",                   True),

                # ── RenderProduct  (dump: enabled=True, 640×640, cameraPrim) ──
                # cameraPrim: GUI(og_rtx_sensors.py)와 동일하게 SET_VALUES에 문자열로 전달
                ("RenderProduct.inputs:enabled",                       True),
                ("RenderProduct.inputs:width",                         CAM_WIDTH),
                ("RenderProduct.inputs:height",                        CAM_HEIGHT),
                ("RenderProduct.inputs:cameraPrim",                    TOP_CAM_PATH),

                # ── RGBPublish  (dump 설정값 전체) ────────────────────────────
                ("RGBPublish.inputs:type",                             "rgb"),
                ("RGBPublish.inputs:topicName",                        RGB_TOPIC),
                ("RGBPublish.inputs:frameId",                          CAM_FRAME_ID),
                ("RGBPublish.inputs:enabled",                          True),
                ("RGBPublish.inputs:frameSkipCount",                   0),
                ("RGBPublish.inputs:resetSimulationTimeOnStop",        True),
                ("RGBPublish.inputs:enableSemanticLabels",             False),
                ("RGBPublish.inputs:semanticLabelsTopicName",          "semantic_labels"),
                ("RGBPublish.inputs:useSystemTime",                    False),
                ("RGBPublish.inputs:stereoOffset",                     [0.0, 0.0]),

                # ── DepthPublish  (dump 설정값 전체) ──────────────────────────
                ("DepthPublish.inputs:type",                           "depth"),
                ("DepthPublish.inputs:topicName",                      DEPTH_TOPIC),
                ("DepthPublish.inputs:frameId",                        CAM_FRAME_ID),
                ("DepthPublish.inputs:enabled",                        True),
                ("DepthPublish.inputs:frameSkipCount",                 0),
                ("DepthPublish.inputs:resetSimulationTimeOnStop",      True),
                ("DepthPublish.inputs:enableSemanticLabels",           False),
                ("DepthPublish.inputs:semanticLabelsTopicName",        "semantic_labels"),
                ("DepthPublish.inputs:useSystemTime",                  False),
                ("DepthPublish.inputs:stereoOffset",                   [0.0, 0.0]),

                # ── CameraInfoPublish  (dump 설정값 전체) ─────────────────────
                ("CameraInfoPublish.inputs:topicName",                 INFO_TOPIC),
                ("CameraInfoPublish.inputs:frameId",                   CAM_FRAME_ID),
                ("CameraInfoPublish.inputs:enabled",                   True),
                ("CameraInfoPublish.inputs:frameSkipCount",            0),
                ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
                ("CameraInfoPublish.inputs:frameIdRight",              "sim_camera_right"),
                ("CameraInfoPublish.inputs:topicNameRight",            "camera_info_right"),
                ("CameraInfoPublish.inputs:useSystemTime",             False),
            ],
            keys.CONNECT: [
                # ── exec 체인 (graph_dump.txt CONNECT 그대로) ─────────────────
                ("OnPlaybackTick.outputs:tick",             "RunOnce.inputs:execIn"),
                ("RunOnce.outputs:step",                    "RenderProduct.inputs:execIn"),
                ("RenderProduct.outputs:execOut",           "RGBPublish.inputs:execIn"),
                ("RenderProduct.outputs:execOut",           "DepthPublish.inputs:execIn"),
                ("RenderProduct.outputs:execOut",           "CameraInfoPublish.inputs:execIn"),
                # ── renderProductPath → 세 Helper ─────────────────────────────
                ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
                ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
                # ── ROS2 컨텍스트 → 세 Helper ─────────────────────────────────
                ("Context.outputs:context",                 "RGBPublish.inputs:context"),
                ("Context.outputs:context",                 "DepthPublish.inputs:context"),
                ("Context.outputs:context",                 "CameraInfoPublish.inputs:context"),
            ],
        },
    )
    print(f"✅ Camera Graph 생성 완료  →  {CAM_GRAPH_PATH}")
    print(f"   RGB  토픽 : {RGB_TOPIC}  /  Depth 토픽 : {DEPTH_TOPIC}  /  Info 토픽 : {INFO_TOPIC}")


def _build_clock_graph():
    """
    graph_dump.txt clock_graph 재현.

    exec 체인:
      OnPlaybackTick.outputs:tick → PublishClock.inputs:execIn
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": CLOCK_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context",        "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock",   "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id",         CAM_DOMAIN_ID),
                ("Context.inputs:useDomainIDEnvVar",  True),
                ("PublishClock.inputs:topicName",     CLOCK_TOPIC),
                ("PublishClock.inputs:queueSize",     10),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick",               "PublishClock.inputs:execIn"),
                ("Context.outputs:context",                   "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime",        "PublishClock.inputs:timeStamp"),
            ],
        },
    )
    print(f"✅ Clock Graph 생성 완료  →  {CLOCK_GRAPH_PATH}  ({CLOCK_TOPIC})")


def _build_tf_graph():
    """
    graph_dump.txt tf_graph 재현.

    exec 체인:
      OnPlaybackTick.outputs:tick → PublisherTF.inputs:execIn
    targetPrims: set_targets() 로 연결 (다중 prim 관계)
    """
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": TF_GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context",        "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadSimTime",    "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublisherTF",    "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id",           CAM_DOMAIN_ID),
                ("Context.inputs:useDomainIDEnvVar",    True),
                ("PublisherTF.inputs:topicName",        TF_TOPIC),
                ("PublisherTF.inputs:staticPublisher",  False),
                ("PublisherTF.inputs:queueSize",        10),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick",               "PublisherTF.inputs:execIn"),
                ("Context.outputs:context",                   "PublisherTF.inputs:context"),
                ("ReadSimTime.outputs:simulationTime",        "PublisherTF.inputs:timeStamp"),
            ],
        },
    )
    # targetPrims: 다중 prim 관계 → set_targets() 로 연결
    set_targets(
        prim=stage.GetPrimAtPath(f"{TF_GRAPH_PATH}/PublisherTF"),
        attribute="inputs:targetPrims",
        target_prim_paths=["/World/Franka", "/World/Camera"],
    )
    print(f"✅ TF Graph 생성 완료  →  {TF_GRAPH_PATH}")
    print(f"   {TF_TOPIC}  토픽 발행  (대상: /World/Franka, /World/Camera)")


# ================================================================
# 뷰포트 설정  (5:5 분할)
# ================================================================
#   ┌─────────────────┬─────────────────┐
#   │  Viewport 1     │  Viewport 2     │
#   │  Perspective    │  Top-View Cam   │
#   │  (로봇 줌인)    │  (바나나 하향)  │
#   └─────────────────┴─────────────────┘
try:
    set_camera_view(eye=PERSP_EYE, target=PERSP_TARGET, camera_prim_path="/OmniverseKit_Persp")
    vp1_api = vp_util.get_viewport_from_window_name("Viewport")
    if vp1_api:
        vp1_api.camera_path = "/OmniverseKit_Persp"
    print(f"✅ Perspective 줌인  eye={PERSP_EYE}  target={PERSP_TARGET}")

    ui.Workspace.show_window(CAM_VIEWPORT_NAME)
    simulation_app.update()
    simulation_app.update()
    vp2_api = vp_util.get_viewport_from_window_name(CAM_VIEWPORT_NAME)
    if vp2_api:
        vp2_api.camera_path = TOP_CAM_PATH
        try:
            vp2_api.set_texture_resolution((DISPLAY_WIDTH, DISPLAY_HEIGHT))
        except Exception:
            pass
    print(f"✅ {CAM_VIEWPORT_NAME} 카메라 연결  →  {TOP_CAM_PATH}  (표시 {DISPLAY_WIDTH}×{DISPLAY_HEIGHT}  /  발행 {CAM_WIDTH}×{CAM_HEIGHT})")

    vp1_handle = ui.Workspace.get_window("Viewport")
    vp2_handle = ui.Workspace.get_window(CAM_VIEWPORT_NAME)
    if vp1_handle and vp2_handle:
        vp2_handle.dock_in(vp1_handle, ui.DockPosition.RIGHT, 0.5)
        print("✅ 5:5 분할 완료  (Left: Perspective | Right: Top-View)")
    else:
        print("⚠  WindowHandle 취득 실패 → 별도 창으로 표시")

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

# reset() 호출 시 JOINT_HOME을 복원 기준 자세로 등록
my_franka.set_joints_default_state(
    positions=JOINT_HOME,
    velocities=np.zeros(len(JOINT_HOME)),
)
print("✅ RMPflow 컨트롤러 / 관절 기본 자세 등록 완료")

# ================================================================
# PhysX 사전 워밍업
# ================================================================
PRE_WARMUP_FRAMES = 200
print(f"PhysX 워밍업 중... ({PRE_WARMUP_FRAMES} steps, render=False)")
for _ in range(PRE_WARMUP_FRAMES):
    my_world.step(render=False)
my_world.reset()
print("✅ PhysX 워밍업 완료")

# 워밍업 완료 후 세 그래프 생성
# → RunOnce가 render=True 첫 프레임에서 발화하도록 타이밍 보장
for _build_fn in [_build_camera_graph, _build_clock_graph, _build_tf_graph]:
    try:
        _build_fn()
    except Exception as e:
        print(f"⚠  Graph 생성 실패 ({_build_fn.__name__}): {e}")
        traceback.print_exc()

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
    """RMPflow로 엔드이펙터를 target(XYZ)으로 이동."""
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

WARMUP_FRAMES = 80

current_state = STATE_WARMUP
state_counter = 0
_pick_x = BANANA_POS[0]
_pick_y = BANANA_POS[1]

print("\n" + "=" * 60)
print("  ▶  Franka 바나나 Pick & Place  +  ROS2 Camera + Clock + TF")
print(f"  ROBOT_POS  = {ROBOT_POS}")
print(f"  BANANA_POS = {BANANA_POS}")
print(f"  PICK_Z     = {PICK_Z}  ← 튜닝 포인트")
print(f"  PLACE_POS  = {PLACE_POS}")
print(f"  domain_id  : {CAM_DOMAIN_ID}")
print(f"  발행 이미지: {CAM_WIDTH} × {CAM_HEIGHT}")
print(f"  RGB  토픽  : {RGB_TOPIC}")
print(f"  Depth 토픽 : {DEPTH_TOPIC}")
print(f"  Info 토픽  : {INFO_TOPIC}")
print(f"  Clock 토픽 : {CLOCK_TOPIC}")
print(f"  TF    토픽 : {TF_TOPIC}  (대상: /World/Franka, /World/Camera)")
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
                print(f"   수렴: panda_hand z={get_hand_pos()[2]:.4f}  ← PICK_Z 튜닝 참고")
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
