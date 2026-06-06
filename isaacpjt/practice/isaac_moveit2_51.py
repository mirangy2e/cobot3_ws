# -*- coding: utf-8 -*-
import sys
import os
import numpy as np
import carb

# Isaac Sim 5.1+ Imports
from isaacsim import SimulationApp

# Configuration
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

# Core & Utils Imports
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, prims, rotations, stage, viewports
from isaacsim.core.utils.prims import set_targets
from isaacsim.storage.native import nucleus

# Action Graph & ROS2 Bridge
import omni.graph.core as og
from pxr import Gf, UsdGeom

# Enable ROS2 Bridge Extension
extensions.enable_extension("isaacsim.ros2.bridge")

# --- Constants ---
FRANKA_STAGE_PATH = "/Franka"
# Isaac Sim 5.x에서는 표준 Franka 에셋 사용 권장 (필요시 경로 수정 가능)
# https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/FrankaRobotics/FrankaPanda
# FRANKA_USD_PATH = "/Isaac/Robots/Franka/franka.usd" 
FRANKA_USD_PATH = "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd" 
BACKGROUND_STAGE_PATH = "/background"
BACKGROUND_USD_PATH = "/Isaac/Environments/Simple_Room/simple_room.usd"
GRAPH_PATH = "/ActionGraph"
REALSENSE_VIEWPORT_NAME = "realsense_viewport"

# 카메라를 부착할 경로 (Panda Hand)
CAMERA_PARENT_PATH = f"{FRANKA_STAGE_PATH}/panda_hand"
CAMERA_PRIM_PATH = f"{CAMERA_PARENT_PATH}/realsense_camera"

simulation_context = SimulationContext(stage_units_in_meters=1.0)

# --- Asset Loading ---
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# Set Main Camera View
viewports.set_camera_view(eye=np.array([1.2, 1.2, 0.8]), target=np.array([0, 0, 0.5]))

# Load Environment
stage.add_reference_to_stage(
    assets_root_path + BACKGROUND_USD_PATH, BACKGROUND_STAGE_PATH
)

# Load Franka Robot
prims.create_prim(
    FRANKA_STAGE_PATH,
    "Xform",
    position=np.array([0, -0.64, 0]),
    orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(0, 0, 1), 90)),
    usd_path=assets_root_path + FRANKA_USD_PATH,
)

# [FIX] Manually Create Camera Prim to avoid "invalid prim" error
# 에셋 내부에 카메라가 없더라도 강제로 생성하여 부착합니다.
if not stage.get_current_stage().GetPrimAtPath(CAMERA_PRIM_PATH).IsValid():
    # 1. 카메라 Prim 정의
    camera_prim = UsdGeom.Camera.Define(stage.get_current_stage(), CAMERA_PRIM_PATH)
    
    # Clipping Range 설정 (예: Near=0.01, Far=10000000.0)
    clipping_range = Gf.Vec2f(0.01, 10000000.0)
    camera_prim.CreateClippingRangeAttr().Set(clipping_range)

    # 2. 위치 및 회전 설정 (Hand 좌표계 기준)
    # Realsense 장착 위치와 유사하게 오프셋 설정 (필요에 따라 튜닝)
    xform = UsdGeom.Xformable(camera_prim)
    xform.AddTranslateOp().Set(Gf.Vec3d(0.05, 0.0, 0.05))  # 손목 앞쪽으로 약간 이동
    # 카메라가 앞을 보도록 회전 (Y축 -90도 회전 등 좌표축에 맞게 조정)
    xform.AddRotateXYZOp().Set(Gf.Vec3d(180, 0, 0)) 

# Add props
props = [
    ("/cracker_box", "/Isaac/Props/YCB/Axis_Aligned_Physics/003_cracker_box.usd", [-0.2, -0.25, 0.15], [1, 0, 0]),
    ("/sugar_box", "/Isaac/Props/YCB/Axis_Aligned_Physics/004_sugar_box.usd", [-0.07, -0.25, 0.1], [0, 1, 0]),
    ("/soup_can", "/Isaac/Props/YCB/Axis_Aligned_Physics/005_tomato_soup_can.usd", [0.1, -0.25, 0.10], [1, 0, 0]),
    ("/mustard_bottle", "/Isaac/Props/YCB/Axis_Aligned_Physics/006_mustard_bottle.usd", [0.0, 0.15, 0.12], [1, 0, 0]),
]

for name, usd, pos, rot_axis in props:
    prims.create_prim(
        name,
        "Xform",
        position=np.array(pos),
        orientation=rotations.gf_rotation_to_np_array(Gf.Rotation(Gf.Vec3d(*rot_axis), -90)),
        usd_path=assets_root_path + usd,
    )

simulation_app.update()

# --- ROS 2 Configuration ---
try:
    ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", 0))
    print("Using ROS_DOMAIN_ID: ", ros_domain_id)
except ValueError:
    ros_domain_id = 0

# --- Action Graph Construction (Isaac Sim 5.1 Style) ---
try:
    og_keys_set_values = [
        ("Context.inputs:domain_id", ros_domain_id),
        ("ArticulationController.inputs:robotPath", FRANKA_STAGE_PATH),
        ("PublishJointState.inputs:topicName", "isaac_joint_states"),
        ("SubscribeJointState.inputs:topicName", "isaac_joint_commands"),
        ("createViewport.inputs:name", REALSENSE_VIEWPORT_NAME),
        ("createViewport.inputs:viewportId", 1),
        ("cameraHelperRgb.inputs:frameId", "sim_camera"),
        ("cameraHelperRgb.inputs:topicName", "rgb"),
        ("cameraHelperRgb.inputs:type", "rgb"),
        ("cameraHelperInfo.inputs:frameId", "sim_camera"),
        ("cameraHelperInfo.inputs:topicName", "camera_info"),
        ("cameraHelperDepth.inputs:frameId", "sim_camera"),
        ("cameraHelperDepth.inputs:topicName", "depth"),
        ("cameraHelperDepth.inputs:type", "depth"),
    ]

    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnImpulseEvent", "omni.graph.action.OnImpulseEvent"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("OnTick", "omni.graph.action.OnTick"),
                ("createViewport", "isaacsim.core.nodes.IsaacCreateViewport"),
                ("getRenderProduct", "isaacsim.core.nodes.IsaacGetViewportRenderProduct"),
                ("setCamera", "isaacsim.core.nodes.IsaacSetCameraOnRenderProduct"),
                ("cameraHelperRgb", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("cameraHelperInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ("cameraHelperDepth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnImpulseEvent.outputs:execOut", "PublishJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "PublishClock.inputs:execIn"),
                ("OnImpulseEvent.outputs:execOut", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
                ("OnTick.outputs:tick", "createViewport.inputs:execIn"),
                ("createViewport.outputs:execOut", "getRenderProduct.inputs:execIn"),
                ("createViewport.outputs:viewport", "getRenderProduct.inputs:viewport"),
                ("getRenderProduct.outputs:execOut", "setCamera.inputs:execIn"),
                ("getRenderProduct.outputs:renderProductPath", "setCamera.inputs:renderProductPath"),
                ("setCamera.outputs:execOut", "cameraHelperRgb.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperInfo.inputs:execIn"),
                ("setCamera.outputs:execOut", "cameraHelperDepth.inputs:execIn"),
                ("Context.outputs:context", "cameraHelperRgb.inputs:context"),
                ("Context.outputs:context", "cameraHelperInfo.inputs:context"),
                ("Context.outputs:context", "cameraHelperDepth.inputs:context"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperRgb.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperInfo.inputs:renderProductPath"),
                ("getRenderProduct.outputs:renderProductPath", "cameraHelperDepth.inputs:renderProductPath"),
            ],
            og.Controller.Keys.SET_VALUES: og_keys_set_values,
        },
    )
except Exception as e:
    print(f"Graph generation error: {e}")

simulation_app.update()

# Setting the target prim for Publish JointState
set_targets(
    prim=stage.get_current_stage().GetPrimAtPath("/ActionGraph/PublishJointState"),
    attribute="inputs:targetPrim",
    target_prim_paths=[FRANKA_STAGE_PATH],
)

# [FIX] Camera Parameter Setup
# 이제 위에서 카메라를 명시적으로 생성했으므로, 이 코드는 안전하게 실행됩니다.
realsense_prim = UsdGeom.Camera(
    stage.get_current_stage().GetPrimAtPath(CAMERA_PRIM_PATH)
)
# Check validity just in case
if realsense_prim.GetPrim().IsValid():
    realsense_prim.GetHorizontalApertureAttr().Set(20.955)
    realsense_prim.GetVerticalApertureAttr().Set(15.7)
    realsense_prim.GetFocalLengthAttr().Set(18.8)
    realsense_prim.GetFocusDistanceAttr().Set(400)
else:
    print(f"Error: Camera prim at {CAMERA_PRIM_PATH} is still invalid.")

# Link camera to Action Graph
set_targets(
    prim=stage.get_current_stage().GetPrimAtPath(GRAPH_PATH + "/setCamera"),
    attribute="inputs:cameraPrim",
    target_prim_paths=[CAMERA_PRIM_PATH],
)

# Warmup
simulation_app.update()
simulation_app.update()

simulation_context.initialize_physics()
simulation_context.play()
simulation_app.update()

# Dock Viewport
try:
    viewport = omni.ui.Workspace.get_window("Viewport")
    rs_viewport = omni.ui.Workspace.get_window(REALSENSE_VIEWPORT_NAME)
    if viewport and rs_viewport:
        rs_viewport.dock_in(viewport, omni.ui.DockPosition.RIGHT)
except Exception as e:
    print(f"Viewport docking error (safe to ignore if headless): {e}")

while simulation_app.is_running():
    simulation_context.step(render=True)
    
    # Tick Action Graph
    og.Controller.set(
        og.Controller.attribute("/ActionGraph/OnImpulseEvent.state:enableImpulse"), True
    )

simulation_context.stop()
simulation_app.close()