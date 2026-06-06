# -*- coding: utf-8 -*-
"""
Isaac Sim 5.1.0 - Action Graph형 ROS 2 토픽 수신 및 동적 큐브 생성
"""

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import SimulationContext
from isaacsim.core.api.objects.ground_plane import GroundPlane
from isaacsim.core.api.physics_context import PhysicsContext
from isaacsim.core.utils import extensions, prims
from pxr import Gf
import omni.graph.core as og

# ✅ ROS2 브릿지 익스텐션 활성화
extensions.enable_extension("isaacsim.ros2.bridge")

# ✅ 익스텐션이 완전히 로드될 때까지 앱 업데이트 (핵심 수정)
for _ in range(5):
    simulation_app.update()

# 월드 컨텍스트 세팅
sim_context = SimulationContext(stage_units_in_meters=1.0)

# 바닥 평면 생성
PhysicsContext()
GroundPlane(
    prim_path="/World/groundPlane",
    size=10,
    color=np.array([0.5, 0.5, 0.5])
)

# ═══════════════════════════════════════════════════════
#  큐브 스폰 함수
# ═══════════════════════════════════════════════════════
cube_count = 0

def spawn_blue_cube():
    global cube_count
    cube_count += 1
    cube_name = f"twin_blue_cube_{cube_count}"
    cube_path = f"/World/DroppedCubes/{cube_name}"
    spawn_pos = np.array([0.0, 0.2 * cube_count, 0.8])
    print(f"★ [Digital Twin] {cube_name} 생성 위치: {spawn_pos}")

    cube_prim = prims.create_prim(
        prim_path=cube_path,
        prim_type="Cube",
        position=spawn_pos,
        scale=np.array([0.05, 0.05, 0.05])
    )

    from pxr import UsdPhysics, UsdGeom
    UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    UsdGeom.Cube(cube_prim).GetDisplayColorAttr().Set([Gf.Vec3f(0.0, 0.0, 1.0)])

# ═══════════════════════════════════════════════════════
#  Action Graph 구성
# ═══════════════════════════════════════════════════════
GRAPH_PATH = "/ActionGraph"
graph_ok = False  # ✅ 그래프 생성 성공 여부 플래그

try:
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick",            "omni.graph.action.OnTick"),
                ("Context",           "isaacsim.ros2.bridge.ROS2Context"),
                ("SubscribeOrder",    "isaacsim.ros2.bridge.ROS2SubscribeGeneric"),
                ("ReadIntAttribute",  "omni.graph.nodes.ReadComponentAttribute"),
                ("CompareOrder",      "omni.graph.action.CompareInt"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnTick.outputs:tick",          "SubscribeOrder.inputs:execIn"),
                ("Context.outputs:context",      "SubscribeOrder.inputs:context"),
                ("SubscribeOrder.outputs:execOut","ReadIntAttribute.inputs:execIn"),
                ("SubscribeOrder.outputs:buffer", "ReadIntAttribute.inputs:buffer"),
                ("ReadIntAttribute.outputs:execOut","CompareOrder.inputs:execIn"),
                ("ReadIntAttribute.outputs:value", "CompareOrder.inputs:value1"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubscribeOrder.inputs:topicName",    "/order"),
                ("SubscribeOrder.inputs:messageName",  "Int32"),
                ("SubscribeOrder.inputs:packageName",  "std_msgs"),
                ("ReadIntAttribute.inputs:attributeName", "data"),
                ("CompareOrder.inputs:value2", 1),
            ],
        },
    )
    graph_ok = True
    print("✅ Action Graph 생성 성공")
except Exception as e:
    print(f"❌ Action Graph 생성 실패: {e}")

# ═══════════════════════════════════════════════════════
#  시뮬레이션 루프
# ═══════════════════════════════════════════════════════
sim_context.initialize_physics()
sim_context.play()

print("\n" + "="*60)
print("  [Clean Platform] ROS2 Bridge 기동 완료 (5.1.0 Spec)")
print("="*60 + "\n")

last_trigger_state = False

while simulation_app.is_running():
    sim_context.step(render=True)

    if not graph_ok:
        continue  # ✅ 그래프가 없으면 루프만 돌고 스킵

    try:
        is_equal = og.Controller.get(
            og.Controller.attribute(GRAPH_PATH + "/CompareOrder.outputs:isEqual")
        )
        if is_equal and not last_trigger_state:
            spawn_blue_cube()
            last_trigger_state = True
        elif not is_equal:
            last_trigger_state = False
    except Exception as e:
        print(f"[Warning] OmniGraph 읽기 실패: {e}")

sim_context.stop()
simulation_app.close()