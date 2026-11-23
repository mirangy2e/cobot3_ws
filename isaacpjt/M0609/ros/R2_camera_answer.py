"""[R2 연습 정답] 손목 카메라의 컬러 영상(/rgb)과 왼쪽 영상(/rgb_left) 함께 발행"""

from pathlib import Path

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import numpy as np
import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage

SCENE_USD = Path(__file__).resolve().parents[1] / "scenes" / "pick_place_cube.usd"  # GUI 강의에서 만든 기본 씬
GRAPH_PATH = "/World/Graph/camera_graph"  # GUI 강의와 같은 위치
CAMERA_PRIM = "/World/m0609_rg2_d455/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"  # 손목 컬러 카메라
LEFT_CAMERA_PRIM = "/World/m0609_rg2_d455/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Left"  # 왼쪽 카메라
WIDTH, HEIGHT = 640, 640  # 발행할 영상 크기 (픽셀)
FRAME_ID = "Camera"       # 메시지 header 의 frame_id
CAMERA_LOOK_DEG = [0, 0, 90, 0, 90, 0]  # 카메라가 바닥의 큐브를 내려다보는 팔 자세 (joint_3, joint_5 = 90 deg)

# (가) ROS2 Bridge 확장 켜기: ROS2 노드를 쓰기 전에 켠다
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# (나) 작업 씬 열기: 로봇, 바닥, 큐브가 들어 있는 기본 씬
open_stage(str(SCENE_USD))
world = World(stage_units_in_meters=1.0)
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))

# (다) Camera Graph 만들기: GUI 의 Tools > Robotics > ROS 2 OmniGraphs > Camera 와 같은 노드 7개
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},  # execution = Action Graph
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),                       # Play 중 매 프레임 실행 신호
            ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),             # 첫 프레임에만 다음 노드 실행
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),            # 카메라의 렌더 결과 만들기
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),                              # ROS2 통신 설정 (Domain ID)
            ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),           # /camera_info 발행
            ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),                      # /rgb 발행
            ("DepthPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),                    # /depth 발행
            ("RenderProductLeft", "isaacsim.core.nodes.IsaacCreateRenderProduct"),        # 왼쪽 카메라의 렌더 결과
            ("RGBLeftPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),                  # /rgb_left 발행
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),                     # 실행 신호 -> 첫 프레임만
            ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),                      # 첫 프레임에 렌더 결과 만들기
            ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),         # 렌더 결과를 만든 뒤 발행 노드 실행
            ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "DepthPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),  # 어느 렌더 결과를
            ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
            ("RenderProduct.outputs:renderProductPath", "DepthPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "CameraInfoPublish.inputs:context"),               # 통신 설정
            ("Context.outputs:context", "RGBPublish.inputs:context"),
            ("Context.outputs:context", "DepthPublish.inputs:context"),
            ("RunOnce.outputs:step", "RenderProductLeft.inputs:execIn"),                  # 왼쪽 카메라도 첫 프레임에 렌더 결과 만들기
            ("RenderProductLeft.outputs:execOut", "RGBLeftPublish.inputs:execIn"),
            ("RenderProductLeft.outputs:renderProductPath", "RGBLeftPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "RGBLeftPublish.inputs:context"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("RenderProduct.inputs:cameraPrim", CAMERA_PRIM),                             # 어느 카메라로 찍을지
            ("RenderProduct.inputs:width", WIDTH),                                        # 영상 크기
            ("RenderProduct.inputs:height", HEIGHT),
            ("CameraInfoPublish.inputs:topicName", "camera_info"),                        # 토픽 이름
            ("CameraInfoPublish.inputs:frameId", FRAME_ID),
            ("RGBPublish.inputs:type", "rgb"),                                            # 발행할 데이터 종류
            ("RGBPublish.inputs:topicName", "rgb"),
            ("RGBPublish.inputs:frameId", FRAME_ID),
            ("DepthPublish.inputs:type", "depth"),
            ("DepthPublish.inputs:topicName", "depth"),
            ("DepthPublish.inputs:frameId", FRAME_ID),
            ("RenderProductLeft.inputs:cameraPrim", LEFT_CAMERA_PRIM),
            ("RenderProductLeft.inputs:width", WIDTH),
            ("RenderProductLeft.inputs:height", HEIGHT),
            ("RGBLeftPublish.inputs:type", "rgb"),
            ("RGBLeftPublish.inputs:topicName", "rgb_left"),
            ("RGBLeftPublish.inputs:frameId", "CameraLeft"),
        ],
    },
)
print("Camera Graph 생성:", GRAPH_PATH)

# (라) 카메라가 큐브를 내려다보는 자세로 순간이동하고, 멈춘 상태로 기다린다
world.reset()
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]  # 팔 관절 6개가 관절 배열의 몇 번째인지
pose = robot.get_joint_positions()
pose[arm] = np.radians(CAMERA_LOOK_DEG)
robot.set_joint_positions(pose)
world.step(render=True)  # 한 프레임 진행해서 순간이동한 자세를 화면에 반영
world.pause()            # 화면 왼쪽의 Play(▶) 를 누르면 영상 발행 시작
print("Play 를 누르면 /rgb, /depth, /camera_info, /rgb_left 를 발행한다")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
while simulation_app.is_running():
    world.step(render=True)

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
