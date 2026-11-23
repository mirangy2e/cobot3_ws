"""[R5 예제] TF 를 코드로 발행: 로봇 링크와 손목 카메라의 위치

실행: isaac_python R5_tf.py
      창에서 Play(▶) 를 누르면 /tf, /clock 이 발행되고, 로봇팔이 좌우로 움직인다
      Window > TF Viewer 에서 Root Frame 을 world 로 고르면 TF 가 뷰포트에 그려진다
      새 터미널에서 ros2 run tf2_ros tf2_echo base_link Camera_OmniVision_OV9782_Color 로 확인한다
"""

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
from isaacsim.core.utils.types import ArticulationAction

SCENE_USD = Path(__file__).resolve().parents[1] / "scenes" / "pick_place_cube.usd"  # GUI 강의에서 만든 기본 씬
GRAPH_PATH = "/World/Graph/ROS_TF"  # TF 그래프 위치
ROBOT_BASE = "/World/m0609_rg2_d455/base_link"  # 로봇 링크 전체를 발행할 시작 prim
BRACKET_PRIM = "/World/m0609_rg2_d455/angle_bracket"  # 손목 카메라의 부모 프레임
CAMERA_PRIM = "/World/m0609_rg2_d455/angle_bracket/realsense_d455/RSD455/Camera_OmniVision_OV9782_Color"  # 손목 컬러 카메라
HOME_DEG = [0, 0, 90, 0, 90, 0]  # 홈 자세 (deg). 카메라가 바닥을 내려다본다
SWING_DEG = 30.0                 # joint_1 을 좌우로 돌릴 각도 (deg)
SWING_SEC = 3.0                  # 이 시간(시뮬레이션 초)마다 방향을 바꾼다

# (가) ROS2 Bridge 확장 켜기: ROS2 노드를 쓰기 전에 켠다
enable_extension("isaacsim.ros2.bridge")
enable_extension("isaacsim.ros2.tf_viewer")  # TF Viewer: 받은 /tf 를 뷰포트에 그려 준다 (Window > TF Viewer)
simulation_app.update()

# (나) 작업 씬 열기
open_stage(str(SCENE_USD))
world = World(stage_units_in_meters=1.0)
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))

# (다) TF Graph 만들기: GUI 의 Tools > Robotics > ROS 2 OmniGraphs > TF Publisher 와 같은 노드 4개 + 카메라 TF 발행 노드 + /clock 발행 노드
og.Controller.edit(
    {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},  # execution = Action Graph
    {
        og.Controller.Keys.CREATE_NODES: [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),              # Play 중 매 프레임 실행 신호
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),                     # ROS2 통신 설정 (Domain ID)
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),      # 시뮬레이션 시간 읽기
            ("PublisherTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),    # 로봇 링크 /tf 발행
            ("PublisherCameraTF", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),  # 손목 카메라 /tf 발행
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),           # /clock 발행 (외부 노드의 use_sim_time 용)
        ],
        og.Controller.Keys.CONNECT: [
            ("OnPlaybackTick.outputs:tick", "PublisherTF.inputs:execIn"),       # 실행 신호
            ("Context.outputs:context", "PublisherTF.inputs:context"),          # 통신 설정
            ("ReadSimTime.outputs:simulationTime", "PublisherTF.inputs:timeStamp"),  # TF 의 시간 = 시뮬레이션 시간
            ("OnPlaybackTick.outputs:tick", "PublisherCameraTF.inputs:execIn"),
            ("Context.outputs:context", "PublisherCameraTF.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublisherCameraTF.inputs:timeStamp"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),  # /clock 도 같은 시뮬레이션 시간
        ],
        og.Controller.Keys.SET_VALUES: [
            ("PublisherTF.inputs:targetPrims", [ROBOT_BASE]),                    # 로봇 링크 (부모: world)
            ("PublisherCameraTF.inputs:parentPrim", [BRACKET_PRIM]),             # 카메라의 부모: angle_bracket
            ("PublisherCameraTF.inputs:targetPrims", [CAMERA_PRIM]),             # 손목 카메라
        ],
    },
)
print("TF Graph 생성:", GRAPH_PATH)

# (라) 홈 자세로 순간이동하고, 멈춘 상태로 기다린다
world.reset()
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]  # 팔 관절 6개가 관절 배열의 몇 번째인지
home = robot.get_joint_positions()
home[arm] = np.radians(HOME_DEG)
robot.set_joint_positions(home)
world.step(render=True)  # 한 프레임 진행해서 순간이동한 자세를 화면에 반영
world.pause()            # 화면 왼쪽의 Play(▶) 를 누르면 시작
print("Play 를 누르면 /tf, /clock 을 발행하고, joint_1 이 좌우로 움직인다")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
direction = 0  # 지금 보낸 방향 (1: 왼쪽, -1: 오른쪽, 0: 아직 안 보냄)
while simulation_app.is_running():
    world.step(render=True)
    if not world.is_playing():
        continue
    want = 1 if int(world.current_time // SWING_SEC) % 2 == 0 else -1  # SWING_SEC 마다 방향을 바꾼다
    if want != direction:
        direction = want
        goal = home.copy()
        goal[arm[0]] = np.radians(SWING_DEG * direction)  # joint_1 만 좌우로
        robot.apply_action(ArticulationAction(joint_positions=goal))

# ============================================================
# 4. 종료
# ============================================================
simulation_app.close()
