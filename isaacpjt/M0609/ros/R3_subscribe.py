"""[R3 예제] 토픽을 구독해서 로봇팔 움직이기

실행: isaac_python R3_subscribe.py
      창에서 Play(▶) 를 누르고, 새 터미널에서 /arm_command 토픽으로 left, right, home 을 보낸다
      ros2 topic pub --once /arm_command std_msgs/msg/String "{data: left}"
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
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.types import ArticulationAction

# (가) ROS2 Bridge 확장을 켠 뒤에 rclpy 를 import 한다 (Isaac Sim 에 들어 있는 rclpy)
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
import rclpy
from std_msgs.msg import String

SCENE_USD = Path(__file__).resolve().parents[1] / "scenes" / "pick_place_cube.usd"  # GUI 강의에서 만든 기본 씬
HOME_DEG = [0, 0, 90, 0, 90, 0]                       # 홈 자세 (deg)
COMMANDS = {"left": 45.0, "right": -45.0, "home": 0.0}  # 명령 -> joint_1 목표 각도 (deg)

# (나) 작업 씬 열기
open_stage(str(SCENE_USD))
world = World(stage_units_in_meters=1.0)
robot = world.scene.add(SingleArticulation(prim_path="/World/m0609_rg2_d455", name="m0609"))

# (다) ROS2 노드와 구독 만들기: /arm_command 로 문자열이 오면 on_command 가 불린다
target_deg = None  # 콜백이 받아 둔 joint_1 목표 (deg). 아직 받은 명령이 없으면 None


def on_command(msg):
    """명령을 받으면 목표 각도만 저장한다. 로봇은 메인 루프에서 움직인다"""
    global target_deg
    command = msg.data.strip()
    if command in COMMANDS:
        target_deg = COMMANDS[command]
        print(f"명령 받음: {command} -> joint_1 {target_deg:.0f} deg")
    else:
        print(f"모르는 명령: {command} (left, right, home 중 하나)")


rclpy.init()
node = rclpy.create_node("arm_commander")                             # ROS2 노드 이름
node.create_subscription(String, "arm_command", on_command, 10)       # (메시지 타입, 토픽 이름, 콜백, 큐 크기)

# (라) 홈 자세로 순간이동하고, 멈춘 상태로 기다린다
world.reset()
arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]  # 팔 관절 6개가 관절 배열의 몇 번째인지
home = robot.get_joint_positions()
home[arm] = np.radians(HOME_DEG)
robot.set_joint_positions(home)
world.step(render=True)  # 한 프레임 진행해서 순간이동한 자세를 화면에 반영
world.pause()            # 화면 왼쪽의 Play(▶) 를 누르면 시작
print("Play 를 누르고 /arm_command 로 left, right, home 을 보낸다")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
while simulation_app.is_running():
    world.step(render=True)
    rclpy.spin_once(node, timeout_sec=0.0)  # 도착한 메시지가 있으면 콜백 실행 (기다리지 않음)
    if world.is_playing() and target_deg is not None:
        goal = home.copy()
        goal[arm[0]] = np.radians(target_deg)  # joint_1 만 목표 각도로
        robot.apply_action(ArticulationAction(joint_positions=goal))
        target_deg = None  # 한 번 보냈으면 비운다 (모터가 목표까지 계속 움직인다)

# ============================================================
# 4. 종료
# ============================================================
node.destroy_node()
rclpy.shutdown()
simulation_app.close()
