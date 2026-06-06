# -*- coding: utf-8 -*-
"""
[2b단계] ROS2 토픽 구독  —  Thread 방식
=========================================
목표: 외부 터미널에서 숫자(Int32)를 보내면
      Isaac Sim 안에 큐브가 생성된다.

실행
  ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/step02b_ros2_topic_subscribe_thread.py

토픽 발행 테스트 (다른 터미널)
  export ROS_DOMAIN_ID=50
  ros2 topic pub /spawn_cube std_msgs/msg/Int32 "data: 1" --once
  ros2 topic pub /spawn_cube std_msgs/msg/Int32 "data: 2" --once
  ros2 topic pub /spawn_cube std_msgs/msg/Int32 "data: 3" --once

확인 사항
  - 숫자 1을 보내면 → 빨간 큐브 생성
  - 숫자 2를 보내면 → 초록 큐브 생성
  - 숫자 3을 보내면 → 파란 큐브 생성
  - 같은 숫자를 또 보내면 → 추가로 또 생성됨
  - 1~3 이외의 숫자는 → 흰색 큐브 생성
"""

# ================================================================
# STEP 1. SimulationApp 초기화 (반드시 가장 먼저)
# ================================================================
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ================================================================
# STEP 2. 모듈 임포트
# ================================================================
import os
import threading
import numpy as np
from pxr import Gf, UsdPhysics, UsdGeom

from isaacsim.core.api import SimulationContext
from isaacsim.core.api.physics_context import PhysicsContext
from isaacsim.core.utils import prims, stage as stage_utils
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import nucleus

# ================================================================
# STEP 3. ROS2 브릿지 익스텐션 활성화
# ================================================================
# Isaac Sim 내부에서 ROS2 통신을 가능하게 하는 모듈을 로드한다
# 이 한 줄이 Isaac Sim ↔ ROS2 연결의 핵심이다
enable_extension("isaacsim.ros2.bridge")

# 익스텐션이 완전히 로드될 시간을 준다 (필수)
simulation_app.update()

# ================================================================
# STEP 4. rclpy 임포트 (브릿지 로드 이후에만 가능)
# ================================================================
# rclpy : ROS2 Python 클라이언트 라이브러리
# Isaac Sim 내부에 번들로 포함되어 있어 별도 설치 불필요
import rclpy
import rclpy.node
from std_msgs.msg import Int32

ROS_DOMAIN_ID = 50
os.environ.setdefault("ROS_DOMAIN_ID", str(ROS_DOMAIN_ID))

# ================================================================
# STEP 5. 시뮬레이션 컨텍스트 + 씬 구성
# ================================================================
simulation_context = SimulationContext(stage_units_in_meters=1.0)
PhysicsContext()

assets_root_path = nucleus.get_assets_root_path()
stage_utils.add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
    prim_path="/World/Environment"
)

print("🌐 그리드 환경 로드 완료")

# ================================================================
# STEP 6. 큐브 색상 & 위치 정의
# ================================================================
# 숫자별로 색상과 위치를 미리 정해둔다
# 위치는 X축 방향으로 나란히 배치
CUBE_CONFIG = {
    1: {"color": Gf.Vec3f(1.0, 0.0, 0.0), "position": np.array([-0.6, 0.0, 0.5]), "name": "빨간"},
    2: {"color": Gf.Vec3f(0.0, 1.0, 0.0), "position": np.array([ 0.0, 0.0, 0.5]), "name": "초록"},
    3: {"color": Gf.Vec3f(0.0, 0.0, 1.0), "position": np.array([ 0.6, 0.0, 0.5]), "name": "파란"},
}
DEFAULT_COLOR    = Gf.Vec3f(1.0, 1.0, 1.0)   # 1~3 이외 숫자 → 흰색
DEFAULT_POSITION = np.array([0.0, 0.6, 0.5])  # 1~3 이외 숫자 → 앞쪽에 생성

cube_counter = {"count": 0}

# ================================================================
# STEP 7. 큐브 생성 함수 정의
# ================================================================
def spawn_cube(number: int):
    """
    숫자를 받아서 해당 색상의 큐브를 생성한다.
    number : 외부에서 받은 Int32 값
    """
    cube_counter["count"] += 1
    cube_id = cube_counter["count"]

    config   = CUBE_CONFIG.get(number)
    color    = config["color"]    if config else DEFAULT_COLOR
    position = config["position"] if config else DEFAULT_POSITION
    label    = config["name"]     if config else "흰색"

    prim_path = f"/World/Cube_{cube_id:03d}"

    cube_prim = prims.create_prim(
        prim_path=prim_path,
        prim_type="Cube",
        position=position + np.array([0.0, 0.0, cube_id * 0.01]),
        scale=np.array([0.15, 0.15, 0.15]),
    )
    UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    UsdPhysics.CollisionAPI.Apply(cube_prim)
    UsdGeom.Cube(cube_prim).GetDisplayColorAttr().Set([color])

    print(f"📦 [{cube_id:03d}] 숫자 {number} 수신 → {label} 큐브 생성 ({prim_path})")

# ================================================================
# STEP 8. 공유 변수 & Lock  +  구독 노드 & spin 스레드
# ================================================================
_received = None              # 수신된 Int32 값
_lock     = threading.Lock()  # 공유 변수 보호

def _topic_callback(msg: Int32):
    """토픽 수신 시 spin 스레드에서 자동 호출됨."""
    global _received
    with _lock:
        _received = msg.data
    print(f"📨 토픽 수신: /spawn_cube → data={msg.data}")

rclpy.init()
ros_node = rclpy.node.Node("isaac_spawn_node")
ros_node.create_subscription(Int32, "/spawn_cube", _topic_callback, 10)

# daemon=True → Isaac Sim 종료 시 이 스레드도 자동으로 함께 종료됨
_ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
_ros_thread.start()

print("✅ ROS2 구독자 생성 완료: /spawn_cube (std_msgs/Int32)")
print(f"✅ spin 스레드 시작  (domain_id={ROS_DOMAIN_ID})")

# ================================================================
# STEP 9. 시뮬레이션 시작
# ================================================================
simulation_context.initialize_physics()
simulation_context.play()

print("\n" + "=" * 55)
print("  ▶  시뮬레이션 시작")
print("  📡 /spawn_cube 토픽 대기 중...")
print("  터미널2에서 아래 명령어를 입력하세요:")
print("  ros2 topic pub /spawn_cube std_msgs/msg/Int32 'data: 1' --once")
print("=" * 55 + "\n")

# ================================================================
# STEP 10. 메인 루프
# ================================================================
while simulation_app.is_running():
    simulation_context.step(render=True)

    with _lock:
        number    = _received
        _received = None

    if number is not None:
        spawn_cube(number)

# ================================================================
# STEP 11. 종료
# ================================================================
print("\n⏹  시뮬레이션 종료")
ros_node.destroy_node()
rclpy.shutdown()
simulation_context.stop()
simulation_app.close()
