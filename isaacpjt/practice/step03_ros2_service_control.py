# -*- coding: utf-8 -*-
"""
[3단계] Isaac Sim 5.1.0 + ROS2 서비스 연동
===========================================
목표: 외부 터미널에서 서비스를 호출하면
      Isaac Sim이 반응하고 결과를 응답으로 돌려준다.

      토픽(2단계) : 단방향 - 보내고 끝
      서비스(3단계): 양방향 - 보내면 응답이 온다 ← 오늘 배울 것

실행 방법:
    # 터미널 1 - Isaac Sim 실행
    export ROS_DOMAIN_ID=50
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
    export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/home/rokey/isaacsim/exts/isaacsim.ros2.bridge/humble/lib
    ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/step03_ros2_service_control.py

    # 터미널 2 - 서비스 호출
    export ROS_DOMAIN_ID=50
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    export FASTRTPS_DEFAULT_PROFILES_FILE=~/.ros/fastdds.xml
    source /opt/ros/humble/setup.bash

    # 큐브 생성 (data: true)
    ros2 service call /cube_control std_srvs/srv/SetBool "data: true"

    # 전체 리셋 - 모든 큐브 삭제 (data: false)
    ros2 service call /cube_control std_srvs/srv/SetBool "data: false"

확인 사항:
    - true  호출 → 큐브 생성 + "큐브 생성 완료" 응답
    - false 호출 → 모든 큐브 삭제 + "리셋 완료" 응답
    - 터미널 2에서 응답 메시지가 출력된다 (토픽과의 차이!)
"""

# ================================================================
# STEP 1. SimulationApp 초기화 (반드시 가장 먼저)
# ================================================================
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ================================================================
# STEP 2. 모듈 임포트
# ================================================================
import numpy as np
import random
from pxr import Gf, UsdPhysics, UsdGeom, Sdf

from isaacsim.core.api import SimulationContext
from isaacsim.core.api.physics_context import PhysicsContext
from isaacsim.core.utils import prims, stage as stage_utils
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.storage.native import nucleus

# ================================================================
# STEP 3. ROS2 브릿지 익스텐션 활성화
# ================================================================
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

# ================================================================
# STEP 4. rclpy 임포트 (브릿지 로드 이후에만 가능)
# ================================================================
import rclpy
from std_srvs.srv import SetBool   # true/false 를 받고 응답을 돌려주는 표준 서비스

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
# STEP 6. 생성된 큐브 경로를 추적하는 리스트
# ================================================================
# 나중에 리셋할 때 어떤 큐브가 있는지 알아야 삭제할 수 있다
spawned_cubes = []
cube_counter  = {"count": 0}

# ================================================================
# STEP 7. 큐브 생성 함수
# ================================================================
def spawn_cube() -> str:
    """
    랜덤 위치에 랜덤 색상의 큐브를 생성한다.
    반환값: 생성 결과 메시지 (서비스 응답으로 전달됨)
    """
    cube_counter["count"] += 1
    cube_id   = cube_counter["count"]
    prim_path = f"/World/SpawnedCubes/Cube_{cube_id:03d}"

    # 랜덤 위치: x, y 는 -0.8 ~ 0.8 범위, z 는 0.3 ~ 1.0 높이
    position = np.array([
        random.uniform(-0.8, 0.8),
        random.uniform(-0.8, 0.8),
        random.uniform( 0.3, 1.0),
    ])

    # 랜덤 색상
    color = Gf.Vec3f(
        random.uniform(0.2, 1.0),
        random.uniform(0.2, 1.0),
        random.uniform(0.2, 1.0),
    )

    # 큐브 생성
    cube_prim = prims.create_prim(
        prim_path=prim_path,
        prim_type="Cube",
        position=position,
        scale=np.array([0.12, 0.12, 0.12]),
    )

    # 물리 속성 적용
    UsdPhysics.RigidBodyAPI.Apply(cube_prim)
    UsdPhysics.CollisionAPI.Apply(cube_prim)

    # 색상 적용
    UsdGeom.Cube(cube_prim).GetDisplayColorAttr().Set([color])

    # 생성된 큐브 경로를 리스트에 저장 (나중에 삭제할 때 사용)
    spawned_cubes.append(prim_path)

    msg = f"큐브 생성 완료: {prim_path} | 위치: ({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f})"
    print(f"📦 {msg}")
    return msg

# ================================================================
# STEP 8. 전체 리셋 함수
# ================================================================
def reset_all_cubes() -> str:
    """
    생성된 모든 큐브를 삭제한다.
    반환값: 삭제 결과 메시지 (서비스 응답으로 전달됨)
    """
    count = len(spawned_cubes)

    if count == 0:
        msg = "삭제할 큐브가 없습니다."
        print(f"⚠️  {msg}")
        return msg

    # USD 스테이지에서 큐브 Prim 삭제
    current_stage = stage_utils.get_current_stage()
    for prim_path in spawned_cubes:
        current_stage.RemovePrim(Sdf.Path(prim_path))
        print(f"🗑️  삭제: {prim_path}")

    # 리스트 초기화
    spawned_cubes.clear()

    msg = f"리셋 완료: {count}개 큐브 삭제"
    print(f"✅ {msg}")
    return msg

# ================================================================
# STEP 9. ROS2 서비스 서버 생성
# ================================================================
rclpy.init()
ros_node = rclpy.create_node("isaac_service_node")

def handle_cube_control(request, response):
    """
    /cube_control 서비스가 호출되면 이 함수가 실행된다.

    request.data = True  → 큐브 생성 요청
    request.data = False → 전체 리셋 요청

    response.success : 성공 여부 (True/False)
    response.message : 응답 메시지 (터미널 2에서 확인 가능)
    """
    print(f"\n📞 서비스 호출 수신: data={request.data}")

    if request.data:
        # true: 큐브 생성
        msg = spawn_cube()
        response.success = True
        response.message = msg
    else:
        # false: 전체 리셋
        msg = reset_all_cubes()
        response.success = True
        response.message = msg

    return response

ros_node.create_service(SetBool, "/cube_control", handle_cube_control)
print("✅ ROS2 서비스 서버 생성 완료: /cube_control (std_srvs/SetBool)")

# ================================================================
# STEP 10. 시뮬레이션 시작
# ================================================================
simulation_context.initialize_physics()
simulation_context.play()

print("\n" + "=" * 60)
print("  ▶  시뮬레이션 시작")
print("  📡 /cube_control 서비스 대기 중...")
print()
print("  [큐브 생성]")
print("  ros2 service call /cube_control std_srvs/srv/SetBool \"data: true\"")
print()
print("  [전체 리셋]")
print("  ros2 service call /cube_control std_srvs/srv/SetBool \"data: false\"")
print("=" * 60 + "\n")

# ================================================================
# STEP 11. 메인 루프
# ================================================================
while simulation_app.is_running():
    # 물리 + 렌더링 한 프레임 진행
    simulation_context.step(render=True)

    # ROS2 서비스 요청 처리 (non-blocking)
    rclpy.spin_once(ros_node, timeout_sec=0)

# ================================================================
# STEP 12. 종료
# ================================================================
print("\n⏹  시뮬레이션 종료")
ros_node.destroy_node()
rclpy.shutdown()
simulation_context.stop()
simulation_app.close()