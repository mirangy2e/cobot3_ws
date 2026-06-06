# -*- coding: utf-8 -*-
"""
[1단계] Isaac Sim 5.1.0 Standalone 기초
========================================
목표: ROS2 없이 Isaac Sim을 standalone 방식으로 실행하고
      그리드 바닥과 파란 큐브를 생성한다.

실행 방법:
    ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/step01_standalone_basic.py
"""

# ================================================================
# STEP 1. SimulationApp 초기화 (반드시 가장 먼저)
# ================================================================
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# ================================================================
# STEP 2. 모듈 임포트 (SimulationApp 이후에만 가능)
# ================================================================
import numpy as np
from pxr import Gf, UsdPhysics, UsdGeom

from isaacsim.core.api import SimulationContext
from isaacsim.core.api.physics_context import PhysicsContext
from isaacsim.core.utils import prims, stage as stage_utils
from isaacsim.storage.native import nucleus

# ================================================================
# STEP 3. 시뮬레이션 컨텍스트 설정
# ================================================================
simulation_context = SimulationContext(stage_units_in_meters=1.0)

# ================================================================
# STEP 4. Isaac 기본 그리드 환경 로드
# ================================================================
# Isaac Sim 서버에서 기본 그리드 환경 USD를 불러온다
# 이 USD 안에 조명 + 그리드 바닥 + 물리 환경이 모두 포함되어 있다
assets_root_path = nucleus.get_assets_root_path()

stage_utils.add_reference_to_stage(
    usd_path=assets_root_path + "/Isaac/Environments/Grid/default_environment.usd",
    prim_path="/World/Environment"
)

print("🌐 그리드 환경 로드 완료")

# ================================================================
# STEP 5. 물리 컨텍스트 초기화
# ================================================================
PhysicsContext()

# ================================================================
# STEP 6. 파란 큐브 생성
# ================================================================
cube_prim = prims.create_prim(
    prim_path="/World/BlueCube",
    prim_type="Cube",
    position=np.array([0.0, 0.0, 0.5]),   # z=0.5m 위에서 떨어뜨림
    scale=np.array([0.15, 0.15, 0.15]),    # 30cm 큐브 (보기 좋은 크기)
)

# 물리 속성: 중력 + 충돌 적용
UsdPhysics.RigidBodyAPI.Apply(cube_prim)
UsdPhysics.CollisionAPI.Apply(cube_prim)

# 파란색 설정
UsdGeom.Cube(cube_prim).GetDisplayColorAttr().Set(
    [Gf.Vec3f(0.0, 0.0, 1.0)]
)

print("🟦 파란 큐브 생성 완료 (30cm, z=0.5m)")
print("✅ 씬 구성 완료")

# ================================================================
# STEP 7. 시뮬레이션 시작
# ================================================================
simulation_context.initialize_physics()
simulation_context.play()

print("▶  시뮬레이션 시작 (Ctrl+C 로 종료)")
print("=" * 50)

# ================================================================
# STEP 8. 메인 루프
# ================================================================
while simulation_app.is_running():
    simulation_context.step(render=True)

# ================================================================
# STEP 9. 종료
# ================================================================
print("\n⏹  시뮬레이션 종료")
simulation_context.stop()
simulation_app.close()