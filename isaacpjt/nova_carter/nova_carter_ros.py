from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import time
from isaacsim.core.utils.stage import open_stage, is_stage_loading

USD_PATH = str(Path(__file__).resolve().parent / "scenes/carter_warehouse_navigation.usd")

# USD 파일 열기 (File → Open과 같은 동작)
open_stage(USD_PATH)

# 씬 로딩이 끝날 때까지 대기
while is_stage_loading():
    simulation_app.update()

print("\n[완료] 씬 로드 — Play 버튼을 눌러 ROS2 토픽 발행을 시작하세요")

while simulation_app.is_running():
    simulation_app.update()
    time.sleep(0.016)

simulation_app.close()
