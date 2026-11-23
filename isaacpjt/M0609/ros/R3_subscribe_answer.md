---
notion_parent: https://app.notion.com/p/27b450ef7c5983939fa701761ca13d7d
notion_page: https://app.notion.com/p/409450ef7c5981cb8615c43be1d72b08
---

# R3. 토픽 구독: 명령을 받아 로봇팔 움직이기_정답

## 💡학습 내용
1. **정답 코드**

   ### R3_subscribe_answer.py

   ```python
   """[R3 연습 정답] /arm_command (문자열) 와 /arm_angle (각도 숫자) 두 토픽으로 로봇팔 움직이기"""

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
   from std_msgs.msg import Int32, String

   SCENE_USD = Path(__file__).resolve().parents[1] / "scenes" / "pick_place_cube.usd"  # GUI 강의에서 만든 기본 씬
   HOME_DEG = [0, 0, 90, 0, 90, 0]                       # 홈 자세 (deg)
   COMMANDS = {"left": 45.0, "right": -45.0, "home": 0.0}  # 명령 -> joint_1 목표 각도 (deg)
   ANGLE_LIMIT_DEG = 90.0                                # /arm_angle 로 받을 수 있는 각도 범위 (+-deg)

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


   def on_angle(msg):
       """각도(deg)를 받으면 범위 안일 때만 목표 각도로 저장한다"""
       global target_deg
       if abs(msg.data) <= ANGLE_LIMIT_DEG:
           target_deg = float(msg.data)
           print(f"각도 받음: joint_1 {target_deg:.0f} deg")
       else:
           print(f"범위 밖 각도: {msg.data} deg (-{ANGLE_LIMIT_DEG:.0f} ~ {ANGLE_LIMIT_DEG:.0f})")


   rclpy.init()
   node = rclpy.create_node("arm_commander")                             # ROS2 노드 이름
   node.create_subscription(String, "arm_command", on_command, 10)       # (메시지 타입, 토픽 이름, 콜백, 큐 크기)
   node.create_subscription(Int32, "arm_angle", on_angle, 10)            # 각도(deg)를 숫자로 받는 토픽

   # (라) 홈 자세로 순간이동하고, 멈춘 상태로 기다린다
   world.reset()
   arm = [robot.get_dof_index(f"joint_{i}") for i in range(1, 7)]  # 팔 관절 6개가 관절 배열의 몇 번째인지
   home = robot.get_joint_positions()
   home[arm] = np.radians(HOME_DEG)
   robot.set_joint_positions(home)
   world.step(render=True)  # 한 프레임 진행해서 순간이동한 자세를 화면에 반영
   world.pause()            # 화면 왼쪽의 Play(▶) 를 누르면 시작
   print("Play 를 누르고 /arm_command 로 left, right, home 을, /arm_angle 로 각도를 보낸다")

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
   ```

   ```bash
   cd ~/cobot3_ws/isaacpjt/M0609/ros
   isaac_python R3_subscribe_answer.py
   ```
2. **각도를 받는 콜백 추가**

   ```python
   def on_angle(msg):
       """각도(deg)를 받으면 범위 안일 때만 목표 각도로 저장한다"""
       global target_deg
       if abs(msg.data) <= ANGLE_LIMIT_DEG:
           target_deg = float(msg.data)
           print(f"각도 받음: joint_1 {target_deg:.0f} deg")
       else:
           print(f"범위 밖 각도: {msg.data} deg (-{ANGLE_LIMIT_DEG:.0f} ~ {ANGLE_LIMIT_DEG:.0f})")
   ```

   - `Int32` 메시지의 숫자는 `msg.data` 에 들어 있다
   - 범위 안일 때만 `target_deg` 에 저장한다. 범위 밖이면 저장하지 않으므로 로봇은 움직이지 않는다
3. **구독 하나 더 만들기**

   ```python
   node.create_subscription(Int32, "arm_angle", on_angle, 10)            # 각도(deg)를 숫자로 받는 토픽
   ```

   - 한 노드에 구독을 여러 개 만들 수 있다. `spin_once` 한 번에 도착한 메시지의 콜백이 실행된다
   - 두 콜백이 같은 `target_deg` 에 저장하므로, 메인 루프는 바꿀 필요가 없다
4. **실행 결과**

   ```bash
   ros2 topic pub --once /arm_angle std_msgs/msg/Int32 "{data: 30}"
   ros2 topic pub --once /arm_angle std_msgs/msg/Int32 "{data: -60}"
   ros2 topic pub --once /arm_angle std_msgs/msg/Int32 "{data: 120}"
   ros2 topic pub --once /arm_command std_msgs/msg/String "{data: left}"
   ```

   ```
   각도 받음: joint_1 30 deg
   각도 받음: joint_1 -60 deg
   범위 밖 각도: 120 deg (-90 ~ 90)
   명령 받음: left -> joint_1 45 deg
   ```

---

## 📝 요약 정리
- [ ] 콜백에서 로봇을 바로 움직이지 않고 목표만 저장하는 구조를 설명할 수 있다
- [ ] 메시지 타입이 달라지면 무엇을 바꿔야 하는지 설명할 수 있다
