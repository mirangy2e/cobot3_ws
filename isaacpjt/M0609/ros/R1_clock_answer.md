---
notion_parent: https://app.notion.com/p/27b450ef7c5983939fa701761ca13d7d
notion_page: https://app.notion.com/p/409450ef7c5981828318e16fe357e281
---

# R1. Clock Graph 를 코드로: /clock 발행_정답

## 💡학습 내용
1. **정답 코드**

   ### R1_clock_answer.py

   ```python
   """[R1 연습 정답] 시뮬레이션 시간(/clock)과 실제 시간(/system_clock) 함께 발행"""

   from pathlib import Path

   # ============================================================
   # 1. 앱 시작 (Isaac 모듈보다 먼저)
   # ============================================================
   from isaacsim import SimulationApp

   simulation_app = SimulationApp({"headless": False})

   # ============================================================
   # 2. Isaac 모듈 import (앱을 켠 뒤에)
   # ============================================================
   import omni.graph.core as og
   from isaacsim.core.api import World
   from isaacsim.core.utils.extensions import enable_extension
   from isaacsim.core.utils.stage import open_stage

   SCENE_USD = Path(__file__).resolve().parents[1] / "scenes" / "pick_place_cube.usd"  # GUI 강의에서 만든 기본 씬
   GRAPH_PATH = "/World/Graph/ROS_Clock"  # GUI 강의와 같은 위치

   # (가) ROS2 Bridge 확장 켜기: ROS2 노드(ROS2Context, ROS2PublishClock 등)를 쓰기 전에 켠다
   enable_extension("isaacsim.ros2.bridge")
   simulation_app.update()

   # (나) 작업 씬 열기: 로봇, 바닥, 큐브가 들어 있는 기본 씬
   open_stage(str(SCENE_USD))
   world = World(stage_units_in_meters=1.0)

   # (다) Clock Graph 만들기: GUI 의 Tools > Robotics > ROS 2 OmniGraphs > Clock 과 같은 노드 4개
   og.Controller.edit(
       {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},  # execution = Action Graph
       {
           og.Controller.Keys.CREATE_NODES: [
               ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),              # Play 중 매 프레임 실행 신호
               ("Context", "isaacsim.ros2.bridge.ROS2Context"),                     # ROS2 통신 설정 (Domain ID)
               ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),      # 시뮬레이션 시간 읽기
               ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),           # 시간을 토픽으로 발행
               ("ReadSysTime", "isaacsim.core.nodes.IsaacReadSystemTime"),          # 실제(시스템) 시간 읽기
               ("PublishSysClock", "isaacsim.ros2.bridge.ROS2PublishClock"),        # 실제 시간을 토픽으로 발행
           ],
           og.Controller.Keys.CONNECT: [
               ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),       # 실행 신호
               ("Context.outputs:context", "PublishClock.inputs:context"),          # 통신 설정
               ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),  # 시간
               ("OnPlaybackTick.outputs:tick", "PublishSysClock.inputs:execIn"),
               ("Context.outputs:context", "PublishSysClock.inputs:context"),
               ("ReadSysTime.outputs:systemTime", "PublishSysClock.inputs:timeStamp"),  # 실제 시간
           ],
           og.Controller.Keys.SET_VALUES: [
               ("PublishClock.inputs:topicName", "clock"),                          # 토픽 이름: /clock
               ("PublishSysClock.inputs:topicName", "system_clock"),                # 토픽 이름: /system_clock
           ],
       },
   )
   print("Clock Graph 생성:", GRAPH_PATH)

   # (라) 멈춘 상태로 기다린다
   world.reset()
   world.pause()  # 화면 왼쪽의 Play(▶) 를 누르면 /clock 발행 시작
   print("Play 를 누르면 /clock 과 /system_clock 을 발행한다")

   # ============================================================
   # 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
   # ============================================================
   frame = 0  # Play 중에 지난 프레임 수
   while simulation_app.is_running():
       world.step(render=True)
       if not world.is_playing():
           continue
       frame += 1
       if frame % 60 == 0:  # 60 프레임마다 시뮬레이션 시간 출력 (ros2 topic echo /clock 과 비교)
           print(f"{frame} 프레임 | 시뮬레이션 시간 {world.current_time:6.2f} s")

   # ============================================================
   # 4. 종료
   # ============================================================
   simulation_app.close()
   ```

   ```bash
   cd ~/cobot3_ws/isaacpjt/M0609/ros
   isaac_python R1_clock_answer.py
   ```

2. **실제 시간을 발행하는 노드 추가**

   ```python
   ("ReadSysTime", "isaacsim.core.nodes.IsaacReadSystemTime"),          # 실제(시스템) 시간 읽기
   ("PublishSysClock", "isaacsim.ros2.bridge.ROS2PublishClock"),        # 실제 시간을 토픽으로 발행
   ```

   ```python
   ("OnPlaybackTick.outputs:tick", "PublishSysClock.inputs:execIn"),
   ("Context.outputs:context", "PublishSysClock.inputs:context"),
   ("ReadSysTime.outputs:systemTime", "PublishSysClock.inputs:timeStamp"),  # 실제 시간
   ```

   - 시간을 읽는 노드만 `IsaacReadSimulationTime` 에서 `IsaacReadSystemTime` 으로 바꾸고, 발행 노드는 같은 `ROS2PublishClock` 을 하나 더 쓴다
   - 실행 신호(`OnPlaybackTick`)와 통신 설정(`Context`)은 두 발행 노드가 함께 쓴다
   - 토픽 이름은 `SET_VALUES` 에서 `system_clock` 으로 정한다

3. **두 시계 비교**

   ```bash
   ros2 topic echo /clock --once
   ros2 topic echo /system_clock --once
   ```

   - **실행 결과**

   ```
   /clock         sec: 15  nanosec: 700000818
   /system_clock  sec: 1793344629  nanosec: 468526592
   ```

   - `/clock` 은 Play 를 시작한 뒤 흐른 **시뮬레이션 시간**이다. 0 부터 시작한다
   - `/system_clock` 은 PC 의 **실제 시각**이다. 1970년 1월 1일부터 센 초(Unix time)라서 값이 크다
   - Pause 하면 `OnPlaybackTick` 이 멈춰 두 토픽 모두 발행되지 않는다. 다시 Play 하면 `/clock` 은 멈췄던 값에서 이어지고, `/system_clock` 은 Pause 한 시간만큼 건너뛴 값으로 나온다. 그래서 Pause 할 때마다 두 시계의 차이가 커진다

   > 💡 ROS2 노드가 `/clock` 의 시간을 기준으로 동작하게 하는 설정이 `use_sim_time` 이다.

---

## 📝 요약 정리

- [ ] GUI 메뉴가 만든 Clock Graph 와 코드로 만든 그래프가 같은 구성임을 설명할 수 있다
- [ ] `CREATE_NODES`, `CONNECT`, `SET_VALUES` 가 각각 무엇을 하는지 설명할 수 있다
- [ ] Pause 할 때마다 두 시계의 차이가 커지는 이유를 설명할 수 있다
