---
notion_parent: https://app.notion.com/p/27b450ef7c5983939fa701761ca13d7d
notion_page: https://app.notion.com/p/409450ef7c5981b8bd65e8aecbcd7902
---

# R5. 연습 문제 정답

## 💡학습 내용
1. **정답 코드**

   ### R5_tf_answer.py

   ```python
   """[R5 연습 정답] TF 조회: base_link 기준 그리퍼(gripper_body) 위치를 1초마다 출력

   실행 (ROS2 터미널, ros_set 후. Isaac Sim 에서 R5_tf.py 를 Play 한 상태):
       python3 R5_tf_answer.py --ros-args -p use_sim_time:=true
   """

   import rclpy
   import tf2_ros
   from rclpy.executors import ExternalShutdownException

   PARENT_FRAME = "base_link"     # 기준 프레임
   CHILD_FRAME = "gripper_body"   # 위치를 알고 싶은 프레임

   rclpy.init()
   node = rclpy.create_node("gripper_tf_reader")  # ROS2 노드 이름
   tf_buffer = tf2_ros.Buffer()                                # 받은 TF 를 시간과 함께 모아 두는 곳
   tf_listener = tf2_ros.TransformListener(tf_buffer, node)    # /tf 를 구독해 tf_buffer 에 넣는다


   def on_timer():
       """가장 최근 TF 로 base_link 기준 gripper_body 위치를 출력한다"""
       try:
           tf = tf_buffer.lookup_transform(PARENT_FRAME, CHILD_FRAME, rclpy.time.Time())  # Time() = 가장 최근 값
       except tf2_ros.TransformException as e:
           print("TF 를 아직 받지 못함:", e)
           return
       p = tf.transform.translation
       stamp = tf.header.stamp.sec + tf.header.stamp.nanosec / 1e9
       print(f"{stamp:6.2f} s | {CHILD_FRAME} 위치 x {p.x:.3f}  y {p.y:.3f}  z {p.z:.3f} m")


   node.create_timer(1.0, on_timer)  # 1초마다 on_timer 실행 (노드의 시계 기준)

   try:
       rclpy.spin(node)  # Ctrl+C 를 누를 때까지 콜백 실행
   except (KeyboardInterrupt, ExternalShutdownException):
       pass
   node.destroy_node()
   rclpy.try_shutdown()
   ```

   ```bash
   cd ~/cobot3_ws/isaacpjt/M0609/ros
   python3 R5_tf_answer.py --ros-args -p use_sim_time:=true
   ```
2. **TF 받기**

   ```python
   tf_buffer = tf2_ros.Buffer()                                # 받은 TF 를 시간과 함께 모아 두는 곳
   tf_listener = tf2_ros.TransformListener(tf_buffer, node)    # /tf 를 구독해 tf_buffer 에 넣는다
   ```

   - `TransformListener` 가 `/tf` 를 구독해 받은 TF 를 `Buffer` 에 쌓는다. 직접 `/tf` 구독 콜백을 만들 필요가 없다
3. **두 프레임 사이 변환 조회**

   ```python
   tf = tf_buffer.lookup_transform(PARENT_FRAME, CHILD_FRAME, rclpy.time.Time())  # Time() = 가장 최근 값
   ```

   - `lookup_transform(기준 프레임, 알고 싶은 프레임, 시간)` 은 기준 프레임에서 본 프레임의 위치와 회전을 돌려준다
   - `base_link` 와 `gripper_body` 사이에 링크가 여러 개 있어도 Buffer 가 트리를 따라 계산한다
   - 아직 TF 를 받지 못했으면 예외(`TransformException`)가 난다. 처음 한 번은 `TF 를 아직 받지 못함` 이 출력될 수 있다
4. **실행 결과**

   ```
   TF 를 아직 받지 못함: "base_link" passed to lookupTransform argument target_frame does not exist.
     3.00 s | gripper_body 위치 x 0.340  y 0.203  z 0.375 m
     4.00 s | gripper_body 위치 x 0.346  y -0.193  z 0.375 m
     5.00 s | gripper_body 위치 x 0.346  y -0.193  z 0.375 m
     6.00 s | gripper_body 위치 x 0.346  y -0.193  z 0.375 m
     7.00 s | gripper_body 위치 x 0.340  y 0.203  z 0.375 m
   ```

   - `use_sim_time` 을 켰으므로 타이머가 시뮬레이션 시간 1초마다 실행된다. 출력 앞의 시간은 TF 의 시간(시뮬레이션 시간)이다
   - 3초마다 joint_1 이 바뀌어 y 가 약 +0.20 ↔ -0.19 m 로 바뀐다

---

## 📝 요약 정리
- [ ] `Buffer`, `TransformListener`, `lookup_transform` 이 각각 무엇을 하는지 설명할 수 있다
- [ ] 다른 두 프레임(예: `base_link` 와 카메라) 사이의 위치를 조회하도록 바꿀 수 있다
