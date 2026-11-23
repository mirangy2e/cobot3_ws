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
