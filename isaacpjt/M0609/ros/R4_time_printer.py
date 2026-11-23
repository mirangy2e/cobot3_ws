"""[R4 예제] use_sim_time: ROS2 노드가 보는 현재 시간을 1초마다 출력

실행 (ROS2 터미널, ros_set 후):
    python3 R4_time_printer.py                                   # 실제 시간 (기본)
    python3 R4_time_printer.py --ros-args -p use_sim_time:=true  # Isaac Sim 의 /clock 시간
"""

import rclpy
from rclpy.executors import ExternalShutdownException

rclpy.init()
node = rclpy.create_node("time_printer")  # ROS2 노드 이름
print("use_sim_time:", node.get_parameter("use_sim_time").value)  # 모든 노드에 있는 파라미터 (기본 False)


def on_timer():
    """노드의 시계로 현재 시간을 읽어 출력한다"""
    now = node.get_clock().now()  # use_sim_time 이 True 면 /clock 시간, False 면 실제 시간
    print(f"ROS 시간 {now.nanoseconds / 1e9:.2f} s")


node.create_timer(1.0, on_timer)  # 1초마다 on_timer 실행 (노드의 시계 기준)

try:
    rclpy.spin(node)  # Ctrl+C 를 누를 때까지 콜백 실행
except (KeyboardInterrupt, ExternalShutdownException):
    pass
node.destroy_node()
rclpy.try_shutdown()
