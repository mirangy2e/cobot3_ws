#!/usr/bin/env python3
"""
net_monitor.py
이미지 토픽의 수신 주파수(hz)와 대역폭(bw)을 계산해 Float32 토픽으로 발행한다.
반드시 이미지를 발행하는 노트북(GPU-A)에서 실행할 것.
  - 같은 머신이면 DDS가 shared memory로 전달하므로 대용량 이미지도 놓치지 않고 받는다.
  - 다른 노트북(rclpy)에서 재면 대용량 이미지를 다 못 받아 값이 낮게 왜곡될 수 있다.

발행 토픽:
  /monitor/image_hz        (Float32, Hz)
  /monitor/image_bw_mbps   (Float32, Mbps = megabits/sec)

실행:
  python3 net_monitor.py
  python3 net_monitor.py --ros-args -p image_topic:=/front_stereo_camera/left/image_raw
"""
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class NetMonitor(Node):
    def __init__(self):
        super().__init__('net_monitor')
        self.declare_parameter('image_topic', '/front_stereo_camera/left/image_raw')
        topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # 이미지 스트림은 BEST_EFFORT가 일반적. 발행 측 QoS와 맞춰야 구독됨.
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.sub = self.create_subscription(Image, topic, self.on_image, qos)
        self.pub_hz = self.create_publisher(Float32, '/monitor/image_hz', 10)
        self.pub_bw = self.create_publisher(Float32, '/monitor/image_bw_mbps', 10)

        self.count = 0
        self.bytes = 0
        self.last = time.monotonic()
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(f"net_monitor watching '{topic}'")

    def on_image(self, msg):
        self.count += 1
        self.bytes += len(msg.data)   # 픽셀 데이터 바이트 수

    def tick(self):
        now = time.monotonic()
        dt = now - self.last
        if dt <= 0:
            return
        hz = self.count / dt
        bw_mbps = (self.bytes * 8.0) / dt / 1e6   # bytes -> bits -> Mbps
        self.pub_hz.publish(Float32(data=float(hz)))
        self.pub_bw.publish(Float32(data=float(bw_mbps)))
        self.get_logger().info(f"hz {hz:5.1f}   bw {bw_mbps:7.1f} Mbps")
        self.count = 0
        self.bytes = 0
        self.last = now


def main():
    rclpy.init()
    node = NetMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()