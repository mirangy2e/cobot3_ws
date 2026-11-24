"""[색상 분류 미션] 색상 감지 노드 (PC B, ROS2)

/rgb (손목 카메라 영상) 를 받아 파랑/초록 큐브를 찾고, /color_id 로 색 번호를 발행한다
    0: 못 찾음, 1: 파랑, 2: 초록

실행 (colcon build 후):
    ros_set
    source ~/cobot3_ws/install/setup.bash
    ros2 run m0609 m0609_color_detector
"""

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

RGB_TOPIC = "rgb"          # 받을 영상 토픽 (Isaac Sim 의 Camera Graph 가 발행)
COLOR_TOPIC = "color_id"   # 보낼 색 번호 토픽
SHOW_WINDOW = True         # True 면 감지 결과를 창으로 보여 준다 (화면이 없는 PC 에서는 False)

# HSV 범위 (OpenCV: H 0 ~ 179, S 0 ~ 255, V 0 ~ 255). 바닥과 로봇은 채도(S)가 낮아 걸리지 않는다
COLOR_RANGES = {
    1: (np.array([100, 120, 50]), np.array([130, 255, 255])),  # 파랑
    2: (np.array([40, 120, 50]), np.array([80, 255, 255])),    # 초록
}
COLOR_NAMES = {0: "못 찾음", 1: "파랑", 2: "초록"}
MIN_AREA = 300             # 이 픽셀 수보다 작은 덩어리는 잡음으로 본다


def to_bgr(msg):
    """sensor_msgs/Image (rgb8 / bgr8 / rgba8) -> OpenCV BGR 배열"""
    channels = 4 if msg.encoding in ("rgba8", "bgra8") else 3
    image = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, channels)
    if msg.encoding.startswith("rgb"):
        return cv2.cvtColor(image[:, :, :3], cv2.COLOR_RGB2BGR)
    return image[:, :, :3].copy()


class ColorDetector(Node):
    """영상마다 파랑/초록 면적을 세어 더 큰 쪽의 색 번호를 발행한다"""

    def __init__(self):
        super().__init__("m0609_color_detector")
        self.create_subscription(Image, RGB_TOPIC, self.on_image, 10)
        self.publisher = self.create_publisher(Int32, COLOR_TOPIC, 10)
        self.color = None  # 바로 전 영상의 색 번호 (바뀔 때만 로그 출력)
        self.get_logger().info(f"/{RGB_TOPIC} 구독, /{COLOR_TOPIC} 발행. 영상을 기다린다")

    def on_image(self, msg):
        bgr = to_bgr(msg)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # 1) 색마다 마스크를 만들고 픽셀 수를 센다
        masks, areas = {}, {}
        for color, (lower, upper) in COLOR_RANGES.items():
            masks[color] = cv2.inRange(hsv, lower, upper)
            areas[color] = cv2.countNonZero(masks[color])

        # 2) 가장 넓은 색. MIN_AREA 보다 작으면 못 찾음(0)
        best = max(areas, key=areas.get)
        color = best if areas[best] >= MIN_AREA else 0

        # 3) 발행 (영상마다)
        self.publisher.publish(Int32(data=color))
        if color != self.color:
            self.get_logger().info(f"color_id {color} ({COLOR_NAMES[color]}) | 파랑 {areas[1]} px, 초록 {areas[2]} px")
            self.color = color

        if SHOW_WINDOW:
            self.show(bgr, masks, color)

    def show(self, bgr, masks, color):
        """찾은 색 덩어리의 윤곽선과 색 번호를 그려 창에 띄운다"""
        view = bgr.copy()
        for c, line in ((1, (255, 0, 0)), (2, (0, 200, 0))):
            contours, _ = cv2.findContours(masks[c], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(view, contours, -1, line, 2)
        cv2.putText(view, f"color_id = {color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 2)
        cv2.imshow("m0609_color_detector", view)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = ColorDetector()
    try:
        rclpy.spin(node)  # Ctrl+C 를 누를 때까지 콜백 실행
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
