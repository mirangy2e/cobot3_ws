"""[카메라 인식 토픽] 2D 바운딩 박스를 영상 위에 그려 발행하는 노드 (ROS2)

Isaac Sim 의 Camera Graph 가 발행하는 /bbox_2d_tight, /bbox_2d_loose 는 사각형 숫자(Detection2DArray)라서
rqt Image View 로 바로 볼 수 없다. 이 노드가 /rgb 영상 위에 두 박스를 겹쳐 그려 /bbox_2d_image 로 발행한다.
    초록 실선: Tight (보이는 부분만 감싼 박스)
    노랑 선  : Loose (가려진 부분까지 감싼 박스)
    글자     : 라벨 이름과 박스 크기 (가로 x 세로, px)

실행:
    ros_set
    python3 ~/cobot3_ws/isaacpjt/M0609/camera_perception/m0609_bbox_2d_overlay.py
확인:
    rqt > Plugins > Visualization > Image View > 토픽 /bbox_2d_image
"""

import json

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

RGB_TOPIC = "rgb"                    # 받을 영상 토픽
TIGHT_TOPIC = "bbox_2d_tight"        # 받을 Tight 박스 토픽
LOOSE_TOPIC = "bbox_2d_loose"        # 받을 Loose 박스 토픽
LABELS_TOPIC = "semantic_labels"     # 받을 라벨 대응표 토픽 (class_id 번호 -> 라벨 이름)
OUT_TOPIC = "bbox_2d_image"          # 보낼 영상 토픽 (박스를 그린 영상)

TIGHT_COLOR = (0, 200, 0)            # Tight 박스 색 (RGB, 초록)
LOOSE_COLOR = (255, 200, 0)          # Loose 박스 색 (RGB, 노랑)
LINE_WIDTH = 2                       # 박스 선 두께 (px, 1280 너비 기준)
BASE_WIDTH = 1280                    # 글자·선 크기의 기준 영상 너비 (px). 영상이 작으면 함께 줄인다
MIN_SCALE = 0.5       # 글자가 읽을 수 없을 만큼 작아지지 않게 하는 최소 배율


def to_rgb(msg):
    """sensor_msgs/Image (rgb8 / rgba8) -> RGB 배열 (높이, 너비, 3)"""
    channels = 4 if msg.encoding == "rgba8" else 3
    image = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(msg.height, msg.width, channels)
    return image[:, :, :3].copy()


def corners(det):
    """Detection2D 의 중심·크기 -> 왼쪽 위, 오른쪽 아래 꼭짓점 (px)"""
    cx, cy = det.bbox.center.position.x, det.bbox.center.position.y
    w, h = det.bbox.size_x, det.bbox.size_y
    return (int(cx - w / 2), int(cy - h / 2)), (int(cx + w / 2), int(cy + h / 2))


class BBox2DOverlay(Node):
    """최신 Tight·Loose 박스를 기억해 두었다가 영상이 올 때마다 겹쳐 그려 발행한다"""

    def __init__(self):
        super().__init__("m0609_bbox_2d_overlay")
        self.tight = []          # 가장 최근 Tight 박스 목록
        self.loose = []          # 가장 최근 Loose 박스 목록
        self.class_names = {}    # bbox 의 class_id -> 라벨 이름 (예: "0" -> "red")

        self.create_subscription(Image, RGB_TOPIC, self.on_image, 10)
        self.create_subscription(Detection2DArray, TIGHT_TOPIC, self.on_tight, 10)
        self.create_subscription(Detection2DArray, LOOSE_TOPIC, self.on_loose, 10)
        self.create_subscription(String, LABELS_TOPIC, self.on_labels, 10)
        self.publisher = self.create_publisher(Image, OUT_TOPIC, 10)
        self.get_logger().info(f"/{RGB_TOPIC}, /{TIGHT_TOPIC}, /{LOOSE_TOPIC} 구독, /{OUT_TOPIC} 발행")

    def on_tight(self, msg):
        self.tight = msg.detections

    def on_loose(self, msg):
        self.loose = msg.detections

    def on_labels(self, msg):
        """/semantic_labels 에는 세그멘테이션용과 bbox 용 대응표가 번갈아 온다.
        bbox 용은 BACKGROUND 항목이 없고 값이 {"class": 이름} 이다"""
        table = json.loads(msg.data)
        table.pop("time_stamp", None)
        values = list(table.values())
        if values and all(isinstance(v, dict) for v in values) and \
                not any(v.get("class") == "BACKGROUND" for v in values):
            self.class_names = {k: v["class"] for k, v in table.items()}

    def on_image(self, msg):
        view = to_rgb(msg)
        k = max(msg.width / BASE_WIDTH, MIN_SCALE)        # 영상 크기에 맞춘 배율
        line, font = max(int(LINE_WIDTH * k), 1), 0.6 * k

        # 1) Loose 를 먼저 그리고 그 위에 Tight 를 그린다 (두 박스가 같으면 Tight 가 보임)
        for det in self.loose:
            p1, p2 = corners(det)
            cv2.rectangle(view, p1, p2, LOOSE_COLOR, line)
        for det in self.tight:
            p1, p2 = corners(det)
            cv2.rectangle(view, p1, p2, TIGHT_COLOR, line)

            # 2) 라벨 이름과 Tight 박스 크기를 박스 위에 쓴다
            class_id = det.results[0].hypothesis.class_id if det.results else "?"
            name = self.class_names.get(class_id, class_id)
            text = f"{name} {int(det.bbox.size_x)}x{int(det.bbox.size_y)}"
            cv2.putText(view, text, (p1[0], p1[1] - 2 * line), cv2.FONT_HERSHEY_SIMPLEX, font, TIGHT_COLOR, line)

        # 3) 범례
        cv2.putText(view, "Tight", (int(10 * k), int(30 * k)), cv2.FONT_HERSHEY_SIMPLEX, 0.8 * k, TIGHT_COLOR, line)
        cv2.putText(view, "Loose", (int(100 * k), int(30 * k)), cv2.FONT_HERSHEY_SIMPLEX, 0.8 * k, LOOSE_COLOR, line)

        # 4) 받은 영상과 같은 header(시간, frame_id)로 발행
        out = Image()
        out.header = msg.header
        out.height, out.width = view.shape[:2]
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = view.tobytes()
        self.publisher.publish(out)


def main():
    rclpy.init()
    node = BBox2DOverlay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
