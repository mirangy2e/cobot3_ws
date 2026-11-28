"""[카메라 인식 토픽] 세그멘테이션 영상을 색으로 칠해 발행하는 노드 (ROS2)

Isaac Sim 의 Camera Graph 가 발행하는 /semantic_segmentation, /instance_segmentation 은
픽셀마다 ID 번호가 들어 있는 32SC1 영상이라서 rqt Image View 와 RViz2 로 바로 볼 수 없다.
이 노드가 ID 마다 색을 정해 칠하고, 왼쪽 위에 ID 와 이름(범례)을 써서 컬러 영상으로 발행한다.
    /semantic_segmentation -> /semantic_segmentation_color  (같은 라벨이면 같은 색)
    /instance_segmentation -> /instance_segmentation_color  (물체마다 다른 색)
ID 와 이름의 대응은 /semantic_labels 에서 받는다.

실행:
    ros_set
    python3 ~/cobot3_ws/isaacpjt/M0609/camera_perception/m0609_seg_colorize.py
확인:
    rqt > Plugins > Visualization > Image View > 토픽 /semantic_segmentation_color, /instance_segmentation_color
"""

import json

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

SEMANTIC_TOPIC = "semantic_segmentation"     # 받을 시맨틱 세그멘테이션 토픽
INSTANCE_TOPIC = "instance_segmentation"     # 받을 인스턴스 세그멘테이션 토픽
LABELS_TOPIC = "semantic_labels"             # 받을 ID-이름 대응표 토픽
OUT_SUFFIX = "_color"                        # 보낼 토픽 이름 = 받은 토픽 이름 + 이 글자

# ID 별 색 (RGB). 0: BACKGROUND, 1: UNLABELLED, 2 부터 라벨이 붙은 물체
# 인스턴스는 블록 색(빨강·초록·파랑)과 헷갈리지 않게 다른 색을 쓴다
# 시맨틱은 라벨 이름이 NAMED_COLORS 에 있으면 그 색이 우선한다
PALETTES = {
    SEMANTIC_TOPIC: np.array([
        [0, 0, 0],          # 0 검정
        [90, 90, 90],       # 1 회색
        [230, 25, 75],      # 2 빨강
        [60, 180, 75],      # 3 초록
        [0, 130, 200],      # 4 파랑
        [255, 225, 25],     # 5 노랑
        [145, 30, 180],     # 6 보라
    ], dtype=np.uint8),
    INSTANCE_TOPIC: np.array([
        [0, 0, 0],          # 0 검정
        [90, 90, 90],       # 1 회색
        [255, 225, 25],     # 2 노랑
        [145, 30, 180],     # 3 보라
        [70, 240, 240],     # 4 하늘
        [245, 130, 48],     # 5 주황
        [240, 50, 230],     # 6 분홍
        [170, 110, 40],     # 7 갈색
    ], dtype=np.uint8),
}

# 시맨틱 라벨 이름이 아래 색 이름이면 번호와 상관없이 이 색으로 칠한다
# (라벨을 붙이는 순서에 따라 번호가 바뀌어도 파랑 블록은 파랑으로 보이게)
NAMED_COLORS = {
    "red": [230, 25, 75],
    "green": [60, 180, 75],
    "blue": [0, 130, 200],
    "yellow": [255, 225, 25],
}

BASE_WIDTH = 1280      # 범례 글자 크기의 기준 영상 너비 (px). 영상이 작으면 범례도 줄인다
MIN_SCALE = 0.5       # 글자가 읽을 수 없을 만큼 작아지지 않게 하는 최소 배율


def to_ids(msg):
    """sensor_msgs/Image (32SC1) -> ID 배열 (높이, 너비)"""
    return np.frombuffer(bytes(msg.data), dtype=np.int32).reshape(msg.height, msg.width)


class SegColorize(Node):
    """세그멘테이션 영상이 올 때마다 ID 를 색으로 바꾸고 범례를 써서 발행한다"""

    def __init__(self):
        super().__init__("m0609_seg_colorize")
        self.names = {SEMANTIC_TOPIC: {}, INSTANCE_TOPIC: {}}   # 토픽별 ID -> 이름
        self.publishers_ = {}
        for topic in (SEMANTIC_TOPIC, INSTANCE_TOPIC):
            self.create_subscription(Image, topic, lambda msg, t=topic: self.on_seg(msg, t), 10)
            self.publishers_[topic] = self.create_publisher(Image, topic + OUT_SUFFIX, 10)
        self.create_subscription(String, LABELS_TOPIC, self.on_labels, 10)
        self.get_logger().info(f"/{SEMANTIC_TOPIC}, /{INSTANCE_TOPIC} 구독, 뒤에 {OUT_SUFFIX} 를 붙인 토픽으로 발행")

    def on_labels(self, msg):
        """/semantic_labels 에는 대응표 3 종류가 번갈아 온다. 값의 모양으로 구분한다
            시맨틱 : {"0": {"class": "BACKGROUND"}, "2": {"class": "red"}, ...}
            인스턴스: {"0": "BACKGROUND", "2": "/World/red_block_1", ...}
            bbox    : {"0": {"class": "red"}, ...}  (BACKGROUND 없음, 여기서는 쓰지 않음)"""
        table = json.loads(msg.data)
        table.pop("time_stamp", None)
        values = list(table.values())
        if values and all(isinstance(v, str) for v in values):
            self.names[INSTANCE_TOPIC] = {int(k): v.split("/")[-1] for k, v in table.items()}
        elif any(isinstance(v, dict) and v.get("class") == "BACKGROUND" for v in values):
            self.names[SEMANTIC_TOPIC] = {int(k): v["class"] for k, v in table.items()}

    def color_table(self, topic, max_id):
        """ID -> 색 표 (max_id + 1 행). 팔레트를 돌려 쓰고, 시맨틱은 색 이름 라벨을 그 색으로 바꾼다"""
        palette = PALETTES[topic]
        table = palette[np.arange(max_id + 1) % len(palette)].copy()
        if topic == SEMANTIC_TOPIC:
            for i, name in self.names[topic].items():
                if i <= max_id and name in NAMED_COLORS:
                    table[i] = NAMED_COLORS[name]
        return table

    def on_seg(self, msg, topic):
        ids = to_ids(msg)

        # 1) ID 를 색으로 바꾼다
        table = self.color_table(topic, int(ids.max()))
        view = table[ids]

        # 2) 영상에 있는 ID 만 범례로 쓴다 (글자 크기는 영상 너비에 맞춤)
        k = max(msg.width / BASE_WIDTH, MIN_SCALE)
        size, gap, thick = max(int(20 * k), 6), max(int(28 * k), 9), max(int(2 * k), 1)
        y = gap
        for i in np.unique(ids):
            color = tuple(int(c) for c in table[i])
            name = self.names[topic].get(int(i), "?")
            cv2.rectangle(view, (gap // 3, y - size), (gap // 3 + size, y), color, -1)
            cv2.rectangle(view, (gap // 3, y - size), (gap // 3 + size, y), (255, 255, 255), 1)
            cv2.putText(view, f"{i}: {name}", (gap // 3 + size + gap // 3, y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6 * k, (255, 255, 255), thick)
            y += gap

        # 3) 받은 영상과 같은 header 로 발행
        out = Image()
        out.header = msg.header
        out.height, out.width = view.shape[:2]
        out.encoding = "rgb8"
        out.step = out.width * 3
        out.data = np.ascontiguousarray(view).tobytes()
        self.publishers_[topic].publish(out)


def main():
    rclpy.init()
    node = SegColorize()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
