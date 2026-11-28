"""[카메라 인식 토픽] 3D 바운딩 박스를 MarkerArray 로 바꿔 발행하는 노드 (ROS2)

Isaac Sim 의 Camera Graph 가 발행하는 /bbox_3d 는 vision_msgs/Detection3DArray 라서
RViz2 는 플러그인을 설치해야 하고, Foxglove 같은 웹 3D 화면은 바로 그리지 못한다.
이 노드가 박스마다 반투명 상자와 라벨 글자를 만들어 visualization_msgs/MarkerArray 로 발행한다.
    상자 색: 라벨 이름이 red / green / blue / yellow 이면 그 색, 아니면 회색
    글자   : 라벨 이름 (상자 위에 상자와 같은 색으로 표시)
좌표계는 받은 메시지의 frame_id 를 그대로 쓴다.
(Camera Graph 의 Bbox3dPublish 노드 frameId 를 World 로 바꿔 두어야 월드 좌표 값과 맞는다)

실행:
    ros_set
    python3 ~/cobot3_ws/isaacpjt/M0609/camera_perception/m0609_bbox_3d_marker.py
확인:
    RViz2 또는 Foxglove 3D 패널 > 고정 좌표 프레임 World > 토픽 /bbox_3d_markers
"""

import json

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray

BBOX_TOPIC = "bbox_3d"               # 받을 3D 박스 토픽
LABELS_TOPIC = "semantic_labels"     # 받을 라벨 대응표 토픽 (class_id 번호 -> 라벨 이름)
OUT_TOPIC = "bbox_3d_markers"        # 보낼 마커 토픽

ALPHA = 0.5                          # 상자 투명도 (0: 투명, 1: 불투명)
TEXT_SIZE = 0.06                     # 라벨 글자 높이 (m)
TEXT_GAP = 0.03                      # 상자 윗면과 글자 사이 간격 (m)

# 라벨 이름별 상자 색 (RGB, 0~1). 없는 이름은 DEFAULT_COLOR
NAMED_COLORS = {
    "red": (0.90, 0.10, 0.29),
    "green": (0.24, 0.71, 0.29),
    "blue": (0.00, 0.51, 0.78),
    "yellow": (1.00, 0.88, 0.10),
}
DEFAULT_COLOR = (0.6, 0.6, 0.6)


class BBox3DMarker(Node):
    """/bbox_3d 가 올 때마다 박스마다 상자·글자 마커를 만들어 발행한다"""

    def __init__(self):
        super().__init__("m0609_bbox_3d_marker")
        self.class_names = {}    # bbox 의 class_id -> 라벨 이름 (예: "0" -> "red")

        self.create_subscription(Detection3DArray, BBOX_TOPIC, self.on_bbox, 10)
        self.create_subscription(String, LABELS_TOPIC, self.on_labels, 10)
        self.publisher = self.create_publisher(MarkerArray, OUT_TOPIC, 10)
        self.get_logger().info(f"/{BBOX_TOPIC} 구독, /{OUT_TOPIC} 발행")

    def on_labels(self, msg):
        """/semantic_labels 에는 세그멘테이션용과 bbox 용 대응표가 번갈아 온다.
        bbox 용은 BACKGROUND 항목이 없고 값이 {"class": 이름} 이다"""
        table = json.loads(msg.data)
        table.pop("time_stamp", None)
        values = list(table.values())
        if values and all(isinstance(v, dict) for v in values) and \
                not any(v.get("class") == "BACKGROUND" for v in values):
            self.class_names = {k: v["class"] for k, v in table.items()}

    def on_bbox(self, msg):
        markers = MarkerArray()

        # 1) 지난번 마커를 모두 지운다 (물체가 사라지면 상자도 사라지게)
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, det in enumerate(msg.detections):
            class_id = det.results[0].hypothesis.class_id if det.results else "?"
            name = self.class_names.get(class_id, class_id)
            r, g, b = NAMED_COLORS.get(name, DEFAULT_COLOR)

            # 2) 반투명 상자: 위치·자세·크기를 박스 값 그대로 쓴다
            box = Marker()
            box.header = msg.header
            box.ns, box.id = "box", i
            box.type, box.action = Marker.CUBE, Marker.ADD
            box.pose = det.bbox.center
            box.scale.x, box.scale.y, box.scale.z = det.bbox.size.x, det.bbox.size.y, det.bbox.size.z
            box.color.r, box.color.g, box.color.b, box.color.a = r, g, b, ALPHA
            markers.markers.append(box)

            # 3) 라벨 글자: 상자 윗면보다 조금 위에 둔다
            text = Marker()
            text.header = msg.header
            text.ns, text.id = "label", i
            text.type, text.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            text.pose.position.x = det.bbox.center.position.x
            text.pose.position.y = det.bbox.center.position.y
            text.pose.position.z = det.bbox.center.position.z + det.bbox.size.z / 2 + TEXT_GAP
            text.pose.orientation.w = 1.0
            text.scale.z = TEXT_SIZE
            text.color.r, text.color.g, text.color.b, text.color.a = r, g, b, 1.0   # 밝은·어두운 배경 모두 보이게 상자 색
            text.text = name
            markers.markers.append(text)

        self.publisher.publish(markers)


def main():
    rclpy.init()
    node = BBox3DMarker()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
