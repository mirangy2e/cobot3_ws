import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class Yolo11Node(Node):
    def __init__(self):
        super().__init__('yolo11_detector')

        self.model  = YOLO('yolo11n.pt')
        self.model.to('cpu')
        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image, '/rgb', self.image_callback, 10
        )
        self.det_pub = self.create_publisher(
            Detection2DArray, '/yolo/detections', 10
        )
        self.img_pub = self.create_publisher(
            Image, '/yolo/debug_image', 10
        )

        self.target_classes = ['cup', 'bottle', 'bowl']
        self.get_logger().info('YOLO11 노드 시작 — 전체 탐지 출력 모드')

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results  = self.model(cv_image, verbose=False)

        det_array = Detection2DArray()
        det_array.header = msg.header

        # ── 전체 탐지 결과 출력 (필터 없음) ──────────────────
        self.get_logger().info('─' * 40)
        for result in results:
            for box in result.boxes:
                cls_name = self.model.names[int(box.cls[0])]
                conf     = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0

                # 전체 탐지 로그 (클래스/신뢰도/좌표 모두 출력)
                self.get_logger().info(
                    f'[전체] {cls_name:<15} conf: {conf:.2f} | '
                    f'중심: ({cx:.0f}, {cy:.0f}) | '
                    f'bbox: ({x1},{y1})→({x2},{y2})'
                )

                # 시각화 — 전체 탐지 결과를 색상으로 구분
                if cls_name in self.target_classes:
                    color = (0, 255, 0)   # 초록 — target 클래스
                else:
                    color = (128, 128, 128)  # 회색 — 그 외 클래스

                
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), color, 4) 
                cv2.putText(cv_image, f'{cls_name} {conf:.2f}',
                            (x1, y1 - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.5, color, 3)  

                # target 클래스만 토픽 발행
                if cls_name in self.target_classes and conf >= 0.4:
                    det = Detection2D()
                    det.bbox.center.position.x = cx
                    det.bbox.center.position.y = cy
                    det.bbox.size_x = float(x2 - x1)
                    det.bbox.size_y = float(y2 - y1)
                    det_array.detections.append(det)

        self.det_pub.publish(det_array)

        debug_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
        debug_msg.header = msg.header
        self.img_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = Yolo11Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()