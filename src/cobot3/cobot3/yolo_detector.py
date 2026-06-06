import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # YOLOv11 모델 로드
        self.model = YOLO('yolo11s.pt')
        self.bridge = CvBridge()

        # COCO 클래스에서 banana id = 46
        self.target_class = 'banana'

        # RGB 이미지 구독
        self.sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        # 감지 결과 퍼블리시
        self.det_pub = self.create_publisher(
            Detection2DArray,
            '/yolo/detections',
            10
        )

        # 시각화 이미지 퍼블리시 (디버깅용)
        self.vis_pub = self.create_publisher(
            Image,
            '/yolo/visualization',
            10
        )

        self.get_logger().info("YOLO11 바나나 감지 노드 시작")

    def image_callback(self, msg):
        # ROS Image → OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # YOLO11 추론
        # conf: 신뢰도 임계값 (0.5 이상만 감지)
        # classes: COCO banana class id = 46
        results = self.model(cv_image, conf=0.5, classes=[46], verbose=False)

        det_array = Detection2DArray()
        det_array.header = msg.header

        for r in results[0].boxes:
            x1, y1, x2, y2 = r.xyxy[0].tolist()
            conf = float(r.conf[0])
            cls_id = int(r.cls[0])

            # 바운딩박스 중심 픽셀 좌표
            u = (x1 + x2) / 2.0   # 중심 x (픽셀)
            v = (y1 + y2) / 2.0   # 중심 y (픽셀)

            det = Detection2D()
            det.bbox.center.position.x = u
            det.bbox.center.position.y = v
            det.bbox.size_x = x2 - x1
            det.bbox.size_y = y2 - y1

            # 클래스 및 신뢰도 정보
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(cls_id)
            hyp.hypothesis.score = conf
            det.results.append(hyp)

            det_array.detections.append(det)

            self.get_logger().info(
                f"바나나 감지: 중심=({u:.1f}, {v:.1f}), "
                f"신뢰도={conf:.2f}, "
                f"크기=({x2-x1:.1f}×{y2-y1:.1f})"
            )

            # 시각화: 바운딩박스 그리기
            cv2.rectangle(cv_image,
                (int(x1), int(y1)), (int(x2), int(y2)),
                (0, 255, 0), 2)
            cv2.circle(cv_image,
                (int(u), int(v)), 5,
                (0, 0, 255), -1)
            cv2.putText(cv_image,
                f"banana {conf:.2f}",      # cup → banana 수정
                (int(x1), int(y1)-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 시각화 이미지 퍼블리시
        vis_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        vis_msg.header = msg.header
        self.vis_pub.publish(vis_msg)

        # 감지 결과 퍼블리시
        self.det_pub.publish(det_array)


def main():
    rclpy.init()
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()