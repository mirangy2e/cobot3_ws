import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import numpy as np
import cv2


class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # YOLO11 모델 로드
        self.model = YOLO('yolo11n.pt')
        self.bridge      = CvBridge()
        self.camera_info = None
        self.latest_depth = None
        self.frame_count  = 0

        # COCO banana class id = 46
        self.target_class = 'banana'

        # 구독
        self.create_subscription(
            CameraInfo, '/camera_info',
            self.camera_info_callback, 10
        )
        self.create_subscription(
            Image, '/camera/depth',
            self.depth_callback, 10
        )
        self.create_subscription(
            Image, '/camera/rgb',
            self.image_callback, 10
        )

        # /yolo/visualization 발행 (디버깅용)
        self.vis_pub = self.create_publisher(
            Image, '/yolo/visualization', 10
        )
        # /banana/point_camera 발행 → franka_coord_transform 수신
        self.point_pub = self.create_publisher(
            PointStamped, '/banana/point_camera', 10
        )

        self.get_logger().info("YOLO11 바나나 탐지 노드 시작")
        self.get_logger().info("구독: /camera/rgb, /camera/depth, /camera_info")
        self.get_logger().info("발행: /yolo/visualization, /banana/point_camera")

    # ── camera_info 수신 (최초 1회) ──────────────────────
    def camera_info_callback(self, msg):
        if self.camera_info is None:
            self.camera_info = msg
            fx = msg.k[0]
            fy = msg.k[4]
            cx = msg.k[2]
            cy = msg.k[5]
            self.get_logger().info(
                f"camera_info 수신 완료 | "
                f"fx={fx:.1f}, fy={fy:.1f}, "
                f"cx={cx:.1f}, cy={cy:.1f}"
            )

    # ── depth 이미지 수신 ─────────────────────────────────
    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='32FC1'
        )

    # ── RGB 이미지 수신 → YOLO 추론 → 3D 좌표 계산 ──────
    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % 20 != 0:  # 20프레임에 1번 처리 (3Hz)
            return

        # ROS Image → OpenCV BGR
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # YOLO11 추론
        results = self.model(cv_image, conf=0.1, classes=[46], verbose=False)

        for r in results[0].boxes:
            x1, y1, x2, y2 = r.xyxy[0].tolist()
            conf   = float(r.conf[0])
            cls_id = int(r.cls[0])

            # 바운딩박스 중심 픽셀 좌표
            u = (x1 + x2) / 2.0
            v = (y1 + y2) / 2.0

            self.get_logger().info(
                f"바나나 탐지: 중심=({u:.1f}, {v:.1f}), "
                f"신뢰도={conf:.2f}, "
                f"크기=({x2-x1:.1f}x{y2-y1:.1f})"
            )

            # 시각화
            cv2.rectangle(cv_image,
                          (int(x1), int(y1)), (int(x2), int(y2)),
                          (0, 255, 0), 3)
            cv2.circle(cv_image, (int(u), int(v)), 5, (0, 0, 255), -1)
            cv2.putText(cv_image, f"banana {conf:.2f}",
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 0), 3)

            # ── 카메라 기준 3D 좌표 계산 ─────────────────
            if self.camera_info is None or self.latest_depth is None:
                continue

            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            u_int = int(np.clip(u, 0, self.latest_depth.shape[1] - 1))
            v_int = int(np.clip(v, 0, self.latest_depth.shape[0] - 1))
            z = float(self.latest_depth[v_int, u_int])

            if z <= 0.0 or np.isnan(z) or z > 3.0:
                self.get_logger().warn(f"유효하지 않은 depth 값: {z:.3f}m")
                continue

            # 핀홀 역투영
            x_cam = (u - cx) * z / fx
            y_cam = (v - cy) * z / fy
            z_cam = z

            self.get_logger().info(
                f"[카메라 기준] "
                f"X={x_cam:.3f}m, Y={y_cam:.3f}m, Z={z_cam:.3f}m"
            )

            # /banana/point_camera 발행
            point = PointStamped()
            point.header.stamp    = self.get_clock().now().to_msg()
            point.header.frame_id = self.camera_info.header.frame_id
            point.point.x = x_cam
            point.point.y = y_cam
            point.point.z = z_cam
            self.point_pub.publish(point)

        # /yolo/visualization 발행
        vis_msg = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
        vis_msg.header = msg.header
        self.vis_pub.publish(vis_msg)



def main():
    rclpy.init()
    node = YoloDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()