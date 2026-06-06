import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
 
 
class CoordTransformNode(Node):
    def __init__(self):
        super().__init__('coord_transform')
 
        self.bridge       = CvBridge()
        self.camera_info  = None
        self.latest_depth = None
 
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
            Detection2DArray, '/yolo/detections',
            self.detection_callback, 10
        )
 
        # /banana/point_camera 발행 → 다음 노드(franka_tf_transform)가 수신
        self.pub = self.create_publisher(
            PointStamped, '/banana/point_camera', 10
        )
 
        self.get_logger().info("좌표 변환 노드 시작")
        self.get_logger().info("구독: /camera_info, /camera/depth, /yolo/detections")
        self.get_logger().info("발행: /banana/point_camera")
 
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
        # 32FC1 인코딩: 미터 단위 float32
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='32FC1'
        )
 
    # ── YOLO 탐지 결과 수신 → 3D 좌표 계산 ──────────────
    def detection_callback(self, msg):
        if self.camera_info is None or self.latest_depth is None:
            self.get_logger().warn("camera_info 또는 depth 미수신 — 대기 중")
            return
 
        if len(msg.detections) == 0:
            return
 
        # 카메라 내부 파라미터
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
 
        for det in msg.detections:
            # bbox 중심 픽셀 좌표
            u = det.bbox.center.position.x
            v = det.bbox.center.position.y
 
            # depth 이미지에서 z값 읽기
            u_int = int(np.clip(u, 0, self.latest_depth.shape[1] - 1))
            v_int = int(np.clip(v, 0, self.latest_depth.shape[0] - 1))
            z = float(self.latest_depth[v_int, u_int])
 
            # 무효 depth 필터링
            if z <= 0.0 or np.isnan(z) or z > 3.0:
                self.get_logger().warn(f"유효하지 않은 depth 값: {z:.3f}m")
                continue
 
            # ── 핀홀 역투영: 카메라 프레임 기준 3D 좌표 ──
            x_cam = (u - cx) * z / fx
            y_cam = (v - cy) * z / fy
            z_cam = z
 
            self.get_logger().info(
                f"[카메라 기준] "
                f"X={x_cam:.3f}m, "
                f"Y={y_cam:.3f}m, "
                f"Z={z_cam:.3f}m"
            )
 
            # ── /banana/point_camera 토픽 발행 ───────────────────
            point = PointStamped()
            point.header.stamp    = self.get_clock().now().to_msg()
            point.header.frame_id = self.camera_info.header.frame_id
            point.point.x = x_cam
            point.point.y = y_cam
            point.point.z = z_cam
            self.pub.publish(point)
 
 
def main():
    rclpy.init()
    node = CoordTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()