import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
import numpy as np
 
class DepthTo3DNode(Node):
    def __init__(self):
        super().__init__('depth_to_3d')
        self.bridge = CvBridge()
 
        # 카메라 내부 파라미터 저장 변수
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
 
        # Depth 이미지 저장 변수
        self.depth_image = None
 
        # CameraInfo 구독 (한 번만 받으면 됨)
        self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_cb,
            10
        )
 
        # Depth 이미지 구독
        self.create_subscription(
            Image,
            '/camera/depth/image_rect_raw',
            self.depth_cb,
            10
        )
 
        # YOLO 감지 결과 구독
        self.create_subscription(
            Detection2DArray,
            '/yolo/detections',
            self.detection_cb,
            10
        )
 
        # 3D 좌표 퍼블리시
        # frame_id: camera_color_optical_frame 기준
        self.point_pub = self.create_publisher(
            PointStamped,
            '/detected_object/xyz_camera',
            10
        )
 
        self.get_logger().info("Depth → 3D 변환 노드 시작")
 
    def camera_info_cb(self, msg):
        # K 행렬에서 내부 파라미터 추출
        # K = [fx,  0, cx,
        #       0, fy, cy,
        #       0,  0,  1]
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
 
    def depth_cb(self, msg):
        # ROS Image → numpy float32 배열 (단위: m)
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, '32FC1')
 
    def detection_cb(self, msg):
        # 파라미터 또는 이미지 미수신 시 스킵
        if self.fx is None:
            self.get_logger().warn("CameraInfo 미수신")
            return
        if self.depth_image is None:
            self.get_logger().warn("Depth 이미지 미수신")
            return
        if not msg.detections:
            return
 
        # 가장 신뢰도 높은 감지 결과 선택
        best = max(
            msg.detections,
            key=lambda d: d.results[0].hypothesis.score if d.results else 0
        )
 
        # 바운딩박스 중심 픽셀 좌표
        u = int(best.bbox.center.position.x)
        v = int(best.bbox.center.position.y)
 
        # 이미지 경계 처리
        h, w = self.depth_image.shape
        u = max(0, min(u, w - 1))
        v = max(0, min(v, h - 1))
 
        # Depth 값 추출 (단위: m)
        Z = float(self.depth_image[v, u])
 
        # 유효하지 않은 Depth 값 처리
        if Z <= 0.0 or np.isnan(Z) or np.isinf(Z):
            self.get_logger().warn(
                f"유효하지 않은 Depth: ({u}, {v}) = {Z}"
            )
            return
 
        # 역투영: 픽셀 → 3D 카메라 좌표
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
 
        # PointStamped 메시지 생성
        # frame_id: camera_color_optical_frame 기준 좌표
        point = PointStamped()
        point.header.stamp = self.get_clock().now().to_msg()
        point.header.frame_id = 'camera_color_optical_frame'
        point.point.x = X
        point.point.y = Y
        point.point.z = Z
 
        self.point_pub.publish(point)
 
        self.get_logger().info(
            f"3D 좌표 (카메라 기준): "
            f"X={X:.3f}m, Y={Y:.3f}m, Z={Z:.3f}m"
        )
 
 
def main():
    rclpy.init()
    node = DepthTo3DNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()