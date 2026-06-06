import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
import tf2_ros
import tf2_geometry_msgs
import numpy as np

class CoordTransformNode(Node):
    def __init__(self):
        super().__init__('coord_transform')
        self.bridge       = CvBridge()
        self.camera_model = PinholeCameraModel()
        self.camera_ready = False
        self.latest_depth = None

        # TF 버퍼
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # 구독
        self.create_subscription(
            CameraInfo, '/camera_info', self.camera_info_callback, 10
        )
        self.create_subscription(
            Image, '/depth', self.depth_callback, 10
        )
        self.create_subscription(
            Detection2DArray, '/yolo/detections', self.detection_callback, 10
        )

        # 발행: 3D 목표 좌표
        self.pub = self.create_publisher(PoseStamped, '/target_pose', 10)

        self.get_logger().info('좌표 변환 노드 시작')

    # ── camera_info 수신 (최초 1회) ──────────────────────────
    def camera_info_callback(self, msg):
        if not self.camera_ready:
            self.camera_model.fromCameraInfo(msg)
            self.camera_ready = True
            self.get_logger().info(
                f'camera_info 수신 완료 | '
                f'fx={self.camera_model.fx():.1f} '
                f'fy={self.camera_model.fy():.1f} '
                f'cx={self.camera_model.cx():.1f} '
                f'cy={self.camera_model.cy():.1f}'
            )

    # ── depth 이미지 저장 ────────────────────────────────────
    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding='passthrough'
        )

    # ── YOLO 탐지 결과 수신 → 3D 좌표 변환 ──────────────────
    def detection_callback(self, msg):
        if not self.camera_ready:
            self.get_logger().warn('camera_info 미수신 — 대기 중')
            return

        if not msg.detections:
            return

        if self.latest_depth is None:
            self.get_logger().warn('depth 미수신 — 대기 중')
            return

        # 첫 번째 탐지 객체만 사용
        det = msg.detections[0]
        u = int(det.bbox.center.position.x)
        v = int(det.bbox.center.position.y)

        # ── Step 1: depth 값 읽기 ───────────────────────────
        z = float(self.latest_depth[v, u])

        if np.isnan(z) or z <= 0.0:
            self.get_logger().warn(f'유효하지 않은 depth: {z}')
            return

        # ── Step 2: 핀홀 역투영 → camera_frame 3D 좌표 ──────
        X = (u - self.camera_model.cx()) * z / self.camera_model.fx()
        Y = (v - self.camera_model.cy()) * z / self.camera_model.fy()
        Z = z

        self.get_logger().info(
            f'[camera_frame] X={X:.3f} Y={Y:.3f} Z={Z:.3f} m'
        )

        # ── Step 3: TF 변환 → world 좌표 ───────────────────
        pose_camera = PoseStamped()
        pose_camera.header.stamp    = rclpy.time.Time().to_msg()
        pose_camera.header.frame_id = 'Camera'          
        pose_camera.pose.position.x = X
        pose_camera.pose.position.y = Y
        pose_camera.pose.position.z = Z
        pose_camera.pose.orientation.w = 1.0

        try:
            pose_world = self.tf_buffer.transform(
                pose_camera,
                'world',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            self.get_logger().info(
                f'[world] '
                f'X={pose_world.pose.position.x:.3f} '
                f'Y={pose_world.pose.position.y:.3f} '
                f'Z={pose_world.pose.position.z:.3f} m'
            )

            self.pub.publish(pose_world)

        except Exception as e:
            self.get_logger().warn(f'TF 변환 실패: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = CoordTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()