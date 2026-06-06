import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
 
class TFTransformerNode(Node):
    def __init__(self):
        super().__init__('tf_transformer')
 
        # TF 버퍼 및 리스너 초기화
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self
        )
 
        # 카메라 기준 3D 좌표 구독
        self.create_subscription(
            PointStamped,
            '/detected_object/xyz_camera',
            self.point_cb,
            10
        )
 
        # 로봇 베이스 기준 3D 좌표 퍼블리시
        self.point_pub = self.create_publisher(
            PointStamped,
            '/detected_object/xyz_robot',
            10
        )
 
        self.get_logger().info("TF 변환 노드 시작")
 
    def point_cb(self, msg):
        try:
            # msg의 타임스탬프 대신 Time(0) 사용
            # Time(0) = 가장 최신 TF 데이터 사용
            msg.header.stamp = rclpy.time.Time().to_msg()

            point_robot = self.tf_buffer.transform(
                msg,
                'panda_link0',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            self.point_pub.publish(point_robot)

            self.get_logger().info(
                f"3D 좌표 (panda_link0 기준): "
                f"X={point_robot.point.x:.3f}m, "
                f"Y={point_robot.point.y:.3f}m, "
                f"Z={point_robot.point.z:.3f}m"
            )

        except tf2_ros.LookupException as e:
            self.get_logger().error(f"TF 조회 실패: {e}")
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().error(f"TF 시간 범위 초과: {e}")
        except tf2_ros.TransformException as e:
            self.get_logger().error(f"TF 변환 실패: {e}")
 
 
def main():
    rclpy.init()
    node = TFTransformerNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()