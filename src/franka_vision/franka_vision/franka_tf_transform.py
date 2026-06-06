import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import tf2_ros
import tf2_geometry_msgs
 
 
class TFTransformNode(Node):
    def __init__(self):
        super().__init__('tf_transform')
 
        # TF 버퍼 및 리스너
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
 
        # /banana/point_camera 구독
        self.create_subscription(
            PointStamped,
            '/banana/point_camera',
            self.point_callback,
            10
        )
 
        # /banana/point_world 발행
        self.pub = self.create_publisher(
            PointStamped,
            '/banana/point_world',
            10
        )
 
        self.get_logger().info("TF 변환 노드 시작")
        self.get_logger().info("구독: /banana/point_camera")
        self.get_logger().info("발행: /banana/point_world")
 
    def point_callback(self, msg):
        try:
            msg.header.stamp = rclpy.time.Time().to_msg()
            # Camera 프레임 → world 프레임 변환
            point_world = self.tf_buffer.transform(
                msg,
                'world',
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
 
            self.get_logger().info(
                f"[world 기준] "
                f"X={point_world.point.x:.3f}m, "
                f"Y={point_world.point.y:.3f}m, "
                f"Z={point_world.point.z:.3f}m"
            )
 
            # /banana/point_world 발행
            self.pub.publish(point_world)
 
        except Exception as e:
            self.get_logger().warn(f"TF 변환 실패: {e}")
 
 
def main():
    rclpy.init()
    node = TFTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()
 
 
if __name__ == '__main__':
    main()