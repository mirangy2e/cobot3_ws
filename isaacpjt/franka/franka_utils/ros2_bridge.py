# ~/cobot3_ws/isaacpjt/franka/utils/ros2_bridge.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
import numpy as np


class ROS2Bridge(Node):
    def __init__(self):
        super().__init__('lula_ros2_bridge')

        self.target_position = None   # [x, y, z]
        self.new_target      = False

        self.create_subscription(
            PointStamped,
            '/banana/point_world',
            self.point_callback,
            10
        )
        self.get_logger().info("ROS2Bridge ready — subscribing /banana/point_world")

    def point_callback(self, msg):
        self.target_position = np.array([
            msg.point.x,
            msg.point.y,
            msg.point.z
        ])
        self.new_target = True
        self.get_logger().info(
            f"Target received: "
            f"X={msg.point.x:.3f}, "
            f"Y={msg.point.y:.3f}, "
            f"Z={msg.point.z:.3f}"
        )