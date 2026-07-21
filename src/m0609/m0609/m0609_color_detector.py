#!/usr/bin/env python3
"""
색상 감지 노드
    /rgb 토픽 수신 → 파랑/초록 감지 → /color_id 발행

실행:
    export ROS_DOMAIN_ID=50
    python3 color_detector_node.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2
import numpy as np
from cv_bridge import CvBridge


# ── 파라미터 ────────────────────────────────────────────────
RGB_TOPIC    = "/rgb"
COLOR_TOPIC  = "/color_id"
DOMAIN_ID    = 50

# HSV 범위 — 조명·환경에 따라 조정
BLUE_LOWER   = np.array([100, 100, 70])
BLUE_UPPER   = np.array([130, 255, 255])

GREEN_LOWER  = np.array([40,  80,  80])
GREEN_UPPER  = np.array([80,  255, 255])

MIN_AREA     = 500   # 노이즈 제거용 최소 픽셀 면적
# ────────────────────────────────────────────────────────────


class ColorDetectorNode(Node):

    def __init__(self):
        super().__init__("color_detector_node")
        self._bridge   = CvBridge()
        self._color_id = 0   # 0=미감지, 1=파랑, 2=초록

        self._sub = self.create_subscription(
            Image, RGB_TOPIC, self._image_callback, 10
        )
        self._pub = self.create_publisher(Int32, COLOR_TOPIC, 10)

        self.get_logger().info(
            f"color_detector_node 시작\n"
            f"  구독: {RGB_TOPIC}\n"
            f"  발행: {COLOR_TOPIC}\n"
            f"  기다리는 중..."
        )

    # ── 이미지 콜백 ─────────────────────────────────────────
    def _image_callback(self, msg: Image):
        # 1. ROS Image → OpenCV BGR
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # 2. BGR → HSV
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # 3. 마스크 생성
        mask_blue  = cv2.inRange(hsv, BLUE_LOWER,  BLUE_UPPER)
        mask_green = cv2.inRange(hsv, GREEN_LOWER, GREEN_UPPER)

        # 4. 노이즈 제거 (Morphology)
        kernel     = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask_blue  = cv2.morphologyEx(mask_blue,  cv2.MORPH_OPEN,  kernel)
        mask_blue  = cv2.morphologyEx(mask_blue,  cv2.MORPH_CLOSE, kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN,  kernel)
        mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

        # 5. 면적 계산
        area_blue  = cv2.countNonZero(mask_blue)
        area_green = cv2.countNonZero(mask_green)

        # 6. 색상 판단
        prev_id = self._color_id

        if area_blue > MIN_AREA and area_blue > area_green:
            self._color_id = 1
        elif area_green > MIN_AREA and area_green > area_blue:
            self._color_id = 2
        else:
            self._color_id = 0

        # 7. color_id 발행
        msg_out      = Int32()
        msg_out.data = self._color_id
        self._pub.publish(msg_out)

        # 8. 상태 변경 시 로그
        if self._color_id != prev_id:
            label = {0: "미감지", 1: "파랑 → Place 1", 2: "초록 → Place 2"}
            self.get_logger().info(
                f"color_id = {self._color_id} ({label[self._color_id]})"
                f"  blue_area={area_blue}  green_area={area_green}"
            )

        # 9. 디버그 시각화 (선택)
        self._visualize(bgr, mask_blue, mask_green)

    # ── 디버그 창 (종료: q 키) ───────────────────────────────
    def _visualize(self, bgr, mask_blue, mask_green):
        label = {0: "NONE", 1: "BLUE (id=1)", 2: "GREEN (id=2)"}
        color = {0: (128,128,128), 1: (255,0,0), 2: (0,255,0)}

        vis = bgr.copy()
        cv2.putText(
            vis,
            f"color_id = {self._color_id}  {label[self._color_id]}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            color[self._color_id], 2
        )

        # 윤곽선만 그리기 → 잔상 없음
        contours_blue,  _ = cv2.findContours(mask_blue,  cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(vis, contours_blue,  -1, (255,   0,   0), 2)
        cv2.drawContours(vis, contours_green, -1, (0,   200,   0), 2)

        cv2.imshow("color_detector", vis)
        cv2.waitKey(1)


# ── 메인 ────────────────────────────────────────────────────
def main():
    import os
    os.environ["ROS_DOMAIN_ID"] = str(DOMAIN_ID)

    rclpy.init()
    node = ColorDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()