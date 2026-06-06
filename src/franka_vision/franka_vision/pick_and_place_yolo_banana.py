#!/usr/bin/env python3

import rclpy
import math
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PointStamped, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration


# ═══════════════════════════════════════════════════════════
#  로봇 설정
# ═══════════════════════════════════════════════════════════

ARM_JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]

BASE_LINK    = 'panda_link0'
END_EFFECTOR = 'panda_link8'
ARM_GROUP    = 'panda_arm'

ARM_TRAJECTORY_TOPIC = '/panda_arm_controller/joint_trajectory'
GRIPPER_ACTION_NAME  = '/panda_gripper_controller/gripper_cmd'

GRIPPER_OPEN_POSITION  = 0.08
GRIPPER_CLOSE_POSITION = 0.00


# ═══════════════════════════════════════════════════════════
#  Pick and Place 설정
# ═══════════════════════════════════════════════════════════

# EE orientation 고정 (X축 수평, Y축 수직 하향)
# Roll=180도 = [qx=1, qy=0, qz=0, qw=0]
QUAT_DOWN = [1.0, 0.0, 0.0, 0.0]

# Z 고정값 (panda_link0 기준, 단위: 미터)
PICK_Z     = 0.123   # 바나나 파지 높이 (실측값)
PRE_PICK_Z = 0.350   # 바나나 위 접근 높이

# XY 보정값 - TF 변환 오차 보정 (단위: 미터)
# 바나나에 부딪힐 경우 값을 조정
OFFSET_X = 0.0    # X축 보정
OFFSET_Y = -0.02    # Y축 보정

# Place 고정 위치
POSE_PRE_PLACE = {'pos': [0.39712, -0.39406, 0.450],   'quat': QUAT_DOWN}
POSE_PLACE     = {'pos': [0.39362, -0.39102, 0.20303], 'quat': QUAT_DOWN}

# Ready 관절값
def d2r(deg_list):
    return [math.radians(d) for d in deg_list]

JOINT_READY = d2r([0, 0, 0, -90, 0, 90, 45])


# ═══════════════════════════════════════════════════════════
#  노드
# ═══════════════════════════════════════════════════════════

class PickAndPlaceYoloBananaNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_yolo_banana')
        cb_group = ReentrantCallbackGroup()

        # 팔 제어: JointTrajectory 토픽 퍼블리셔
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            ARM_TRAJECTORY_TOPIC,
            10
        )

        # IK 서비스 클라이언트
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=cb_group
        )

        # 그리퍼 액션 클라이언트
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            GRIPPER_ACTION_NAME,
            callback_group=cb_group
        )

        # 바나나 위치 구독
        self.create_subscription(
            PointStamped,
            '/banana/point_world',
            self._banana_pos_cb,
            10,
            callback_group=cb_group
        )

        self._banana_pos = None
        self._running    = False
        self._lock       = threading.Lock()
        # Pick 자세 기준 seed (RViz 실측값)
        self._last_joints = d2r([0, 34, 0, -122, 0, 156, 45])

        self.get_logger().info('pick_and_place_yolo_banana 노드 시작')
        self.get_logger().info('/banana/point_world 대기 중...')

    # ───────────────────────────────────────────
    #  바나나 위치 콜백
    # ───────────────────────────────────────────

    def _banana_pos_cb(self, msg: PointStamped):
        """바나나 위치 수신 → 시퀀스 시작 (중복 실행 방지)"""
        with self._lock:
            if self._running:
                return
            self._running = True
            self._banana_pos = msg

        self.get_logger().info(
            f'바나나 위치 수신: '
            f'X={msg.point.x:.3f}, Y={msg.point.y:.3f}, Z={msg.point.z:.3f}'
        )
        threading.Thread(target=self.run, daemon=True).start()

    # ───────────────────────────────────────────
    #  IK 계산
    # ───────────────────────────────────────────

    def _compute_ik(self, pos: list, quat: list) -> list:
        """XYZ + 쿼터니언 → 관절값"""
        req = GetPositionIK.Request()
        req.ik_request.group_name       = ARM_GROUP
        req.ik_request.ik_link_name     = END_EFFECTOR
        req.ik_request.avoid_collisions = True

        pose = PoseStamped()
        pose.header.frame_id    = BASE_LINK
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.pose.position.x    = pos[0]
        pose.pose.position.y    = pos[1]
        pose.pose.position.z    = pos[2]
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        req.ik_request.pose_stamped = pose

        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('/compute_ik 서비스 없음')
            return self._last_joints

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        res = future.result()

        if res and res.error_code.val == res.error_code.SUCCESS:
            joints = []
            for name in ARM_JOINT_NAMES:
                if name in res.solution.joint_state.name:
                    idx = res.solution.joint_state.name.index(name)
                    joints.append(res.solution.joint_state.position[idx])
            self._last_joints = joints
            return joints
        else:
            self.get_logger().warn(
                f'IK 실패 (error_code={res.error_code.val}) 이전 관절값 유지'
            )
            return self._last_joints

    # ───────────────────────────────────────────
    #  팔 이동
    # ───────────────────────────────────────────

    def _move_to_xyz(self, pos: list, quat: list,
                     label: str = '', duration_sec: float = 2.5):
        self.get_logger().info(
            f'[XYZ] {label}  '
            f'pos=[{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]  '
            f'(duration={duration_sec}s)'
        )
        joints = self._compute_ik(pos, quat)

        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joints
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        msg.points.append(point)
        self.arm_pub.publish(msg)
        time.sleep(duration_sec + 0.3)

    def _move_to_joint(self, joint_positions: list,
                       label: str = '', duration_sec: float = 2.5):
        self.get_logger().info(f'[JOINT] {label}  (duration={duration_sec}s)')
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(
            sec=int(duration_sec),
            nanosec=int((duration_sec % 1) * 1e9)
        )
        msg.points.append(point)
        self.arm_pub.publish(msg)
        time.sleep(duration_sec + 0.3)

    # ───────────────────────────────────────────
    #  그리퍼
    # ───────────────────────────────────────────

    def _send_gripper_goal(self, position: float, label: str = ''):
        self.get_logger().info(f'[GRIPPER] {label}')
        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('그리퍼 액션 서버 없음')
            return
        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = 150.0
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=3.0)
        time.sleep(0.5)

    def gripper_open(self):
        self._send_gripper_goal(GRIPPER_OPEN_POSITION, 'Open')

    def gripper_grip(self):
        self._send_gripper_goal(GRIPPER_CLOSE_POSITION, 'Grip')

    # ───────────────────────────────────────────
    #  메인 시퀀스
    # ───────────────────────────────────────────

    def run(self):
        try:
            # YOLO 탐지 결과: X, Y 사용
            # Z는 바나나 실제 높이 고정값 사용
            bx = self._banana_pos.point.x # offset 추가
            by = self._banana_pos.point.y

            print('=' * 55)
            print(f'  바나나 Pick and Place 시작')
            print(f'  바나나 XY: X={bx:.3f}, Y={by:.3f}')
            print(f'  바나나 Z : {PICK_Z} (고정값)')
            print('=' * 55)

            # 1. Ready
            self._move_to_joint(JOINT_READY, '1. Ready', duration_sec=3.0)

            # 2. 그리퍼 열기
            self.gripper_open()

            # 3. Pre-pick (바나나 위 접근)
            self._move_to_xyz(
                [bx + OFFSET_X, by + OFFSET_Y, PRE_PICK_Z], QUAT_DOWN,
                '3. Pre-pick', duration_sec=3.5
            )
            time.sleep(3.0)

            # 4. Pick (바나나 높이 하강)
            self._move_to_xyz(
                [bx + OFFSET_X, by + OFFSET_Y, PICK_Z], QUAT_DOWN,
                '4. Pick', duration_sec=3.0
            )
            time.sleep(3.0)

            # 5. 파지
            self.gripper_grip()
            time.sleep(1.0)

            # 6. Lift (들어올리기)
            self._move_to_xyz(
                [bx + OFFSET_X, by + OFFSET_Y, PRE_PICK_Z], QUAT_DOWN,
                '6. Lift', duration_sec=1.8
            )

            # 7. Pre-place
            self._move_to_xyz(
                POSE_PRE_PLACE['pos'], POSE_PRE_PLACE['quat'],
                '7. Pre-place', duration_sec=2.5
            )

            # 8. Place (하강)
            self._move_to_xyz(
                POSE_PLACE['pos'], POSE_PLACE['quat'],
                '8. Place', duration_sec=1.5
            )

            time.sleep(1.0)

            # 9. 그리퍼 열기 (놓기)
            self.gripper_open()
            time.sleep(1.0)

            # 10. 복귀: Pre-place → Ready
            self._move_to_xyz(
                POSE_PRE_PLACE['pos'], POSE_PRE_PLACE['quat'],
                '10-1. Retreat -> Pre-place', duration_sec=1.5
            )
            self._move_to_joint(
                JOINT_READY, '10-2. Retreat -> Ready', duration_sec=2.5
            )

            print('=' * 55)
            print('  Pick and Place 완료')
            print('=' * 55)

        finally:
            self._running    = False
            self._banana_pos = None
            self.get_logger().info('/detected_object/xyz_robot 대기 중...')


# ═══════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PickAndPlaceYoloBananaNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()