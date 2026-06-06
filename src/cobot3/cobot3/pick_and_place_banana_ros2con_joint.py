#!/usr/bin/env python3
"""
Franka Panda - 바나나 Pick and Place
=====================================
환경 : Ubuntu 22.04, ROS2 Humble, Isaac Sim 5.1.0
실행 : ros2 run cobot3 pick_and_place_banana

제어 방식:
  팔    : /panda_arm_controller/joint_trajectory 토픽 직접 발행
  그리퍼 : /panda_gripper_controller/gripper_cmd 액션 클라이언트

사전 조건:
  1. Isaac Sim Play 상태 (물리 시뮬레이션 + /joint_states 발행)
  2. ros2 launch panda_moveit_config demo.launch.py 실행
     (controller_manager + panda_arm_controller + panda_gripper_controller 활성화)
"""

import rclpy
import math
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand
from builtin_interfaces.msg import Duration


# ═══════════════════════════════════════════════════════════
#  로봇 설정
# ═══════════════════════════════════════════════════════════

ARM_JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]

# 팔 컨트롤러 토픽 (JointTrajectoryController)
ARM_TRAJECTORY_TOPIC = '/panda_arm_controller/joint_trajectory'

# 그리퍼 컨트롤러 액션 (GripperActionController)
GRIPPER_ACTION_NAME = '/panda_gripper_controller/gripper_cmd'

# 그리퍼 위치 (단위: 미터, 양쪽 합산 기준)
GRIPPER_OPEN_POSITION  = 0.08   # 완전 개방
GRIPPER_CLOSE_POSITION = 0.00   # 완전 닫힘


# ═══════════════════════════════════════════════════════════
#  관절값 정의 (도 -> 라디안)
#
#  Isaac Sim MotionPlanning 패널 > Joints 탭에서 확인한 값
#  단계         J1   J2  J3    J4  J5   J6   J7
# ═══════════════════════════════════════════════════════════

def d2r(deg_list: list) -> list:
    return [math.radians(d) for d in deg_list]

JOINT_READY     = d2r([  0,   0,  0,  -90,  0,   90,  45])
JOINT_PRE_PICK  = d2r([  0,  20,  0, -126,  0,  147,  45])  # EE: ( 0.554,  0.000,  0.216)
JOINT_PICK      = d2r([  0,  33,  0, -124,  0,  157,  45])  # EE: ( 0.554,  0.000,  0.123)
JOINT_LIFT      = d2r([  0,  20,  0, -126,  0,  147,  45])  # EE: ( 0.559, -0.001,  0.222)
JOINT_PRE_PLACE = d2r([-45,  20,  0, -126,  0,  147,  45])  # EE: ( 0.397, -0.394,  0.222)


# ═══════════════════════════════════════════════════════════
#  노드
# ═══════════════════════════════════════════════════════════

class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_banana')
        cb_group = ReentrantCallbackGroup()

        # 팔 제어: JointTrajectory 토픽 퍼블리셔
        # panda_arm_controller(JointTrajectoryController)가 구독
        # demo.launch.py 실행 후 controller_manager가 활성화해야 사용 가능
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            ARM_TRAJECTORY_TOPIC,
            10
        )

        # 그리퍼 제어: GripperCommand 액션 클라이언트
        # panda_gripper_controller(GripperActionController)의 액션 서버에 연결
        # open/close 두 명령만 지원하며 완료 신호(result)를 반환함
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            GRIPPER_ACTION_NAME,
            callback_group=cb_group
        )

    # ───────────────────────────────────────────
    #  팔 이동
    # ───────────────────────────────────────────

    def move_to_joint(self, joint_positions: list, label: str = '',
                      duration_sec: float = 2.5):
        """
        JointTrajectory 메시지를 구성하여 panda_arm_controller에 발행

        joint_positions : 목표 관절값 (라디안, 7개)
        duration_sec    : 목표 도달 허용 시간 (초)
                          time_from_start 에 설정되며 ros2_control이
                          이 시간 안에 목표값에 도달하도록 보간함
                          값이 작을수록 빠르게, 클수록 부드럽게 이동
        """
        self.get_logger().info(f'[ARM] {label}  (duration={duration_sec}s)')

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

        # 완료 신호가 없으므로 time_from_start 만큼 대기
        time.sleep(duration_sec + 0.3)

    # ───────────────────────────────────────────
    #  그리퍼 제어
    # ───────────────────────────────────────────

    def _send_gripper_goal(self, position: float, label: str = ''):
        """
        GripperCommand 액션 goal을 panda_gripper_controller에 전송

        position   : 그리퍼 목표 위치 (미터, 양쪽 합산)
        max_effort : 파지력 (N)
        """
        self.get_logger().info(f'[GRIPPER] {label}')

        if not self.gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'그리퍼 액션 서버({GRIPPER_ACTION_NAME}) 없음')
            return

        goal = GripperCommand.Goal()
        goal.command.position   = position
        goal.command.max_effort = 150.0

        # 비동기 전송 후 동기 대기
        future = self.gripper_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('그리퍼 goal 거절됨')
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
        print('=' * 55)
        print('  바나나 Pick and Place 시작')
        print('=' * 55)

        # 1. Ready — 초기 자세
        self.move_to_joint(JOINT_READY, '1. Ready', duration_sec=3.0)

        # 2. 그리퍼 열기
        self.gripper_open()

        # 3. Pre-pick — 바나나 위 접근
        self.move_to_joint(JOINT_PRE_PICK, '3. Pre-pick', duration_sec=2.5)

        # 4. Pick — 바나나 높이 하강 (충격 최소화를 위해 천천히)
        self.move_to_joint(JOINT_PICK, '4. Pick', duration_sec=1.8)

        # 5. 파지
        self.gripper_grip()

        # 6. Lift — 들어올리기 (관성 최소화를 위해 천천히)
        self.move_to_joint(JOINT_LIFT, '6. Lift', duration_sec=1.8)

        # 7. Pre-place — J1만 -45도 회전하여 place 위치로 이동
        self.move_to_joint(JOINT_PRE_PLACE, '7. Pre-place', duration_sec=2.5)

        # 8. 그리퍼 열기 (바나나 놓기)
        self.gripper_open()

        # 9. 복귀 — Pre-pick 경유
        self.move_to_joint(JOINT_PRE_PICK, '9. Retreat -> Pre-pick', duration_sec=2.5)

        print('=' * 55)
        print('  Pick and Place 완료')
        print('=' * 55)


# ═══════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PickAndPlaceNode()

    # ActionClient와 토픽 발행을 동시에 처리하기 위해 MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    time.sleep(1.0)  # 하드웨어 채널 동기화 대기

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()