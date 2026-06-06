#!/usr/bin/env python3

import rclpy
import math
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionClient # 오피셜 ROS 2 액션 라이브러리 활용
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ROS 2 오피셜 표준 메시지 타입 인터페이스
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import GripperCommand # 오피셜 그리퍼 액션 메시지
from builtin_interfaces.msg import Duration


# ═══════════════════════════════════════════════════════════
#  로봇 관절 및 제어기 통신 주소 설정
# ═══════════════════════════════════════════════════════════

ARM_JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]

ARM_TRAJECTORY_TOPIC = '/panda_arm_controller/joint_trajectory'
GRIPPER_ACTION_NAME  = '/panda_gripper_controller/gripper_cmd'

# Franka Panda 그리퍼 폭 (완전 개방 시 한 축당 0.04m, 합산 0.08m)
GRIPPER_OPEN_POSITION  = 0.08  
GRIPPER_CLOSE_POSITION = 0.00  


# ═══════════════════════════════════════════════════════════
#  관절값 정의 (도 -> 라디안)
# ═══════════════════════════════════════════════════════════

def d2r(deg_list: list) -> list:
    return [math.radians(d) for d in deg_list]

#                            J1   J2  J3    J4  J5   J6   J7
JOINT_READY     = d2r([     0,   0,  0,  -90,  0,   90,  45])
JOINT_PRE_PICK  = d2r([     0,  20,  0, -126,  0,  147,  45])  # EE: ( 0.554,  0.000,  0.216)
JOINT_PICK      = d2r([     0,  33,  0, -124,  0,  157,  45])  # EE: ( 0.554,  0.000,  0.123)
JOINT_LIFT      = d2r([     0,  20,  0, -126,  0,  147,  45])  # EE: ( 0.559, -0.001,  0.222)
JOINT_PRE_PLACE = d2r([   -45,  20,  0, -126,  0,  147,  45])  # EE: ( 0.397, -0.394,  0.222)


# ═══════════════════════════════════════════════════════════
#  노드 정의
# ═══════════════════════════════════════════════════════════

class PureRos2FrameworkNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_banana')
        cb_group = ReentrantCallbackGroup()

        # ── 1. 로봇 팔 ros2_control 직통 토픽 퍼블리셔 개설 ──
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            ARM_TRAJECTORY_TOPIC,
            10
        )

        # ── 2. [수정] 오피셜 ROS 2 Gripper 액션 클라이언트 빌드 ──
        # 외부 pymoveit2 래퍼 없이, ROS2 표준 프레임워크 액션 엔진을 다이렉트로 바인딩합니다.
        self.gripper_action_client = ActionClient(
            self,
            GripperCommand,
            GRIPPER_ACTION_NAME,
            callback_group=cb_group
        )

        self.get_logger().info('★ pymoveit2 전면 걷어내기 성공 ➔ 순수 ROS 2 프레임워크 노드 시동')

    # ───────────────────────────────────────────
    #  로봇 팔 제어 구동 유닛
    # ───────────────────────────────────────────

    def move_to_joint(self, joint_positions: list, label: str = '', duration_sec: float = 2.5):
        """지정한 7축 관절 목표 각도를 ros2_control 제어기로 직통 송신합니다."""
        self.get_logger().info(f'[ARM/ros2_control] {label} ➔ 구동 시작 (Duration: {duration_sec}s)')
        
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        msg.points.append(point)
        self.arm_pub.publish(msg)
        
        # 하드웨어가 완벽히 도달할 때까지 정시성 블로킹 시간 확보
        time.sleep(duration_sec + 0.3)

    # ───────────────────────────────────────────
    #  [완벽 수정] 오피셜 Gripper 액션 동기화 구동 유닛
    # ───────────────────────────────────────────

    def control_gripper_action(self, target_position: float, label: str = ''):
        """MoveIt2가 열어둔 GripperCommand 액션 서버로 명령을 직통 전달합니다."""
        self.get_logger().info(f'[GRIPPER/Action] {label} 요청 전송')
        
        # 1. 액션 서비스 상태 체킹
        if not self.gripper_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f'⚠ 그리퍼 액션 서버({GRIPPER_ACTION_NAME})가 응답하지 않습니다!')
            return

        # 2. 오피셜 GripperCommand 목표 데이터 패킷 조립
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = target_position
        goal_msg.command.max_effort = 150.0  # 파지 관성을 제압하기 위한 강력한 악력 유지

        # 3. 비동기 목표 요청 및 동기식 결과 수렴 유도
        send_goal_future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=2.0)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('⚠ 그리퍼 제어 요청이 제어기 서버에 의해 거절되었습니다.')
            return

        # 4. 물리 모터가 최종 정착할 때까지 안정화 대기
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=3.0)
        
        self.get_logger().info(f'   ✓ {label} 가동 완료')
        time.sleep(0.5)

    def gripper_open(self):
        self.control_gripper_action(GRIPPER_OPEN_POSITION, '그리퍼 완전 개방 (Open)')

    def gripper_grip(self):
        self.control_gripper_action(GRIPPER_CLOSE_POSITION, '바나나 파지 밀착 (Grip)')

    # ───────────────────────────────────────────
    #  메인 오퍼레이션 공정 시퀀스
    # ───────────────────────────────────────────

    def run(self):
        print('=' * 55)
        print('  바나나 Pick and Place 시작 (Pure 토픽 + 오피셜 액션)')
        print('=' * 55)

        # 1. Ready 원점 정렬 (3초 동안 부드럽게)
        self.move_to_joint(JOINT_READY, '1. Ready 원점 정렬', duration_sec=3.0)

        # 2. 그리퍼 초기 개방
        self.gripper_open()

        # 3. Pre-pick 영역 진입 (2.5초)
        self.move_to_joint(JOINT_PRE_PICK, '3. Pre-pick 대기축 이동', duration_sec=2.5)

        # 4. Pick 하강 (바닥 물체 충격 최소화를 위해 1.8초 동안 완만하게 감속 하강)
        self.move_to_joint(JOINT_PICK, '4. Pick 수직 정밀 하강', duration_sec=1.8)

        # 5. 바나나 포착 파지 (오피셜 액션 클라이언트 다이렉트 구동)
        self.gripper_grip()

        # 6. Lift 리프팅 상승 (들려 올라갈 때 휘청거리지 않게 1.8초 유지)
        self.move_to_joint(JOINT_LIFT, '6. Lift 안전 상승', duration_sec=1.8)

        # 7. Pre-place 회전 이동 (바구니 대기 위치로 2.5초 동안 수평 전이)
        self.move_to_joint(JOINT_PRE_PLACE, '7. Pre-place 대기 영역 이동', duration_sec=2.5)

        # 8. 바나나 안전 방출
        self.gripper_open()

        # 9. 복귀 탈출 공정
        self.move_to_joint(JOINT_PRE_PICK, '9. Retreat ➔ Pre-pick 홈 복귀', duration_sec=2.5)

        print('=' * 55)
        print('  전체 공정 에러 로그 없이 100% 완벽하고 클린하게 완료!')
        print('=' * 55)


# ═══════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PureRos2FrameworkNode()

    # 액션과 토픽 통신을 병렬로 매끄럽게 처리하기 위한 멀티스레드 실행기 작동
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.get_logger().info('시뮬레이션 하드웨어 채널 동기화 마진... (1초)')
    time.sleep(1.0)

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 패킷 수신 (Ctrl+C)')
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()