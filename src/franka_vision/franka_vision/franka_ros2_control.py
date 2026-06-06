#!/usr/bin/env python3
"""
Franka Panda 원격 제어 노드
- panda_arm: 좌 / 우 왕복 이동
- hand: 그리퍼 열기 / 닫기
Isaac Sim 5.0.0 + MoveIt2 연동 확인용
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration as RclpyDuration

from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

import time


# ─────────────────────────────────────────
#  관절 포즈 정의
# ─────────────────────────────────────────

# panda_arm 7개 관절 이름
ARM_JOINTS = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'
]

# hand 2개 관절 이름
HAND_JOINTS = ['panda_finger_joint1', 'panda_finger_joint2']

# ── 팔 포즈 ──
# 홈 포즈 (MoveIt2 기본 ready 포즈)
POSE_HOME   = [0.0, -0.785, 0.0, -2.356, 0.0,  1.571,  0.785]
# 오른쪽 포즈 (joint1 +45도)
POSE_RIGHT  = [0.785, -0.785, 0.0, -2.356, 0.0,  1.571,  0.785]
# 왼쪽 포즈 (joint1 -45도)
POSE_LEFT   = [-0.785, -0.785, 0.0, -2.356, 0.0,  1.571,  0.785]

# ── 그리퍼 포즈 (GripperCommand: 손가락 1개 기준 위치, 단위 m) ──
GRIPPER_OPEN  = 0.04   # 최대 40mm
GRIPPER_CLOSE = 0.0    # 완전 닫힘


class FrankaRemoteController(Node):

    def __init__(self):
        super().__init__('franka_remote_controller')

        # ── Action Clients ──
        self._arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/panda_arm_controller/follow_joint_trajectory'
        )
        self._hand_client = ActionClient(
            self,
            GripperCommand,
            '/panda_gripper_controller/gripper_cmd'
        )

        # ── Joint State Subscriber (연결 확인용) ──
        self._joint_state_received = False
        self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10
        )

        self.get_logger().info('FrankaRemoteController 초기화 완료')
        self.get_logger().info('Isaac Sim + MoveIt2 서버 대기 중...')

    # ────────────────────────────────────
    #  연결 확인
    # ────────────────────────────────────

    def _joint_state_cb(self, msg):
        if not self._joint_state_received:
            self._joint_state_received = True
            self.get_logger().info(
                f'/joint_states 수신 확인 → 관절 수: {len(msg.name)}'
            )

    def wait_for_servers(self, timeout_sec=10.0):
        """Action 서버 및 토픽 연결 대기"""
        self.get_logger().info('Action 서버 연결 중...')

        arm_ready  = self._arm_client.wait_for_server(timeout_sec=timeout_sec)
        hand_ready = self._hand_client.wait_for_server(timeout_sec=timeout_sec)

        if not arm_ready:
            self.get_logger().error(
                'panda_arm_controller 연결 실패! '
                'Isaac Sim이 실행 중이고 ▶ Play 상태인지 확인하세요.'
            )
            return False
        if not hand_ready:
            self.get_logger().error(
                'hand_controller 연결 실패! '
                '컨트롤러 이름을 ros2 control list_controllers 로 확인하세요.'
            )
            return False

        self.get_logger().info('✅ 모든 Action 서버 연결 완료')
        return True

    # ────────────────────────────────────
    #  Goal 생성 헬퍼
    # ────────────────────────────────────

    @staticmethod
    def _make_goal(joint_names, positions, duration_sec):
        """FollowJointTrajectory Goal 메시지 생성"""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joint_names

        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.velocities = [0.0] * len(positions)
        pt.time_from_start = Duration(sec=int(duration_sec))

        goal.trajectory.points = [pt]
        return goal

    # ────────────────────────────────────
    #  액션 전송 (동기 대기)
    # ────────────────────────────────────

    def _send_goal_sync(self, client, goal, label='', is_gripper=False):
        """Goal 전송 후 완료까지 블로킹 대기"""
        self.get_logger().info(f'  → {label} 전송 중...')

        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f'  ✗ {label} Goal 거부됨')
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if is_gripper:
            # GripperCommand 결과: reached_goal(bool), stalled(bool)
            self.get_logger().info(f'  ✓ {label} 완료 (reached={result.reached_goal})')
        else:
            # FollowJointTrajectory 결과: error_code == 0 이면 성공
            if result.error_code != 0:
                self.get_logger().warn(f'  △ {label} 완료 (error_code={result.error_code})')
            else:
                self.get_logger().info(f'  ✓ {label} 완료')
        return True

    # ────────────────────────────────────
    #  팔 제어
    # ────────────────────────────────────

    def move_arm(self, positions, duration_sec=3.0, label=''):
        goal = self._make_goal(ARM_JOINTS, positions, duration_sec)
        return self._send_goal_sync(self._arm_client, goal, label)

    # ────────────────────────────────────
    #  그리퍼 제어
    # ────────────────────────────────────

    def move_gripper(self, position, label=''):
        goal = GripperCommand.Goal()
        goal.command.position = position
        goal.command.max_effort = 50.0
        return self._send_goal_sync(self._hand_client, goal, label, is_gripper=True)

    # ────────────────────────────────────
    #  시퀀스 실행
    # ────────────────────────────────────

    def run_sequence(self, repeat=2):
        """
        좌우 왕복 + 그리퍼 열고닫기 시퀀스
        repeat: 왕복 반복 횟수
        """
        self.get_logger().info('━━━ 시퀀스 시작 ━━━')

        # 0) 홈 포즈로 이동
        self.get_logger().info('[0] 홈 포즈')
        self.move_arm(POSE_HOME, duration_sec=3.0, label='HOME')
        self.move_gripper(GRIPPER_OPEN, label='그리퍼 OPEN')
        time.sleep(0.5)

        for i in range(repeat):
            self.get_logger().info(f'━━━ 반복 {i+1}/{repeat} ━━━')

            # 1) 오른쪽 이동
            self.get_logger().info('[1] 오른쪽 이동')
            self.move_arm(POSE_RIGHT, duration_sec=3.0, label='RIGHT')
            time.sleep(0.3)

            # 2) 그리퍼 닫기
            self.get_logger().info('[2] 그리퍼 닫기')
            self.move_gripper(GRIPPER_CLOSE, label='그리퍼 CLOSE')
            time.sleep(0.3)

            # 3) 왼쪽 이동
            self.get_logger().info('[3] 왼쪽 이동')
            self.move_arm(POSE_LEFT, duration_sec=3.0, label='LEFT')
            time.sleep(0.3)

            # 4) 그리퍼 열기
            self.get_logger().info('[4] 그리퍼 열기')
            self.move_gripper(GRIPPER_OPEN, label='그리퍼 OPEN')
            time.sleep(0.3)

        # 5) 홈 복귀
        self.get_logger().info('[5] 홈 복귀')
        self.move_arm(POSE_HOME, duration_sec=3.0, label='HOME')
        self.move_gripper(GRIPPER_OPEN, label='그리퍼 OPEN')

        self.get_logger().info('━━━ 시퀀스 완료 ━━━')


# ─────────────────────────────────────────
#  메인
# ─────────────────────────────────────────

def main():
    rclpy.init()
    node = FrankaRemoteController()

    try:
        # Action 서버 연결 확인
        if not node.wait_for_servers(timeout_sec=10.0):
            node.get_logger().error('서버 연결 실패. 종료합니다.')
            return

        # 시퀀스 실행 (2회 반복)
        node.run_sequence(repeat=2)

    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 (Ctrl+C)')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()