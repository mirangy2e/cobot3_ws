#!/usr/bin/env python3
"""
Franka Panda - 바나나 Pick and Place (XYZ 좌표 기반)
=====================================================
환경 : Ubuntu 22.04, ROS2 Humble, Isaac Sim 5.1.0
실행 : ros2 run cobot3 pick_and_place_banana_xyz

제어 방식:
  팔    : /compute_ik 서비스로 XYZ → 관절값 변환 후
          /panda_arm_controller/joint_trajectory 토픽 발행
  그리퍼 : /panda_gripper_controller/gripper_cmd 액션 클라이언트

사전 조건:
  1. Isaac Sim Play 상태 (물리 시뮬레이션 + /joint_states 발행)
  2. ros2 launch panda_moveit_config demo.launch.py 실행
     (controller_manager + panda_arm_controller + panda_gripper_controller 활성화
      + move_group 실행 → /compute_ik 서비스 제공)
"""

import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
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

BASE_LINK    = 'panda_link0'   # IK 계산 기준 프레임 (TF 루트)
END_EFFECTOR = 'panda_link8'   # IK 목표 링크
ARM_GROUP    = 'panda_arm'     # MoveIt2 Planning Group 이름

# 팔 컨트롤러 토픽 (JointTrajectoryController)
ARM_TRAJECTORY_TOPIC = '/panda_arm_controller/joint_trajectory'

# 그리퍼 컨트롤러 액션 (GripperActionController)
GRIPPER_ACTION_NAME = '/panda_gripper_controller/gripper_cmd'

# 그리퍼 위치 (단위: 미터, 양쪽 합산 기준)
GRIPPER_OPEN_POSITION  = 0.08   # 완전 개방
GRIPPER_CLOSE_POSITION = 0.00   # 완전 닫힘


# ═══════════════════════════════════════════════════════════
#  포즈 데이터 테이블 (XYZ + 쿼터니언)
#
#  position  : Isaac Sim Transform > Translate (panda_hand 기준, 미터)
#  quat_xyzw : Isaac Sim Orient(XYZ Euler, 도) → 쿼터니언 변환
#              from scipy.spatial.transform import Rotation
#              q = Rotation.from_euler('xyz',[rx,ry,rz],degrees=True).as_quat()
#
#  QUAT_DOWN : Pick 자세의 실측 orientation
#              모든 단계에 동일하게 적용하여 IK 솔루션을 하나로 고정
#              → 쿼터니언을 느슨하게 설정하면 IK 솔루션이 여러 개가 되어
#                관절이 예측 불가능하게 움직임
# ═══════════════════════════════════════════════════════════

QUAT_DOWN = [-0.999996, -0.002443, -0.001475, 0.000012]

POSE_DATA = {
    'Ready':     {'pos': [0.55466,  0.00433,  0.62109], 'quat': QUAT_DOWN},
    'Pre-pick':  {'pos': [0.55427,  0.00000,  0.35000], 'quat': QUAT_DOWN},  # Z=0.35: 바나나 위 여유 확보
    'Pick':      {'pos': [0.55403, -0.00002,  0.12262], 'quat': QUAT_DOWN},
    'Lift':      {'pos': [0.55941, -0.00117,  0.22206], 'quat': QUAT_DOWN},
    'Pre-place': {'pos': [0.39712, -0.39406,  0.22210], 'quat': QUAT_DOWN},
    'Place':     {'pos': [0.39362, -0.39102,  0.12303], 'quat': QUAT_DOWN},
}


# ═══════════════════════════════════════════════════════════
#  노드
# ═══════════════════════════════════════════════════════════

class PickAndPlaceBananaXyzNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_banana_xyz')
        cb_group = ReentrantCallbackGroup()

        # 팔 제어: JointTrajectory 토픽 퍼블리셔
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            ARM_TRAJECTORY_TOPIC,
            10
        )

        # IK 서비스 클라이언트
        # move_group이 제공하는 /compute_ik 서비스로 XYZ → 관절값 변환
        # demo.launch.py 실행 후 move_group이 떠 있어야 사용 가능
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=cb_group
        )

        # 그리퍼 제어: GripperCommand 액션 클라이언트
        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            GRIPPER_ACTION_NAME,
            callback_group=cb_group
        )

        # IK 실패 시 사용할 이전 관절값 (연속성 유지)
        self._last_joints = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.78]

    # ───────────────────────────────────────────
    #  IK 계산
    # ───────────────────────────────────────────

    def _compute_ik(self, pos: list, quat: list) -> list:
        """
        /compute_ik 서비스로 XYZ + 쿼터니언 → 관절값 변환

        pos  : [x, y, z] (panda_hand 기준, 미터)
        quat : [qx, qy, qz, qw]

        반환 : 7개 관절값 (라디안)
               IK 실패 시 이전 관절값 반환
        """
        req = GetPositionIK.Request()
        req.ik_request.group_name    = ARM_GROUP
        req.ik_request.ik_link_name  = END_EFFECTOR
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
            self.get_logger().error('/compute_ik 서비스 없음 (move_group 실행 확인)')
            return self._last_joints

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
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
            self.get_logger().warn(f'IK 실패 (error_code={res.error_code.val}) 이전 관절값 유지')
            return self._last_joints

    # ───────────────────────────────────────────
    #  팔 이동
    # ───────────────────────────────────────────

    def move_to_xyz(self, key: str, label: str = '', duration_sec: float = 2.5):
        """
        POSE_DATA 키로 XYZ 이동

        1. /compute_ik 서비스로 관절값 계산
        2. JointTrajectory 메시지 구성 후 토픽 발행
        3. duration_sec 만큼 대기 (완료 신호 없음)
        """
        pos  = POSE_DATA[key]['pos']
        quat = POSE_DATA[key]['quat']
        self.get_logger().info(
            f'[XYZ] {label}  pos={[round(v,3) for v in pos]}  (duration={duration_sec}s)'
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
        print('  바나나 Pick and Place 시작 (XYZ 좌표 기반)')
        print('=' * 55)

        # 1. Ready — 초기 자세
        self.move_to_xyz('Ready', '1. Ready', duration_sec=3.0)

        # 2. 그리퍼 열기
        self.gripper_open()

        # 3. Pre-pick — 바나나 위 접근 (Z=0.35, 충분한 여유)
        self.move_to_xyz('Pre-pick', '3. Pre-pick', duration_sec=2.5)

        # 4. Pick — 바나나 높이 하강 (충격 최소화를 위해 천천히)
        self.move_to_xyz('Pick', '4. Pick', duration_sec=1.8)

        # 5. 파지
        self.gripper_grip()

        # 6. Lift — 들어올리기 (관성 최소화를 위해 천천히)
        self.move_to_xyz('Lift', '6. Lift', duration_sec=1.8)

        # 7. Pre-place — place 위치 위로 이동
        self.move_to_xyz('Pre-place', '7. Pre-place', duration_sec=2.5)

        # 8. Place — 하강
        self.move_to_xyz('Place', '8. Place', duration_sec=1.5)

        # 9. 그리퍼 열기 (바나나 놓기)
        self.gripper_open()

        # 10. 복귀 — Pre-place 경유 → Ready
        self.move_to_xyz('Pre-place', '10-1. Retreat -> Pre-place', duration_sec=1.5)
        self.move_to_xyz('Ready',     '10-2. Retreat -> Ready',     duration_sec=2.5)

        print('=' * 55)
        print('  Pick and Place 완료')
        print('=' * 55)


# ═══════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PickAndPlaceBananaXyzNode()

    # 서비스 통신과 토픽 발행을 동시에 처리하기 위해 MultiThreadedExecutor 사용
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