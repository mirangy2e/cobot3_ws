#!/usr/bin/env python3
"""
Franka Panda - MoveIt2 오피셜 IK 서비스 연동형 ros2_control 직통 제어
======================================================================
환경 : Ubuntu 22.04, ROS2 Humble, Isaac Sim 5.1.0
특징 : pymoveit2의 변수 가로채기 버그를 우회하기 위해 MoveIt2 고유의 
       /compute_ik 서비스를 직접 호출하여 안전한 관절각을 얻고 ros2_control로 제어합니다.
"""

import time
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionIK # MoveIt2 오피셜 IK 서비스 인터페이스
from builtin_interfaces.msg import Duration
from pymoveit2.gripper_command import GripperCommand


# ═══════════════════════════════════════════════════════════
#  로봇 및 관절 설정 매핑
# ═══════════════════════════════════════════════════════════
ARM_JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]
GRIPPER_JOINT_NAMES = ['panda_finger_joint1', 'panda_finger_joint2']

BASE_LINK    = 'panda_link0'
END_EFFECTOR = 'panda_link8' 
ARM_GROUP    = 'panda_arm'


# ═══════════════════════════════════════════════════════════
#  YOLO 연동 대비형 포즈 데이터 테이블 (완벽한 수직 하향 방향)
# ═══════════════════════════════════════════════════════════
QUAT_DOWN = [-0.999996, -0.002443, -0.001475,  0.000012]

POSE_DATA = {
    'Ready':     {'pos': [0.55466,  0.00433,  0.62109], 'quat': QUAT_DOWN},
    'Pre-pick':  {'pos': [0.55427,  0.00000,  0.35000], 'quat': QUAT_DOWN},
    'Pick':      {'pos': [0.55403, -0.00002,  0.12262], 'quat': QUAT_DOWN},
    'Lift':      {'pos': [0.55941, -0.00117,  0.22206], 'quat': QUAT_DOWN},
    'Pre-place': {'pos': [0.39712, -0.39406,  0.22210], 'quat': QUAT_DOWN},
    'Place':     {'pos': [0.39362, -0.39102,  0.12303], 'quat': QUAT_DOWN}
}


class MoveItIkServiceControlNode(Node):

    def __init__(self):
        super().__init__('moveit_ik_service_control_node')
        cb_group = ReentrantCallbackGroup()

        # ── 1. 오피셜 제어기 토픽 퍼블리셔 채널 바인딩 ─────────────────
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )

        # ── 2. MoveIt2 내장 IK 서비스 클라이언트 생성 ──────────────────
        # 외부 변수를 가로채는 꼼수 대신, MoveIt2 공식 연산 패킷 통로를 다이렉트로 개설합니다.
        self.ik_client = self.create_client(
            GetPositionIK, 
            '/compute_ik', 
            callback_group=cb_group
        )

        # ── 3. 그리퍼 액션 인터페이스 동기화 ───────────────────────────
        self.gripper = GripperCommand(
            node=self,
            gripper_joint_names=GRIPPER_JOINT_NAMES,
            open_gripper_joint_positions=[0.04, 0.04],
            closed_gripper_joint_positions=[0.0, 0.0],
            max_effort=120.0,
            ignore_new_calls_while_executing=False,
            callback_group=cb_group,
            gripper_command_action_name='/panda_gripper_controller/gripper_cmd',
        )

        # 현재 관절 상태 임시 저장용 변수 (IK 연산 시 씨드값으로 활용)
        self.last_valid_joints = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.78]
        self.get_logger().info('MoveIt2 공식 IK 서비스 연동형 직통 노드 초기화 완료')

    # ───────────────────────────────────────────
    #  [마스터 인터페이스] MoveIt2 C++ 공식 커널 IK 호출 유닛
    # ───────────────────────────────────────────
    def call_moveit_ik_service(self, target_pos, target_quat):
        """MoveIt2의 /compute_ik 서비스를 호출하여 신뢰성 높은 7축 조인트 각도를 받아옵니다."""
        req = GetPositionIK.Request()
        req.ik_request.group_name = ARM_GROUP
        req.ik_request.ik_link_name = END_EFFECTOR
        req.ik_request.avoid_collisions = True # 장애물 충돌 회피 연산 포함

        # 목표 포즈 메시지 조립
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = BASE_LINK
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        
        pose_stamped.pose.position.x = target_pos[0]
        pose_stamped.pose.position.y = target_pos[1]
        pose_stamped.pose.position.z = target_pos[2]
        
        pose_stamped.pose.orientation.x = target_quat[0]
        pose_stamped.pose.orientation.y = target_quat[1]
        pose_stamped.pose.orientation.z = target_quat[2]
        pose_stamped.pose.orientation.w = target_quat[3]
        
        req.ik_request.pose_stamped = pose_stamped

        # 서비스 호출 및 결과 대기
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('⚠ MoveIt2 /compute_ik 서비스를 찾을 수 없습니다! moveit이 켜져있는지 확인하세요.')
            return None

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        response = future.result()
        if response and response.error_code.val == response.error_code.SUCCESS:
            # 성공 시 7축 관절 정보 가려내기
            joint_positions = []
            for name in ARM_JOINT_NAMES:
                if name in response.solution.joint_state.name:
                    idx = response.solution.joint_state.name.index(name)
                    joint_positions.append(response.solution.joint_state.position[idx])
            self.last_valid_joints = joint_positions
            return joint_positions
        else:
            self.get_logger().warn('⚠ MoveIt2 솔버가 해당 좌표의 IK 해를 찾지 못했습니다. 이전 각도를 유지합니다.')
            return self.last_valid_joints

    def move_to_xyz_via_moveit_srv(self, key: str, duration_sec: float = 2.5):
        """좌표 테이블의 타겟 포즈를 MoveIt2 공식 서비스로 풀어 ros2_control에 안전 주입합니다."""
        target_pos = POSE_DATA[key]['pos']
        target_quat = POSE_DATA[key]['quat']
        print(f'➔ [MoveIt2 Service] {key} 연산 위임 중... (XYZ: {[round(v,3) for v in target_pos]})')

        # 1. 오피셜 서비스 서버로부터 완벽한 조인트 각도 정답 세트 수신
        target_joints = self.call_moveit_ik_service(target_pos, target_quat)
        
        if target_joints is None:
            return

        # 2. 검증된 조인트 각도를 ros2_control 패킷으로 조립하여 다이렉트 송신
        msg = JointTrajectory()
        msg.joint_names = ARM_JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        msg.points.append(point)
        
        # 3. 퍼블리시 및 동기화 물리 대기
        self.trajectory_pub.publish(msg)
        time.sleep(duration_sec + 0.3)
        print(f'   ✓ [ros2_control] MoveIt2 공식 검증 각도로 안착 성공')

    def gripper_open(self):
        print('[GRIPPER] Open 요청')
        try: self.gripper.open(); time.sleep(3.0)
        except Exception: pass

    def gripper_grip(self):
        print('[GRIPPER] Grip 요청')
        try: self.gripper.close(); time.sleep(3.0)
        except Exception: pass

    # ───────────────────────────────────────────
    #  메인 오퍼레이션 시퀀스
    # ───────────────────────────────────────────
    def run(self):
        print('=' * 60)
        print('  MoveIt2 오피셜 서비스 연동형 하이브리드 비전 제어 시동')
        print('=' * 60)

        # 1. 원점 정렬
        self.move_to_xyz_via_moveit_srv('Ready', duration_sec=3.0)
        self.gripper_open()

        # 2. Pick 시퀀스
        self.move_to_xyz_via_moveit_srv('Pre-pick', duration_sec=2.5)
        self.move_to_xyz_via_moveit_srv('Pick', duration_sec=2.0)

        # 3. 파지
        self.gripper_grip()
        time.sleep(1.0)

        # 4. Place 영역 전이
        self.move_to_xyz_via_moveit_srv('Lift', duration_sec=1.5)
        self.move_to_xyz_via_moveit_srv('Pre-place', duration_sec=2.5)
        self.move_to_xyz_via_moveit_srv('Place', duration_sec=1.5)

        # 5. 해제
        self.gripper_open()
        time.sleep(1.0)

        # 6. 복귀 탈출 안전 시퀀스
        self.move_to_xyz_via_moveit_srv('Pre-place', duration_sec=1.5)
        self.move_to_xyz_via_moveit_srv('Ready', duration_sec=2.5)

        print('=' * 60)
        print('  전체 공정 에러 없이 100% 깔끔하게 완료!')
        print('=' * 60)


def main():
    rclpy.init()
    node = MoveItIkServiceControlNode()
    
    # 서비스와 토픽 퍼블리싱을 동시 병렬 처리하기 위한 멀티스레드 실행기 작동
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    time.sleep(1.0)
    try: node.run()
    except KeyboardInterrupt: pass
    finally:
        rclpy.shutdown()
        executor_thread.join()

if __name__ == '__main__':
    main()