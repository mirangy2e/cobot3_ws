#!/usr/bin/env python3
"""
Franka Panda - 바나나 Pick and Place (XYZ 좌표 기반)
=====================================================
환경 : Ubuntu 22.04, ROS2 Humble, Isaac Sim 5.1.0
실행 : ros2 run cobot3 pick_and_place_pose

pymoveit2 예제(ex_pose_goal.py) 방식을 그대로 적용:
  - use_move_group_action 사용 안 함 (기본값 False)
  - MoveIt2 초기화 먼저 → executor spin 시작 → sleep(1) → 실행
  - planner_id 명시: RRTConnectkConfigDefault
"""

import math
import time
from threading import Thread

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from pymoveit2 import MoveIt2
from pymoveit2.gripper_command import GripperCommand


# ═══════════════════════════════════════════════════════════
#  로봇 설정
# ═══════════════════════════════════════════════════════════

ARM_JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7',
]
GRIPPER_JOINT_NAMES = ['panda_finger_joint1', 'panda_finger_joint2']

BASE_LINK    = 'panda_link0'
END_EFFECTOR = 'panda_link8'
ARM_GROUP    = 'panda_arm'

GRIPPER_OPEN  = 0.04
GRIPPER_CLOSE = 0.0


# ═══════════════════════════════════════════════════════════
#  관절값 정의 (Ready - 관절값 방식)
# ═══════════════════════════════════════════════════════════

def d2r(deg_list):
    return [math.radians(d) for d in deg_list]

JOINT_READY = d2r([0, 0, 0, -90, 0, 90, 45])


# ═══════════════════════════════════════════════════════════
#  포즈 정의 (XYZ + 쿼터니언)
#
#  position  : Isaac Sim Transform > Translate (미터)
#  quat_xyzw : Isaac Sim Orient(XYZ Euler, 도) -> 쿼터니언 변환
#              from scipy.spatial.transform import Rotation
#              q = Rotation.from_euler('xyz',[rx,ry,rz],degrees=True).as_quat()
# ═══════════════════════════════════════════════════════════

# Pre-pick  EE: (0.554,  0.000,  0.350)  Orient: (-179.999, -0.164,  0.273)
# Z를 0.216 -> 0.350 으로 올려서 접근 여유 확보
POSE_PRE_PICK = {
    'position' : [0.55427,  0.00000,  0.35000],
    'quat_xyzw': [-0.999996, -0.002382, -0.001431,  0.000012],
}

# Pick      EE: (0.554,  0.000,  0.123)  Orient: (-179.999, -0.169,  0.280)
POSE_PICK = {
    'position' : [0.55403, -0.00002,  0.12262],
    'quat_xyzw': [-0.999996, -0.002443, -0.001475,  0.000012],
}

# Lift      EE: (0.559, -0.001,  0.222)  Orient: ( 178.171,  1.137,  0.358)
POSE_LIFT = {
    'position' : [0.55941, -0.00117,  0.22206],
    'quat_xyzw': [ 0.999818,  0.003282, -0.009871,  0.015990],
}

# Pre-place EE: (0.397, -0.394,  0.222)  Orient: ( 177.896, -0.475, 44.984)
POSE_PRE_PLACE = {
    'position' : [0.39712, -0.39406,  0.22210],
    'quat_xyzw': [ 0.923798,  0.382416,  0.010853,  0.015378],
}

# Place     EE: (0.394, -0.391,  0.123)  Orient: (-179.941, -0.317, 44.949)
POSE_PLACE = {
    'position' : [0.39362, -0.39102,  0.12303],
    'quat_xyzw': [-0.924046, -0.382272, -0.002359,  0.001533],
}


# ═══════════════════════════════════════════════════════════
#  노드
# ═══════════════════════════════════════════════════════════

class PickAndPlacePoseNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_pose')
        cb_group = ReentrantCallbackGroup()

        # ── 팔 제어 ─────────────────────────────────────────
        # use_move_group_action=True 필수
        # -> Isaac Sim은 /move_action 을 통해야 실제로 로봇이 움직임
        self.arm = MoveIt2(
            node=self,
            joint_names=ARM_JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=ARM_GROUP,
            callback_group=cb_group,
            use_move_group_action=True,   # Isaac Sim 필수
        )
        # RViz2 Context 탭과 동일한 설정
        self.arm.planner_id            = "RRTConnectkConfigDefault"
        self.arm.max_velocity          = 0.1
        self.arm.max_acceleration      = 0.1
        self.arm.planning_time         = 5.0
        self.arm.num_planning_attempts = 10

        # ── 그리퍼 제어 ──────────────────────────────────────
        self.gripper = GripperCommand(
            node=self,
            gripper_joint_names=GRIPPER_JOINT_NAMES,
            open_gripper_joint_positions=[GRIPPER_OPEN,  GRIPPER_OPEN],
            closed_gripper_joint_positions=[GRIPPER_CLOSE, GRIPPER_CLOSE],
            max_effort=50.0,
            ignore_new_calls_while_executing=False,
            callback_group=cb_group,
            gripper_command_action_name='/panda_gripper_controller/gripper_cmd',
        )

    # ───────────────────────────────────────────
    #  이동 메서드
    # ───────────────────────────────────────────

    def move_to_pose(self, position, quat_xyzw, label='',
                     vel_scale=0.1, acc_scale=0.1, cartesian=False):
        self.arm.max_velocity     = vel_scale
        self.arm.max_acceleration = acc_scale
        mode = 'CARTESIAN' if cartesian else 'JOINT'
        print(f'[POSE/{mode}] {label}  pos={[round(v,3) for v in position]}  (vel={vel_scale})')
        self.arm.move_to_pose(
            position=position,
            quat_xyzw=quat_xyzw,
            cartesian=cartesian,
            frame_id=BASE_LINK,   # TF 실제 루트 프레임 (= panda_link0)
        )
        self.arm.wait_until_executed()
        time.sleep(0.5)

    def move_to_joint(self, joint_positions, label='',
                      vel_scale=0.1, acc_scale=0.1):
        self.arm.max_velocity     = vel_scale
        self.arm.max_acceleration = acc_scale
        print(f'[JOINT] {label}  (vel={vel_scale})')
        self.arm.move_to_configuration(joint_positions=joint_positions)
        self.arm.wait_until_executed()
        time.sleep(0.5)

    # ───────────────────────────────────────────
    #  그리퍼
    # ───────────────────────────────────────────

    def gripper_open(self):
        print('[GRIPPER] Open')
        self.gripper.open()
        time.sleep(5.0)

    def gripper_grip(self):
        print('[GRIPPER] Grip')
        self.gripper.close()
        time.sleep(5.0)

    # ───────────────────────────────────────────
    #  메인 시퀀스
    # ───────────────────────────────────────────

    def run(self):
        # joint_state 수신 확인 후 시작
        print('joint_state 동기화 대기 중...')
        while self.arm.joint_state is None:
            time.sleep(0.1)
        print(f'joint_state 수신 완료: {list(self.arm.joint_state.name)}')

        print('=' * 55)
        print('  바나나 Pick and Place 시작 (XYZ 좌표 기반)')
        print('=' * 55)

        # 1. Ready (관절값)
        self.move_to_joint(JOINT_READY, '1. Ready')

        # 2. 그리퍼 열기
        self.gripper_open()

        # 3. Pre-pick
        self.move_to_pose(**POSE_PRE_PICK, label='3. Pre-pick',cartesian=True)

        # 4. Pick (직선 하강 - cartesian=True 로 Z축 직선 경로 강제)
        # Pre-pick -> Pick 구간만 cartesian: OMPL 대신 직선 이동으로 정밀 제어
        self.move_to_pose(**POSE_PICK, label='4. Pick',
                          vel_scale=0.05, acc_scale=0.05, cartesian=True)

        # 5. 그리퍼 닫기 + 안정화
        self.gripper_grip()
        time.sleep(1.5)

        # 6. Lift (직선 상승 - cartesian=True 로 Z축 직선 경로 강제)
        # 바나나를 쥔 상태에서 직선으로 들어올려 흔들림 최소화
        self.move_to_pose(**POSE_LIFT, label='6. Lift',
                          vel_scale=0.05, acc_scale=0.05, cartesian=True)

        # 7. Pre-place
        self.move_to_pose(**POSE_PRE_PLACE, label='7. Pre-place',
                          vel_scale=0.1, acc_scale=0.1,cartesian=True)

        time.sleep(2.0)

        # 8. 그리퍼 열기 (놓기)
        self.gripper_open()
        time.sleep(1.0)

        # 9. 복귀: Pre-pick -> Ready
        self.move_to_pose(**POSE_PRE_PICK, label='9. Retreat -> Pre-pick',
                          vel_scale=0.1, acc_scale=0.1)
        self.move_to_joint(JOINT_READY, '9-2. Retreat -> Ready')

        print('=' * 55)
        print('  Pick and Place 완료')
        print('=' * 55)


# ═══════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PickAndPlacePoseNode()

    # 예제 방식: MoveIt2 초기화 먼저 → executor spin → sleep(1) → 실행
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    # 예제와 동일하게 1초 대기
    # (create_rate(1.0).sleep() 대신 time.sleep 사용 - Isaac Sim 환경 안정적)
    print('move_group 연결 대기 중... (2초)')
    time.sleep(2.0)

    try:
        node.run()
    except KeyboardInterrupt:
        print('사용자 중단 (Ctrl+C)')
    finally:
        rclpy.shutdown()
        executor_thread.join()


if __name__ == '__main__':
    main()