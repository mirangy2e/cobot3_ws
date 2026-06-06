#!/usr/bin/env python3
"""
Franka Panda - 바나나 Pick and Place
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
환경: Ubuntu 22.04, ROS2 Humble, Isaac Sim 5.1.0
패키지: pymoveit2, panda_moveit_config, cobot3_ws

실행:
  ros2 run cobot3 pick_and_place_banana --ros-args -p use_sim_time:=true

동작 시퀀스:
  1. Ready        → 초기 자세
  2. Gripper Open → 그리퍼 열기
  3. Pre-pick     → 바나나 위 접근
  4. Pick         → 바나나 높이 하강
  5. Gripper Grip → 파지
  6. Lift         → 들어올리기
  7. Pre-place    → 45도 회전
  8. Place        → 하강
  9. Gripper Open → 놓기
 10. Retreat      → Ready 복귀
"""

import rclpy
import math
import time
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# ✅ FIX 1: control_msgs 대신 pymoveit2.gripper_command 사용
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
#  관절값 정의 (도 → 라디안)
#
#  단계         J1    J2  J3    J4  J5   J6   J7   EE 위치(참고)
# ═══════════════════════════════════════════════════════════

def d2r(deg_list: list) -> list:
    return [math.radians(d) for d in deg_list]

#                              J1   J2  J3    J4  J5   J6   J7
JOINT_READY     = d2r([       0,   0,  0,  -90,  0,   90,  45])
JOINT_PRE_PICK  = d2r([       0,  20,  0, -126,  0,  147,  45])  # EE: ( 0.554,  0.000,  0.216)
JOINT_PICK      = d2r([       0,  33,  0, -124,  0,  157,  45])  # EE: ( 0.554,  0.000,  0.123)
JOINT_LIFT      = d2r([       0,  20,  0, -126,  0,  147,  45])  # EE: ( 0.559, -0.001,  0.222)
JOINT_PRE_PLACE = d2r([     -45,  20,  0, -126,  0,  147,  45])  # EE: ( 0.397, -0.394,  0.222)
JOINT_PLACE     = d2r([     -45,  33,  0, -124,  0,  157,  45])  # EE: ( 0.394, -0.391,  0.123)

# 참고용 포즈 (Cartesian - 현재 미사용)
POSE_PRE_PICK  = {"position": [0.55427,  0.00000,  0.21584]}
POSE_PICK      = {"position": [0.55403, -0.00002,  0.12262]}
POSE_LIFT      = {"position": [0.55941, -0.00117,  0.22206]}
POSE_PRE_PLACE = {"position": [0.39712, -0.39406,  0.22210]}
POSE_PLACE     = {"position": [0.39362, -0.39102,  0.12303]}


# ═══════════════════════════════════════════════════════════
#  Pick and Place Node
# ═══════════════════════════════════════════════════════════

class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_banana')
        cb_group = ReentrantCallbackGroup()

        # ── 팔 제어 ─────────────────────────────────────
        self.arm = MoveIt2(
            node=self,
            joint_names=ARM_JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=ARM_GROUP,
            callback_group=cb_group,
            use_move_group_action=True,   # ✅ FIX 2: 계획+실행 atomic (start tolerance 오류 방지)
        )
        self.arm.max_velocity     = 0.3
        self.arm.max_acceleration = 0.3

        # ── 그리퍼 제어 ──────────────────────────────────
        # ✅ FIX 1: pymoveit2.gripper_command.GripperCommand 사용
        #           action명: /panda_gripper_controller/gripper_cmd
        self.gripper = GripperCommand(
            node=self,
            gripper_joint_names=GRIPPER_JOINT_NAMES,
            open_gripper_joint_positions=[GRIPPER_OPEN,  GRIPPER_OPEN],
            closed_gripper_joint_positions=[GRIPPER_CLOSE, GRIPPER_CLOSE],
            max_effort=50.0,
            ignore_new_calls_while_executing=True,
            callback_group=cb_group,
            gripper_command_action_name='/panda_gripper_controller/gripper_cmd',
        )

        self.get_logger().info('PickAndPlaceNode 초기화 완료')

    # ───────────────────────────────────────────
    #  이동 메서드
    # ───────────────────────────────────────────

    def move_to_joint(self, joint_positions: list, label: str = ''):
        self.get_logger().info(f'▶ [JOINT] {label}')
        self.arm.move_to_configuration(joint_positions=joint_positions)
        self.arm.wait_until_executed()
        time.sleep(0.5)

    # ───────────────────────────────────────────
    #  그리퍼 메서드
    # ───────────────────────────────────────────

    def gripper_open(self):
        self.get_logger().info('▶ [GRIPPER] Open')
        self.gripper.open()
        time.sleep(5)   # ✅ FIX 3: Isaac Sim은 result 미반환 → sleep으로 대기
        self.get_logger().info('  ✓ Gripper Open 완료')

    def gripper_grip(self):
        self.get_logger().info('▶ [GRIPPER] Grip')
        self.gripper.close()
        time.sleep(5)   # ✅ FIX 3: 동일
        self.get_logger().info('  ✓ Gripper Grip 완료')

    # ───────────────────────────────────────────
    #  메인 시퀀스
    # ───────────────────────────────────────────

    def run(self):
        self.get_logger().info('=' * 55)
        self.get_logger().info('   바나나 Pick and Place 시작')
        self.get_logger().info('=' * 55)

        # 1. Ready
        self.move_to_joint(JOINT_READY,     '1. Ready')

        # 2. Gripper Open
        self.gripper_open()

        # 3. Pre-pick
        self.move_to_joint(JOINT_PRE_PICK,  '3. Pre-pick   EE→(0.554, 0.000, 0.216)')

        # 4. Pick (하강)
        self.move_to_joint(JOINT_PICK,      '4. Pick       EE→(0.554, 0.000, 0.123)')

        # 5. Grip
        self.gripper_grip()

        # 6. Lift
        self.move_to_joint(JOINT_LIFT,      '6. Lift       EE→(0.559,-0.001, 0.222)')

        # 7. Pre-place (45도 회전)
        self.move_to_joint(JOINT_PRE_PLACE, '7. Pre-place  EE→(0.397,-0.394, 0.222)')

        # 8. Place (하강)
        self.move_to_joint(JOINT_PLACE,     '8. Place      EE→(0.394,-0.391, 0.123)')

        # 9. Gripper Open (놓기)
        self.gripper_open()

        # 10. Retreat
        self.move_to_joint(JOINT_READY,     '10. Retreat → Ready')

        self.get_logger().info('=' * 55)
        self.get_logger().info('   Pick and Place 완료!')
        self.get_logger().info('=' * 55)


# ═══════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PickAndPlaceNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # ✅ FIX 4: move_group 연결 안정화 대기 (joint_states 타이밍 문제 방지)
    node.get_logger().info('move_group 연결 대기 중... (2초)')
    time.sleep(2.0)

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('중단됨 (KeyboardInterrupt)')
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()