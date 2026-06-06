#!/usr/bin/env python3
"""
Franka Panda - 바나나 Pick and Place (최종)
==========================================
환경 : Ubuntu 22.04, ROS2 Humble, Isaac Sim 5.1.0
실행 : ros2 run cobot3 pick_and_place_banana --ros-args -p use_sim_time:=true

동작 순서:
  1. Ready
  2. 그리퍼 열기
  3. Pre-pick    EE -> ( 0.554,  0.000,  0.216)
  4. Pick        EE -> ( 0.554,  0.000,  0.123)
  5. 파지
  6. Lift        EE -> ( 0.559, -0.001,  0.222)
  7. Pre-place   EE -> ( 0.397, -0.394,  0.222)
  8. 그리퍼 열기 (놓기)
  9. 복귀: Pre-place -> Pre-pick

참고:
  STATUS_CANCELED 에러는 정상 동작 중 발생하는 예상된 에러.
  ignore_new_calls_while_executing=False 설정이 있어야
  gripper.open() 명령이 Isaac Sim에 실제로 전달됨.
"""

import rclpy
import math
import time
import threading
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
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
#  노드
# ═══════════════════════════════════════════════════════════

class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_banana')
        cb_group = ReentrantCallbackGroup()

        # 팔 제어
        self.arm = MoveIt2(
            node=self,
            joint_names=ARM_JOINT_NAMES,
            base_link_name=BASE_LINK,
            end_effector_name=END_EFFECTOR,
            group_name=ARM_GROUP,
            callback_group=cb_group,
            use_move_group_action=True,  # 계획+실행 atomic -> start tolerance 오류 방지
        )
        self.arm.max_velocity     = 0.1
        self.arm.max_acceleration = 0.1

        # 그리퍼 제어
        # ignore_new_calls_while_executing=False 필수
        # -> True 이면 gripper.open() 명령이 내부적으로 무시되어 Isaac Sim에 미전달
        # -> 팔 이동 시작 시 STATUS_CANCELED 에러 로그가 찍히지만 동작에는 무관
        self.gripper = GripperCommand(
            node=self,
            gripper_joint_names=GRIPPER_JOINT_NAMES,
            open_gripper_joint_positions=[GRIPPER_OPEN,  GRIPPER_OPEN],
            closed_gripper_joint_positions=[GRIPPER_CLOSE, GRIPPER_CLOSE],
            max_effort=150.0,
            ignore_new_calls_while_executing=False,
            callback_group=cb_group,
            gripper_command_action_name='/panda_gripper_controller/gripper_cmd',
        )

        self.get_logger().info('PickAndPlaceNode 초기화 완료')

    # ───────────────────────────────────────────
    #  팔 이동
    # ───────────────────────────────────────────

    def move_to_joint(self, joint_positions: list, label: str = '',
                      vel_scale: float = 0.3, acc_scale: float = 0.3):
        self.arm.max_velocity     = vel_scale
        self.arm.max_acceleration = acc_scale
        self.get_logger().info(f'[JOINT] {label}  (vel={vel_scale})')
        self.arm.move_to_configuration(joint_positions=joint_positions)
        self.arm.wait_until_executed()
        time.sleep(0.5)

    # ───────────────────────────────────────────
    #  그리퍼
    # ───────────────────────────────────────────

    def gripper_open(self):
        self.get_logger().info('[GRIPPER] Open')
        self.gripper.open()
        time.sleep(5.0)

    def gripper_grip(self):
        self.get_logger().info('[GRIPPER] Grip')
        self.gripper.close()
        time.sleep(5.0)

    # ───────────────────────────────────────────
    #  메인 시퀀스
    # ───────────────────────────────────────────

    def run(self):
        self.get_logger().info('=' * 50)
        self.get_logger().info('  바나나 Pick and Place 시작')
        self.get_logger().info('=' * 50)

        # 1. Ready
        self.move_to_joint(JOINT_READY, '1. Ready')

        # 2. 그리퍼 열기
        self.gripper_open()

        # 3. Pre-pick
        self.move_to_joint(JOINT_PRE_PICK, '3. Pre-pick')

        # 4. Pick (충격 최소화를 위해 느리게 하강)
        self.move_to_joint(JOINT_PICK, '4. Pick',
                           vel_scale=0.15, acc_scale=0.1)

        # 5. 파지 + 물리 안정화 대기
        self.gripper_grip()
        time.sleep(1.5)

        # 6. Lift (휘청거림 방지를 위해 매우 느리게)
        self.move_to_joint(JOINT_LIFT, '6. Lift',
                           vel_scale=0.1, acc_scale=0.08)

        # 7. Pre-place (45도 회전 이동)
        self.move_to_joint(JOINT_PRE_PLACE, '7. Pre-place',
                           vel_scale=0.2, acc_scale=0.1)

        time.sleep(2.0)  # place 안정화 대기

        # 8. 그리퍼 열기 (바나나 놓기)
        self.gripper_open()
        time.sleep(1.0)

        # 9. 복귀: Pre-place -> Pre-pick
        self.move_to_joint(JOINT_PRE_PICK, '9. Retreat -> Pre-pick',
                           vel_scale=0.1, acc_scale=0.05)

        self.get_logger().info('=' * 50)
        self.get_logger().info('  바나나 Pick and Place 완료')
        self.get_logger().info('=' * 50)


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

    node.get_logger().info('move_group 연결 대기 중... (2초)')
    time.sleep(2.0)

    try:
        node.run()
    except KeyboardInterrupt:
        node.get_logger().info('사용자 중단 (Ctrl+C)')
    finally:
        rclpy.shutdown()
        spin_thread.join()


if __name__ == '__main__':
    main()