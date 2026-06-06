#!/usr/bin/env python3
"""
Franka Panda - 바나나 Pick and Place (Pilz Industrial Motion Planner)
=====================================================================
환경 : Ubuntu 22.04, ROS2 Humble, Isaac Sim 5.1.0
실행 : ros2 run cobot3 pick_and_place_pilz

방식:
  /compute_ik  → XYZ + 쿼터니언 → 관절값 획득
  /plan_kinematic_path (Pilz) → 경로 계획
  /panda_arm_controller/joint_trajectory → ros2_control 직접 발행

Pilz 플래너:
  PTP (Point to Point) : 관절 공간 최단 경로 (넓은 이동)
  LIN (Linear)         : 직선 경로 보장 (Pick 하강, Lift 상승)
"""

import time
import math
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionIK, GetMotionPlan
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint,
    BoundingVolume, RobotState
)
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration
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
#  포즈 정의 (XYZ + 쿼터니언)
#  position  : Isaac Sim Transform > Translate (panda_link0 기준, 미터)
#  quat_xyzw : Isaac Sim Orient(XYZ Euler, 도) -> 쿼터니언 변환
# ═══════════════════════════════════════════════════════════

POSE_DATA = {
    'Ready': {
        'pos' : [0.55466,  0.00433,  0.62109],
        'quat': [-0.999995,  0.001457, -0.002758,  0.000048],
        'planner': 'LIN',    # 관절 공간 최단 경로
        'vel': 0.3,
        'acc': 0.3,
        'dur': 3.0,
    },
    'Pre-pick': {
        'pos' : [0.55427,  0.00000,  0.35000],
        'quat': [-0.999996, -0.002382, -0.001431,  0.000012],
        'planner': 'LIN',
        'vel': 0.2,
        'acc': 0.2,
        'dur': 2.5,
    },
    'Pick': {
        'pos' : [0.55403, -0.00002,  0.12262],
        'quat': [-0.999996, -0.002443, -0.001475,  0.000012],
        'planner': 'LIN',    # 직선 하강 보장
        'vel': 0.1,
        'acc': 0.1,
        'dur': 2.0,
    },
    'Lift': {
        'pos' : [0.55941, -0.00117,  0.35000],   # Pre-pick 높이로 직선 상승
        'quat': [-0.999996, -0.002382, -0.001431,  0.000012],
        'planner': 'LIN',    # 직선 상승 보장
        'vel': 0.1,
        'acc': 0.1,
        'dur': 2.0,
    },
    'Pre-place': {
        'pos' : [0.39712, -0.39406,  0.35000],   # Place 위 높이
        'quat': [ 0.923798,  0.382416,  0.010853,  0.015378],
        'planner': 'LIN',
        'vel': 0.2,
        'acc': 0.2,
        'dur': 2.5,
    },
    'Place': {
        'pos' : [0.39362, -0.39102,  0.12303],
        'quat': [-0.924046, -0.382272, -0.002359,  0.001533],
        'planner': 'LIN',    # 직선 하강 보장
        'vel': 0.1,
        'acc': 0.1,
        'dur': 2.0,
    },
}


# ═══════════════════════════════════════════════════════════
#  노드
# ═══════════════════════════════════════════════════════════

class PickAndPlacePilzNode(Node):

    def __init__(self):
        super().__init__('pick_and_place_pilz')
        cb_group = ReentrantCallbackGroup()

        # trajectory 직접 발행
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/panda_arm_controller/joint_trajectory',
            10
        )

        # IK 서비스 클라이언트
        self.ik_client = self.create_client(
            GetPositionIK,
            '/compute_ik',
            callback_group=cb_group
        )

        # Pilz 경로 계획 서비스 클라이언트
        self.plan_client = self.create_client(
            GetMotionPlan,
            '/plan_kinematic_path',
            callback_group=cb_group
        )

        # 그리퍼
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

        # joint_states 구독 (IK seed 및 start_state 용)
        self._joint_state = None
        self.create_subscription(
            JointState, '/joint_states',
            lambda msg: setattr(self, '_joint_state', msg),
            10, callback_group=cb_group
        )

        # IK 연속성을 위한 마지막 관절값 저장
        self._last_joints = [0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.7854]

        print('PickAndPlacePilzNode 초기화 완료')

    # ───────────────────────────────────────────
    #  IK 계산
    # ───────────────────────────────────────────

    def _compute_ik(self, pos, quat):
        """/compute_ik 서비스로 XYZ + 쿼터니언 → 관절값 변환"""
        req = GetPositionIK.Request()
        req.ik_request.group_name   = ARM_GROUP
        req.ik_request.ik_link_name = END_EFFECTOR
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

        # seed 로 이전 관절값 사용 → IK 연속성 확보
        seed = JointState()
        seed.name     = ARM_JOINT_NAMES
        seed.position = self._last_joints
        req.ik_request.robot_state.joint_state = seed

        if not self.ik_client.wait_for_service(timeout_sec=3.0):
            print('  [ERROR] /compute_ik 서비스 없음')
            return None

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
            err = res.error_code.val if res else 'None'
            print(f'  [WARN] IK 실패 (error_code={err}) 이전 관절값 유지')
            return self._last_joints

    # ───────────────────────────────────────────
    #  Pilz 경로 계획 + 실행
    # ───────────────────────────────────────────

    def _plan_pilz(self, pos, quat, planner_id, vel, acc):
        """Pilz PTP/LIN 플래너로 경로 계획 → trajectory 반환"""
        req = GetMotionPlan.Request()
        mp  = MotionPlanRequest()

        mp.group_name       = ARM_GROUP
        mp.planner_id       = planner_id   # 'PTP' or 'LIN'
        mp.num_planning_attempts = 5
        mp.allowed_planning_time = 5.0
        mp.max_velocity_scaling_factor     = vel
        mp.max_acceleration_scaling_factor = acc

        # 시작 상태 = 현재 joint_states
        if self._joint_state is not None:
            mp.start_state.joint_state = self._joint_state

        # 목표 포즈 제약 조건
        constraints = Constraints()

        # Position constraint
        pos_c = PositionConstraint()
        pos_c.header.frame_id = BASE_LINK
        pos_c.link_name       = END_EFFECTOR
        pos_c.target_point_offset.x = 0.0
        pos_c.target_point_offset.y = 0.0
        pos_c.target_point_offset.z = 0.0

        box = SolidPrimitive()
        box.type        = SolidPrimitive.BOX
        box.dimensions  = [0.001, 0.001, 0.001]   # 허용 오차

        bv = BoundingVolume()
        bv.primitives.append(box)

        target_pose = PoseStamped()
        target_pose.header.frame_id    = BASE_LINK
        target_pose.pose.position.x    = pos[0]
        target_pose.pose.position.y    = pos[1]
        target_pose.pose.position.z    = pos[2]
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]
        bv.primitive_poses.append(target_pose.pose)

        pos_c.constraint_region = bv
        pos_c.weight = 1.0
        constraints.position_constraints.append(pos_c)

        # Orientation constraint
        ori_c = OrientationConstraint()
        ori_c.header.frame_id      = BASE_LINK
        ori_c.link_name            = END_EFFECTOR
        ori_c.orientation.x        = quat[0]
        ori_c.orientation.y        = quat[1]
        ori_c.orientation.z        = quat[2]
        ori_c.orientation.w        = quat[3]
        ori_c.absolute_x_axis_tolerance = 0.001
        ori_c.absolute_y_axis_tolerance = 0.001
        ori_c.absolute_z_axis_tolerance = 0.001
        ori_c.weight = 1.0
        constraints.orientation_constraints.append(ori_c)

        mp.goal_constraints.append(constraints)
        req.motion_plan_request = mp

        if not self.plan_client.wait_for_service(timeout_sec=3.0):
            print('  [ERROR] /plan_kinematic_path 서비스 없음')
            return None

        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        res = future.result()

        if res and res.motion_plan_response.error_code.val == 1:
            return res.motion_plan_response.trajectory.joint_trajectory
        else:
            err = res.motion_plan_response.error_code.val if res else 'None'
            print(f'  [WARN] Pilz {planner_id} 경로 계획 실패 (error_code={err})')
            print(f'         IK fallback 으로 전환')
            return None

    # ───────────────────────────────────────────
    #  이동 메서드
    # ───────────────────────────────────────────

    def move_to(self, key: str):
        """POSE_DATA 키로 이동 - Pilz 경로 계획 우선, 실패 시 IK fallback"""
        data = POSE_DATA[key]
        pos      = data['pos']
        quat     = data['quat']
        planner  = data['planner']
        vel      = data['vel']
        acc      = data['acc']
        dur      = data['dur']

        print(f'[{planner}] {key}  pos={[round(v,3) for v in pos]}')

        # 1. Pilz 경로 계획 시도
        traj = self._plan_pilz(pos, quat, planner, vel, acc)

        if traj is not None:
            # Pilz 계획 성공 → trajectory 직접 발행
            print(f'  Pilz {planner} 경로 계획 성공 → trajectory 발행')
            self.trajectory_pub.publish(traj)
        else:
            # Fallback: IK 계산 후 단순 trajectory 발행
            print(f'  IK fallback → 단순 관절값 이동')
            joints = self._compute_ik(pos, quat)
            if joints is None:
                print(f'  [ERROR] {key} 이동 실패')
                return

            msg   = JointTrajectory()
            msg.joint_names = ARM_JOINT_NAMES
            point = JointTrajectoryPoint()
            point.positions = joints
            point.time_from_start = Duration(
                sec=int(dur),
                nanosec=int((dur % 1) * 1e9)
            )
            msg.points.append(point)
            self.trajectory_pub.publish(msg)

        time.sleep(dur + 0.5)
        print(f'  ✓ {key} 완료')

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
        # joint_states 수신 확인
        print('joint_state 동기화 대기 중...')
        while self._joint_state is None:
            time.sleep(0.1)
        print('joint_state 수신 완료')

        print('=' * 55)
        print('  바나나 Pick and Place 시작 (Pilz LIN/PTP)')
        print('=' * 55)

        # 1. Ready (PTP)
        self.move_to('Ready')

        # 2. 그리퍼 열기
        self.gripper_open()

        # 3. Pre-pick (PTP)
        self.move_to('Pre-pick')

        # 4. Pick (LIN - 직선 하강)
        self.move_to('Pick')

        # 5. 그리퍼 닫기 + 안정화
        self.gripper_grip()
        time.sleep(1.5)

        # 6. Lift (LIN - 직선 상승)
        self.move_to('Lift')

        # 7. Pre-place (PTP)
        self.move_to('Pre-place')

        time.sleep(2.0)

        # 8. Place (LIN - 직선 하강)
        self.move_to('Place')

        time.sleep(2.0)

        # 9. 그리퍼 열기 (놓기)
        self.gripper_open()
        time.sleep(1.0)

        # 10. 복귀: Pre-place → Ready (PTP)
        self.move_to('Pre-place')
        self.move_to('Ready')

        print('=' * 55)
        print('  Pick and Place 완료')
        print('=' * 55)


# ═══════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PickAndPlacePilzNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    executor_thread = Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    print('서비스 연결 대기 중... (2초)')
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