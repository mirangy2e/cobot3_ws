"""Lula IK 를 이용한 제어: 팔 형태를 유지하는 IK, 매 프레임 목표를 따라가는 컨트롤러, ik 이동 방식.

- KinematicsSolver : Isaac Sim 의 Lula IK 를 M0609 설정(URDF, 로봇 설명 파일, TCP)으로 만든 것
- GoalIK           : TCP 목표 -> 팔 관절값. 팔꿈치/손목이 뒤집히거나 뒤로 뻗는 해는 버린다
- IKController     : 매 호출 바뀌는 목표를 따라감. 관절이 한 번에 MAX_JOINT_STEP 이상 움직이지 않게 제한
                     (run_follow_target.py 에서 직접 사용)
- IKMotion         : pick and place 의 "ik" 이동 방식. 목표점을 TARGET_STEP 씩 옮기며 IKController 로 따라감

IK 는 같은 TCP 자세를 만드는 관절 해가 여러 개 있고 자세 선호가 없어서, 그대로 쓰면 팔이 다른 형태로
갑자기 점프한다 (예: 3번 관절이 펴짐). GoalIK 가 홈 자세와 같은 형태의 해만 돌려주도록 막는다.
"""

from typing import Optional, Tuple

import numpy as np
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver

import m0609_settings as settings

from .base import MotionBackend

NO_SOLUTION = "IK 해를 찾지 못했습니다."
OTHER_CONFIGURATION = "IK 해가 다른 팔 형태입니다."


def wrap_to_pi(angles):
    """각도를 -pi ~ pi 로 맞춘다 (q 와 q + 2pi 는 같은 자세)."""
    return (np.asarray(angles) + np.pi) % (2 * np.pi) - np.pi


class KinematicsSolver(ArticulationKinematicsSolver):
    """Lula IK + 시뮬레이션 로봇 연결. TCP 프레임 기준으로 IK / 정기구학을 계산한다."""

    def __init__(self, robot_articulation: SingleArticulation) -> None:
        self._kinematics = LulaKinematicsSolver(
            robot_description_path=str(settings.ROBOT_DESCRIPTION_PATH),
            urdf_path=str(settings.URDF_PATH),
        )
        # 로봇이 원점에 있지 않아도 맞게 계산하도록 베이스 위치를 알려줌
        base_position, base_orientation = robot_articulation.get_world_pose()
        self._kinematics.set_robot_base_pose(base_position, base_orientation)
        # 현재 관절값에서 시작한 IK 가 실패하면, 무작위 대신 홈 자세에서 먼저 다시 시도
        self._kinematics.set_default_cspace_seeds(np.array([np.radians(settings.HOME_JOINTS_DEG)]))
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, settings.TCP_FRAME)


class GoalIK:
    """TCP 목표를 홈 자세와 같은 팔 형태의 관절값으로 바꾼다. 모든 이동 방식이 함께 쓴다."""

    def __init__(self, robot_articulation: SingleArticulation) -> None:
        self._robot = robot_articulation
        self._solver = KinematicsSolver(robot_articulation)
        self._kinematics = self._solver.get_kinematics_solver()
        self.joint_indices = np.array([robot_articulation.get_dof_index(name) for name in settings.ARM_JOINTS])
        self._home = np.radians(settings.HOME_JOINTS_DEG)
        self._branch_indices = np.array([settings.ARM_JOINTS.index(name) for name in settings.BRANCH_JOINTS])
        self._branch_signs = np.sign(self._home[self._branch_indices])
        self._base_position, _ = robot_articulation.get_world_pose()

    def current_joints(self) -> np.ndarray:
        """현재 팔 관절값 (rad)."""
        return self._robot.get_joint_positions()[self.joint_indices]

    def tcp_pose(self):
        """현재 관절값으로 계산한 TCP 위치와 회전 행렬."""
        return self._solver.compute_end_effector_pose()

    def solve(
        self, position: np.ndarray, orientation: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], Optional[str]]:
        """TCP 목표에 대한 팔 관절값(rad, -pi ~ pi)을 돌려준다. 실패하면 (None, 이유)."""
        # 시작점 두 가지로 차례로 풀어본다:
        # 1) 현재 관절값 (가장 가까운 해)  2) 홈 자세에서 joint_1 만 목표 방향으로 돌린 자세
        facing_home = self._home.copy()
        offset = np.asarray(position) - self._base_position
        facing_home[0] = np.arctan2(offset[1], offset[0])
        reason = NO_SOLUTION
        for seed in (self.current_joints(), facing_home):
            candidate, success = self._kinematics.compute_inverse_kinematics(
                settings.TCP_FRAME, position, orientation, seed
            )
            if not success:
                continue
            if self.same_configuration(candidate, facing_home[0]):
                return wrap_to_pi(candidate), None
            reason = OTHER_CONFIGURATION
        return None, reason

    def same_configuration(self, joints: np.ndarray, facing_angle: Optional[float] = None) -> bool:
        """홈 자세와 같은 팔 형태인지 검사한다.

        - BRANCH_JOINTS(팔꿈치, 손목)의 부호가 홈 자세와 같아야 함
        - facing_angle 을 주면 joint_1 이 그 방향에서 90 deg 안이어야 함 (팔을 뒤로 뻗는 해 제외)
        """
        if np.any(np.sign(joints[self._branch_indices]) != self._branch_signs):
            return False
        return facing_angle is None or abs(wrap_to_pi(joints[0] - facing_angle)) < np.pi / 2


class IKController(BaseController):
    """매 호출 바뀌는 TCP 목표를 따라간다. 관절 명령은 해를 향해 MAX_JOINT_STEP 씩만 움직인다.

    IK 해가 없거나 다른 팔 형태뿐이면 마지막 명령을 유지한다(멈춤).
    """

    def __init__(self, name: str, robot_articulation: SingleArticulation) -> None:
        BaseController.__init__(self, name=name)
        self.goal_ik = GoalIK(robot_articulation)
        self._last_command = None
        self._last_warning = None

    def forward(
        self, target_end_effector_position: np.ndarray, target_end_effector_orientation: np.ndarray = None
    ) -> ArticulationAction:
        if self._last_command is None:  # reset 후 첫 호출: 현재 관절값에서 시작
            self._last_command = self.goal_ik.current_joints()

        solution, warning = self.goal_ik.solve(target_end_effector_position, target_end_effector_orientation)
        if solution is not None:
            # 해로 바로 점프하지 않고 한 번에 MAX_JOINT_STEP 씩만 이동
            delta = solution - self._last_command
            self._last_command = self._last_command + np.clip(delta, -settings.MAX_JOINT_STEP, settings.MAX_JOINT_STEP)
        if warning is not None and warning != self._last_warning:  # 같은 경고는 한 번만 출력
            print(f"{warning} 마지막 명령을 유지합니다.")
        self._last_warning = warning
        return ArticulationAction(joint_positions=self._last_command, joint_indices=self.goal_ik.joint_indices)

    def compute_end_effector_pose(self):
        return self.goal_ik.tcp_pose()

    def reset(self) -> None:
        self._last_command = None
        self._last_warning = None


class IKMotion(MotionBackend):
    """ik 이동 방식: 목표점을 TARGET_STEP 씩 직선으로 옮기고, IKController 가 그 점을 따라간다.

    목표점이 목표에 도달하면 done() 이 True. 계획 실패가 없는 방식이라 plan() 은 항상 True.
    """

    def __init__(self, ik_controller: IKController) -> None:
        self._ik_controller = ik_controller
        self.reset()

    def plan(self, goal_position: np.ndarray, goal_orientation: np.ndarray) -> bool:
        if self._target is None:  # reset 후 첫 이동은 현재 TCP 에서 시작
            self._target, _ = self._ik_controller.compute_end_effector_pose()
        self._goal = np.asarray(goal_position, dtype=float)
        self._orientation = goal_orientation
        return True

    def step(self) -> ArticulationAction:
        # 목표점을 목표 방향으로 TARGET_STEP 만큼 이동 (남은 거리가 더 짧으면 목표로)
        offset = self._goal - self._target
        distance = np.linalg.norm(offset)
        if distance <= settings.TARGET_STEP:
            self._target = self._goal.copy()
        else:
            self._target = self._target + offset / distance * settings.TARGET_STEP
        return self._ik_controller.forward(self._target, self._orientation)

    def done(self) -> bool:
        return self._goal is not None and np.allclose(self._target, self._goal)

    def reset(self) -> None:
        self._ik_controller.reset()
        self._target = None
        self._goal = None
        self._orientation = None
