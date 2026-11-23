"""경로 전체를 먼저 계획하고 재생하는 이동 방식의 공통 부분(TrajectoryMotion)과 직선 이동(LinearMotion).

TrajectoryMotion.plan() 순서
1. build_path() (하위 클래스)가 현재 관절값에서 시작하는 Lula composite path 를 만든다
2. LulaTaskSpaceTrajectoryGenerator 가 시간 최적 trajectory 를 만든다
   (관절 속도·가속도 한계에 TRAJECTORY_SPEED_SCALE 을 곱해 속도 조절)
3. ArticulationTrajectory 가 RENDERING_DT 간격으로 로봇 동작 목록을 만든다 (컨트롤러가 렌더링마다 호출되므로)
4. 첫 동작이 현재 관절값과 다르거나, 경로 중에 팔 형태가 바뀌면 계획을 거부한다
step() 은 동작을 하나씩 보내고, 다 보내면 마지막 자세를 속도 0 으로 유지한다.

composite path 를 쓰는 이유:
Isaac Sim 의 LulaTaskSpaceTrajectoryGenerator 에 직선 경로만 주면 IK 시작점을 정할 수 없어서,
M0609 에서는 현재 자세와 100 ~ 180 deg 다른 팔 형태에서 시작하는 경로가 나왔다.
composite path 에 시작 관절값(현재 관절값)을 넣으면 그 자세에서 이어지는 경로가 나온다.
"""

from abc import abstractmethod
from typing import Optional

import lula
import numpy as np
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import rot_matrix_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot_motion.motion_generation import ArticulationTrajectory, LulaTaskSpaceTrajectoryGenerator
from isaacsim.robot_motion.motion_generation.lula.utils import get_pose3

import m0609_settings as settings

from .base import MotionBackend
from .ik import GoalIK, wrap_to_pi

START_TOLERANCE = 0.05  # 첫 동작과 현재 관절값의 허용 차이 (rad)


def create_trajectory_generator() -> LulaTaskSpaceTrajectoryGenerator:
    return LulaTaskSpaceTrajectoryGenerator(str(settings.ROBOT_DESCRIPTION_PATH), str(settings.URDF_PATH))


class TrajectoryMotion(MotionBackend):
    """Lula trajectory 를 계획하고 재생하는 이동 방식의 공통 부분. build_path() 만 하위 클래스가 구현한다."""

    name = "trajectory"

    def __init__(
        self, robot_articulation: SingleArticulation, goal_ik: GoalIK, generator: LulaTaskSpaceTrajectoryGenerator
    ) -> None:
        self._robot = robot_articulation
        self._goal_ik = goal_ik
        self._generator = generator
        # 원래 한계값을 저장해 두고, 계획할 때마다 TRAJECTORY_SPEED_SCALE 을 곱해 설정 (실행 중 변경 반영)
        self._velocity_limits = np.array(generator.get_c_space_velocity_limits())
        self._acceleration_limits = np.array(generator.get_c_space_acceleration_limits())
        self.reset()

    @abstractmethod
    def build_path(
        self, start_joints: np.ndarray, goal_position: np.ndarray, goal_orientation: np.ndarray
    ) -> Optional[lula.CompositePathSpec]:
        """start_joints 에서 목표까지의 composite path. 만들 수 없으면 self._fail() 을 호출하고 None."""
        raise NotImplementedError

    def plan(self, goal_position: np.ndarray, goal_orientation: np.ndarray) -> bool:
        start_joints = self._goal_ik.current_joints()
        path = self.build_path(start_joints, np.asarray(goal_position, dtype=float), goal_orientation)
        if path is None:
            return False

        # 경로 -> 시간 최적 trajectory -> 렌더링 주기마다의 로봇 동작 목록
        scale = settings.TRAJECTORY_SPEED_SCALE
        self._generator.set_c_space_velocity_limits(self._velocity_limits * scale)
        self._generator.set_c_space_acceleration_limits(self._acceleration_limits * scale)
        trajectory = self._generator.compute_task_space_trajectory_from_path_spec(path, settings.TCP_FRAME)
        if trajectory is None:
            return self._fail("경로를 따라가는 trajectory 를 만들지 못했습니다")
        actions = ArticulationTrajectory(self._robot, trajectory, settings.RENDERING_DT).get_action_sequence()

        # 실제로 보낼 동작 목록을 검사
        joints = np.array([action.joint_positions for action in actions], dtype=float)
        if np.max(np.abs(wrap_to_pi(joints[0] - start_joints))) > START_TOLERANCE:
            return self._fail("trajectory 가 현재 관절값에서 시작하지 않습니다")
        if not all(self._goal_ik.same_configuration(q) for q in joints):
            return self._fail("trajectory 중에 팔 형태가 바뀝니다")
        self._actions = actions
        self._index = 0
        return True

    def step(self) -> ArticulationAction:
        action = self._actions[min(self._index, len(self._actions) - 1)]
        self._index += 1
        if self._index >= len(self._actions):  # 다 보낸 뒤에는 마지막 자세를 속도 0 으로 유지
            positions = action.joint_positions
            return ArticulationAction(positions, np.zeros_like(positions), joint_indices=action.joint_indices)
        return action

    def done(self) -> bool:
        return self._actions is not None and self._index >= len(self._actions)

    def reset(self) -> None:
        self._actions = None
        self._index = 0

    def _fail(self, reason: str) -> bool:
        print(f"[{self.name}] {reason}")
        self._actions = None
        return False


class LinearMotion(TrajectoryMotion):
    """linear 이동 방식: TCP 가 현재 위치에서 목표까지 직선으로 움직인다. 장애물 회피는 없다.

    로봇 베이스가 원점에 회전 없이 있다고 가정한다.
    """

    name = "linear"

    def build_path(self, start_joints, goal_position, goal_orientation):
        # 현재 TCP 자세 -> 목표 자세 직선 (task space)
        start_position, start_rotation = self._goal_ik.tcp_pose()
        line = lula.create_task_space_path_spec(get_pose3(start_position, rot_quat=rot_matrix_to_quat(start_rotation)))
        line.add_linear_path(get_pose3(goal_position, rot_quat=goal_orientation))
        # 시작 관절값을 지정해 현재 팔 형태에서 이어지게 함
        path = lula.create_composite_path_spec(start_joints.astype(np.float64))
        path.add_task_space_path_spec(line, lula.CompositePathSpec.TransitionMode.SKIP)
        return path
