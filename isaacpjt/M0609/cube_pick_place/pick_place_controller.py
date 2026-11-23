"""pick and place 단계를 진행하는 상태 머신. 시간이 아니라 "도착했는지"를 보고 다음 단계로 넘어간다.

단계 (Isaac Sim 기본 PickPlaceController 와 같은 10단계, forward() 인자도 같음)
    0 ABOVE_CUBE  큐브 위로 이동          5 CARRY   놓을 위치 위로 이동
    1 DESCEND     큐브까지 하강            6 LOWER   놓을 위치로 하강
    2 SETTLE      잠시 대기                7 OPEN    그리퍼 열기
    3 CLOSE       그리퍼 닫기              8 UP      상승
    4 LIFT        큐브를 들고 상승         9 RETURN  시작 위치로 복귀

- 이동 단계: m0609_settings.yaml 의 motion_modes(또는 rrt_motion_modes)에서 고른 이동 방식이 단계 시작 때 한 번
  계획하고 매 호출 동작을 보낸다. 명령을 다 보냈고 TCP 가 POSITION_TOLERANCE 안에 들어오면 다음 단계.
  명령을 다 보낸 뒤 STUCK_FRAMES 가 지나도 도착하지 못하면(바닥이나 물체에 걸림) 경고 후 다음 단계.
- 그리퍼 단계: 손가락이 멈추면(큐브에 닿았거나 끝까지 감) 다음 단계.
- 계획 실패: 이동 방식의 on_fail 에 따라 ik 로 대신 이동하거나, 제자리에서 기다렸다가 다시 계획.

Isaac Sim 기본 PickPlaceController 는 events_dt(시간)로 단계를 넘겨서 로봇이 도착하기 전에 다음 단계로
넘어가는 문제가 있어 직접 구현했다.
"""

from typing import Dict, Optional

import numpy as np
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper

import m0609_settings as settings
from m0609_motion.base import MotionBackend
from m0609_motion.ik import IKController, IKMotion
from m0609_motion.trajectory import LinearMotion, create_trajectory_generator

PHASES = ["ABOVE_CUBE", "DESCEND", "SETTLE", "CLOSE", "LIFT", "CARRY", "LOWER", "OPEN", "UP", "RETURN"]
SETTLE, CLOSE, OPEN = 2, 3, 7  # 이동이 아닌 단계


class PickPlaceController(BaseController):
    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        motion_modes: str = "motion_modes",
        extra_motions: Optional[Dict[str, MotionBackend]] = None,
    ) -> None:
        """
        motion_modes : 단계별 이동 방식을 읽을 m0609_settings.yaml 의 키 ("motion_modes" 또는 "rrt_motion_modes")
        extra_motions: ik, linear 외에 추가할 이동 방식 (예: {"rrt": RRTMotion(...)})
        """
        BaseController.__init__(self, name=name)
        self._gripper = gripper
        self._ik_controller = IKController(name=name + "_ik", robot_articulation=robot_articulation)
        self._motions = {
            "ik": IKMotion(self._ik_controller),
            "linear": LinearMotion(robot_articulation, self._ik_controller.goal_ik, create_trajectory_generator()),
        }
        self._motions.update(extra_motions or {})
        self._motion_modes_key = motion_modes.upper()
        self.reset()

    def forward(
        self,
        picking_position: np.ndarray,
        placing_position: np.ndarray,
        current_joint_positions: np.ndarray,
        end_effector_offset: Optional[np.ndarray] = None,
        end_effector_orientation: Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        """매 프레임 호출. 현재 단계를 진행하고 로봇에 보낼 동작을 돌려준다."""
        if self.is_done():
            return ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])
        if end_effector_offset is None:
            end_effector_offset = np.zeros(3)
        if end_effector_orientation is None:
            end_effector_orientation = euler_angles_to_quat(np.radians(settings.GRASP_EULER_DEG))

        tcp_position, _ = self._ik_controller.compute_end_effector_pose()
        if self._start is None:  # 마지막 RETURN 단계의 목표
            self._start = tcp_position.copy()
        if self._event <= 1:  # 잡기 전까지는 큐브 위치를 계속 갱신
            self._pick = np.asarray(picking_position, dtype=float) + end_effector_offset
        place = np.asarray(placing_position, dtype=float) + end_effector_offset
        self._frames += 1

        if self._event == SETTLE:
            action = ArticulationAction(joint_positions=[None] * current_joint_positions.shape[0])
            finished = self._frames >= settings.SETTLE_FRAMES
        elif self._event in (CLOSE, OPEN):
            action = self._gripper.forward(action="close" if self._event == CLOSE else "open")
            finished = self._gripper_stopped()
        else:
            goal = self._goal(self._event, place)
            # 단계 시작 때 한 번 계획. 실패해서 기다리는 중이면 retry 시점까지 제자리 유지
            if self._motion is None and self._frames >= self._retry_frame:
                self._motion = self._plan(goal, end_effector_orientation)
                if self._motion is None:
                    self._retry_frame = self._frames + self._motions[self._mode].retry_frames
            if self._motion is None:
                return ArticulationAction()
            action = self._motion.step()

            # 도착 판정: 명령을 다 보냈고 TCP 가 목표 근처
            distance = np.linalg.norm(tcp_position - goal)
            if self._motion.done():
                self._frames_since_done += 1
            finished = self._frames_since_done > 0 and distance < settings.POSITION_TOLERANCE
            if not finished and self._frames_since_done >= settings.STUCK_FRAMES:
                print(f"[{self._name}] {PHASES[self._event]}: 목표까지 {distance:.3f} m 남은 채 멈춰서 다음 단계로 넘어갑니다")
                finished = True

        if finished:
            self._next_phase()
        return action

    def _next_phase(self) -> None:
        self._event += 1
        self._frames = 0
        self._frames_since_done = 0
        self._finger_history = []
        self._motion = None
        self._retry_frame = 0
        print(f"[{self._name}] {PHASES[self._event] if self._event < len(PHASES) else 'DONE'}")

    def _goal(self, event: int, place: np.ndarray) -> np.ndarray:
        """이동 단계의 TCP 목표 위치."""
        height = settings.END_EFFECTOR_INITIAL_HEIGHT
        above_pick = np.array([self._pick[0], self._pick[1], height])
        above_place = np.array([place[0], place[1], height])
        goals = {0: above_pick, 1: self._pick, 4: above_pick, 5: above_place, 6: place, 8: above_place, 9: self._start}
        return goals[event]

    def _plan(self, goal: np.ndarray, orientation: np.ndarray) -> Optional[MotionBackend]:
        """이 단계의 이동 방식을 골라 계획한다. 실패 시 on_fail 에 따라 ik 로 대신하거나 None(대기)."""
        phase = PHASES[self._event].lower()
        mode = getattr(settings, self._motion_modes_key).get(phase, "ik")
        if mode not in self._motions:
            print(f"[{self._name}] {self._motion_modes_key.lower()}.{phase}: '{mode}' 는 없는 이동 방식이라 ik 를 씁니다 ({list(self._motions)})")
            mode = "ik"
        self._mode = mode
        motion = self._use(self._motions[mode])
        if motion.plan(goal, orientation):
            return motion
        if motion.on_fail == "hold":
            print(f"[{self._name}] {PHASES[self._event]}: {mode} 계획 실패, 기다렸다가 다시 계획합니다")
            return None
        print(f"[{self._name}] {PHASES[self._event]}: {mode} 계획 실패, ik 로 대신 이동합니다")
        motion = self._use(self._motions["ik"])
        motion.plan(goal, orientation)
        return motion

    def _use(self, motion: MotionBackend) -> MotionBackend:
        """다른 이동 방식이 팔을 움직였으면 reset 해서 현재 상태에서 다시 시작하게 한다."""
        if motion is not self._last_motion:
            motion.reset()
        self._last_motion = motion
        return motion

    def _gripper_stopped(self) -> bool:
        """최근 GRIPPER_STILL_FRAMES 동안 손가락 각도 변화가 GRIPPER_STILL 보다 작으면 멈춘 것으로 판정."""
        self._finger_history.append(self._gripper.get_joint_positions()[0])
        window = self._finger_history[-settings.GRIPPER_STILL_FRAMES :]
        if self._frames >= settings.GRIPPER_MAX_FRAMES:
            return True
        return len(window) == settings.GRIPPER_STILL_FRAMES and max(window) - min(window) < settings.GRIPPER_STILL

    def get_current_event(self) -> int:
        return self._event

    def is_done(self) -> bool:
        return self._event >= len(PHASES)

    def reset(self) -> None:
        BaseController.reset(self)
        for motion in self._motions.values():
            motion.reset()
        self._last_motion = None
        self._motion = None
        self._mode = "ik"
        self._event = 0
        self._frames = 0
        self._frames_since_done = 0
        self._finger_history = []
        self._retry_frame = 0
        self._start = None
        self._pick = None
