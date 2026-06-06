# ~/cobot3_ws/isaacpjt/franka/controllers/lula_controller.py
import numpy as np
from isaacsim.robot_motion.motion_generation import RmpFlow
from isaacsim.robot_motion.motion_generation import ArticulationMotionPolicy
from isaacsim.core.api.robots import Robot


class LulaController:
    def __init__(self, world):
        self.world  = world
        self.robot  = Robot("/World/franka")
        self._init_rmpflow()

    def _init_rmpflow(self):
        self.rmpflow = RmpFlow(
            robot_description_path=None,   # Franka 내장 설정 자동 로드
            robot_name="Franka",
            end_effector_frame_name="right_gripper"
        )
        self.articulation_rmpflow = ArticulationMotionPolicy(
            self.robot, self.rmpflow
        )

    def move_to(self, target_position, target_orientation=None):
        """
        target_position: np.array([x, y, z])  world 기준
        target_orientation: np.array([qx, qy, qz, qw]) (선택)
        """
        self.rmpflow.set_end_effector_target(
            target_position,
            target_orientation
        )

    def update(self):
        """매 프레임 호출 — RmpFlow 계산 및 관절 명령 적용"""
        self.rmpflow.update_world()
        action = self.articulation_rmpflow.get_next_articulation_action()
        self.robot.apply_action(action)

    def is_reached(self, target_position, threshold=0.02):
        """목표 위치 도달 여부 확인 (threshold: 미터)"""
        ee_pos, _ = self.robot.end_effector.get_world_pose()
        return np.linalg.norm(ee_pos - target_position) < threshold