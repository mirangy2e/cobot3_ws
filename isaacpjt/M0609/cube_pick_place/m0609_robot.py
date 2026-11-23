"""M0609 + RG2 그리퍼 + D455 카메라 로봇을 장면에 만들고, 홈 자세로 보내는 함수 모음.

- create_robot()      : 로봇 USD 를 불러와 Isaac Sim 매니퓰레이터(SingleManipulator)로 만든다
- go_home()           : 팔을 홈 자세로, 그리퍼를 연 상태로 즉시 옮긴다 (reset 할 때마다 호출)
- home_tcp_position() : 홈 자세일 때 TCP 위치 (follow target 의 타겟 시작 위치로 사용)
"""

import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

import m0609_settings as settings


class RG2Gripper(ParallelGripper):
    """열림/닫힘 각도를 명령할 때마다 settings 에서 다시 읽는 그리퍼.

    기본 ParallelGripper 는 생성할 때의 값을 계속 쓰므로, 실행 중에 gripper_open / gripper_closed 를
    바꿔도 반영되도록 forward() 에서 값을 갱신한다.
    """

    def forward(self, action: str) -> ArticulationAction:
        self._joint_opened_positions = np.array([settings.GRIPPER_OPEN])
        self._joint_closed_positions = np.array([settings.GRIPPER_CLOSED])
        return ParallelGripper.forward(self, action)


def create_robot(name: str = "m0609_robot") -> SingleManipulator:
    """로봇 USD 를 장면에 추가하고 그리퍼가 달린 매니퓰레이터 객체를 돌려준다."""
    # 로봇 USD 의 기본 prim(로봇)을 ROBOT_PRIM_PATH 에 참조로 넣는다
    add_reference_to_stage(usd_path=str(settings.ROBOT_USD), prim_path=settings.ROBOT_PRIM_PATH)

    # 구동 관절은 finger_joint 하나, 나머지 손가락은 USD 의 mimic 관절이 따라 움직임
    gripper = RG2Gripper(
        end_effector_prim_path=settings.END_EFFECTOR_PRIM_PATH,
        joint_prim_names=[settings.GRIPPER_JOINT],
        joint_opened_positions=np.array([settings.GRIPPER_OPEN]),
        joint_closed_positions=np.array([settings.GRIPPER_CLOSED]),
        use_mimic_joints=True,
    )
    return SingleManipulator(
        prim_path=settings.ROBOT_PRIM_PATH,
        name=name,
        end_effector_prim_path=settings.END_EFFECTOR_PRIM_PATH,
        gripper=gripper,
    )


def go_home(robot: SingleManipulator) -> None:
    """팔을 홈 자세로, 그리퍼를 연 상태로 순간이동시킨다.

    모든 관절이 0 인 자세는 팔이 위로 곧게 펴진 특이 자세라서, reset 할 때마다 홈 자세로 보낸다.
    """
    positions = np.zeros(robot.num_dof)
    for joint_name, angle in zip(settings.ARM_JOINTS, np.radians(settings.HOME_JOINTS_DEG)):
        positions[robot.get_dof_index(joint_name)] = angle
    positions[robot.get_dof_index(settings.GRIPPER_JOINT)] = settings.GRIPPER_OPEN
    robot.set_joints_default_state(positions=positions)
    robot.set_joint_positions(positions)
    robot.set_joint_velocities(np.zeros(robot.num_dof))


def home_tcp_position() -> np.ndarray:
    """홈 자세일 때의 TCP 위치를 정기구학으로 계산한다 (로봇 베이스가 원점에 있다고 가정)."""
    kinematics = LulaKinematicsSolver(
        robot_description_path=str(settings.ROBOT_DESCRIPTION_PATH), urdf_path=str(settings.URDF_PATH)
    )
    position, _ = kinematics.compute_forward_kinematics(settings.TCP_FRAME, np.radians(settings.HOME_JOINTS_DEG))
    return np.array(position)
