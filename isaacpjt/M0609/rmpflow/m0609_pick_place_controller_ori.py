from typing import List, Optional
import numpy as np
from scipy.spatial.transform import Rotation  # 하위 레이어로 이동

import isaacsim.robot.manipulators.controllers as manipulators_controllers
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot.manipulators.grippers.parallel_gripper import ParallelGripper
from isaacsim.core.utils.types import ArticulationAction

from m0609_rmpflow_controller import RMPFlowController


class PickPlaceController_ORI(manipulators_controllers.PickPlaceController):
    """M0609용 pick&place controller."""

    def __init__(
        self,
        name: str,
        gripper: ParallelGripper,
        robot_articulation: SingleArticulation,
        end_effector_initial_height: Optional[float] = None,
        events_dt: Optional[List[float]] = None,
        urdf_path: str | None = None,
        robot_description_path: str | None = None,
        rmpflow_config_path: str | None = None,
        end_effector_frame_name: str = "link_6",
    ) -> None:
        if events_dt is None:
            events_dt = [0.008, 0.005, 1.0, 0.1, 0.05, 0.05, 0.0025, 1.0, 0.008, 0.08]

        super().__init__(
            name=name,
            cspace_controller=RMPFlowController(
                name=name + "_cspace_controller",
                robot_articulation=robot_articulation,
                urdf_path=urdf_path,
                robot_description_path=robot_description_path,
                rmpflow_config_path=rmpflow_config_path,
                end_effector_frame_name=end_effector_frame_name,
            ),
            gripper=gripper,
            end_effector_initial_height=end_effector_initial_height,
            events_dt=events_dt,
        )

    def forward(
        self,
        picking_position: np.ndarray,
        placing_position: np.ndarray,
        current_joint_positions: np.ndarray,
        end_effector_offset: Optional[np.ndarray] = None,
        end_effector_orientation: Optional[np.ndarray] = None,  # 큐브의 원본 쿼터니언 접수
    ) -> ArticulationAction:
        
        # 방향 값이 들어왔을 때만 내부에서 기하학적 보정 연산 수행
        if end_effector_orientation is not None:
            w, x, y, z = end_effector_orientation
            r_cube = Rotation.from_quat([x, y, z, w])
            
            r_flip = Rotation.from_euler('y', 180, degrees=True)   # 하방 플립
            r_align = Rotation.from_euler('z', 90, degrees=True)   # 평행 그립 정렬
            
            r_target = r_cube * r_flip * r_align
            
            qx, qy, qz, qw = r_target.as_quat()
            end_effector_orientation = np.array([qw, qx, qy, qz])  # 보정된 값으로 덮어쓰기
        
        # 부모 제어기에게 최종 보정된 방향 전달
        return super().forward(
            picking_position=picking_position,
            placing_position=placing_position,
            current_joint_positions=current_joint_positions,
            end_effector_offset=end_effector_offset,
            end_effector_orientation=end_effector_orientation,
        )