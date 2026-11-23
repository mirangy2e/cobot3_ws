"""rrt 이동 방식: Lula RRT 로 장애물을 돌아가는 관절 경로를 찾고, 그 경로를 따라 trajectory 를 재생한다.
run_pick_place_rrt.py 에서만 사용한다.

build_path() 순서
1. 목표 관절값을 GoalIK 로 구해 RRT 에 관절 목표로 준다
   (TCP 목표를 주면 RRT 가 목표 자세의 팔 형태를 마음대로 고르므로)
2. 현재 관절값에서 계획용 박스(pick_place_scenes.add_obstacles)를 피하는 RRT 경로를 찾는다
3. 경로점 사이를 RRT_INTERPOLATION_STEP 간격으로 보간한다
   (부드러운 trajectory 가 충돌 검사한 경로에서 크게 벗어나지 않게. 너무 촘촘하면 점마다 느려짐:
    같은 이동이 0.01 -> 7.3 s, 0.05 -> 1.8 s)
4. seed 를 바꿔 RRT_ATTEMPTS 번 계획하고, 팔 형태를 유지하는 경로 중 관절 이동량이 가장 짧은 것을 고른다
   (RRT 는 처음 찾은 충돌 없는 경로를 돌려줄 뿐 짧은지는 따지지 않는다. 처음 경로를 그대로 쓰면
    복귀할 때 팔을 바닥 쪽으로 크게 돌리는 경로(관절 이동량 6.3 rad)가 나왔고, 같은 문제의 최단 경로는 2.0 rad)
5. 현재 관절값에서 시작하는 composite path 의 관절 공간 구간으로 넣는다 (재생은 TrajectoryMotion)

한계
- 충돌 검사는 로봇 설명 파일의 충돌 구와 계획용 박스 사이만 한다. 잡은 큐브와 D455 카메라는 포함되지 않는다
  (obstacle_margin 으로 여유를 둠)
- 관절 공간에서 계획하므로 이동 중 그리퍼 자세를 유지하지 않는다 (벽을 넘을 때 그리퍼가 기울어짐)
- 목표는 -180 ~ 180 deg 로 맞추므로, 로봇 뒤쪽에서 180 deg 를 넘는 이동은 반대로 돌아간다
"""

from typing import List

import lula
import numpy as np
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.prims import SingleArticulation
from isaacsim.robot_motion.motion_generation import PathPlannerVisualizer
from isaacsim.robot_motion.motion_generation.lula import RRT

import m0609_settings as settings

from .ik import GoalIK
from .trajectory import TrajectoryMotion, create_trajectory_generator

RRT_SEED = 123456  # RRT 설정 파일과 같은 seed: 같은 장면이면 같은 경로가 나옴


class RRTMotion(TrajectoryMotion):
    name = "rrt"
    on_fail = "hold"  # 실패했을 때 ik 로 대신 가면 장애물로 곧장 가므로 기다렸다가 다시 계획

    def __init__(self, robot_articulation: SingleArticulation, obstacles: List[VisualCuboid]) -> None:
        """obstacles: pick_place_scenes.add_obstacles() 가 돌려준 계획용 박스 목록."""
        self._rrt = RRT(
            robot_description_path=str(settings.ROBOT_DESCRIPTION_PATH),
            urdf_path=str(settings.URDF_PATH),
            rrt_config_path=str(settings.RRT_CONFIG_PATH),
            end_effector_frame_name=settings.TCP_FRAME,
        )
        base_position, base_orientation = robot_articulation.get_world_pose()
        self._rrt.set_robot_base_pose(base_position, base_orientation)
        for obstacle in obstacles:
            self._rrt.add_obstacle(obstacle, static=True)
        self._visualizer = PathPlannerVisualizer(robot_articulation, self._rrt)  # 경로 보간에 사용
        TrajectoryMotion.__init__(self, robot_articulation, GoalIK(robot_articulation), create_trajectory_generator())

    @property
    def retry_frames(self) -> int:
        return int(settings.RRT_RETRY_FRAMES)

    def build_path(self, start_joints, goal_position, goal_orientation):
        # 1. 목표 관절값 (홈 자세와 같은 팔 형태)
        goal_joints, reason = self._goal_ik.solve(goal_position, goal_orientation)
        if goal_joints is None:
            self._fail(f"목표 관절값이 없습니다: {reason}")
            return None
        self._rrt.set_cspace_target(goal_joints)
        self._rrt.update_world()  # 장애물 위치 반영
        self._rrt.set_max_iterations(int(settings.RRT_MAX_ITERATIONS))

        # 2 ~ 4. seed 를 바꿔 여러 번 계획하고, 팔 형태를 유지하는 경로 중 가장 짧은 것을 고름
        waypoints, best_length, found = None, np.inf, 0
        for attempt in range(int(settings.RRT_ATTEMPTS)):
            self._rrt.set_random_seed(RRT_SEED + attempt)
            rrt_path = self._rrt.compute_path(start_joints, np.array([]))
            if rrt_path is None or len(rrt_path) <= 1:
                continue
            candidate = self._visualizer.interpolate_path(rrt_path, settings.RRT_INTERPOLATION_STEP)
            if not all(self._goal_ik.same_configuration(q) for q in candidate):
                continue
            found += 1
            length = np.sum(np.linalg.norm(np.diff(candidate, axis=0), axis=1))  # 관절 공간 이동량 (rad)
            if length < best_length:
                waypoints, best_length = candidate, length
        if waypoints is None:
            self._fail(f"{settings.RRT_ATTEMPTS}번 시도했지만 팔 형태를 유지하는 충돌 없는 경로가 없습니다")
            return None
        print(f"[rrt] 경로 {found}개 중 가장 짧은 경로 선택: 관절 이동량 {best_length:.2f} rad")

        # 5. 관절 경로를 composite path 에 넣음 (현재 관절값에서 시작)
        joint_path = lula.create_c_space_path_spec(waypoints[0].astype(np.float64))
        for joints in waypoints[1:]:
            joint_path.add_c_space_waypoint(joints.astype(np.float64))
        path = lula.create_composite_path_spec(start_joints.astype(np.float64))
        path.add_c_space_path_spec(joint_path, lula.CompositePathSpec.TransitionMode.SKIP)
        return path
