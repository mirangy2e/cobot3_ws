"""실행 파일들이 쓰는 장면(task) 정의와 장애물 생성 함수.

- PickPlaceTask   : 바닥 + 큐브 + 로봇. Isaac Sim 의 PickPlace task 에 M0609 를 넣은 것 (run_pick_place*.py)
- FollowTargetTask: 바닥 + 빨간 타겟 + 큐브 + 로봇. 타겟을 끌어 로봇을 움직이는 디버깅용 (run_follow_target.py)
- add_obstacles() : m0609_settings.yaml 의 obstacles 로 벽을 만들고 RRT 용 박스를 돌려줌 (run_pick_place_rrt.py)
- use_default_light(): 뷰포트 조명을 Lighting 메뉴의 "Default" 로 켬 (모든 실행 파일)
두 task 모두 reset 할 때 로봇을 홈 자세로 보낸다.
"""

from typing import List, Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
import omni.kit.commands
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, VisualCuboid
from isaacsim.core.api.scenes.scene import Scene
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.robot.manipulators.manipulators import SingleManipulator

import m0609_settings as settings
from m0609_robot import create_robot, go_home


class PickPlaceTask(tasks.PickPlace):
    """큐브를 cube_initial_position 에 만들고, 놓을 위치를 place_position 으로 하는 pick and place 장면."""

    def __init__(self, name: str = "m0609_pick_place") -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=settings.CUBE_INITIAL_POSITION,
            target_position=settings.PLACE_POSITION,
            cube_size=settings.CUBE_SIZE,
        )

    def set_robot(self) -> SingleManipulator:
        return create_robot()

    def post_reset(self) -> None:
        go_home(self._robot)
        tasks.PickPlace.post_reset(self)  # 그리퍼를 연 상태로 맞춤


class FollowTargetTask(tasks.FollowTarget):
    """빨간 타겟 큐브를 따라가는 장면. 직접 집어볼 수 있도록 pick and place 와 같은 큐브를 추가했다."""

    def __init__(self, name: str = "m0609_follow_target", target_position: Optional[np.ndarray] = None) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_position=target_position,
            target_orientation=euler_angles_to_quat(np.radians(settings.GRASP_EULER_DEG)),  # 그리퍼가 아래를 향함
        )
        self._cube = None

    def set_up_scene(self, scene: Scene) -> None:
        tasks.FollowTarget.set_up_scene(self, scene)  # 바닥, 타겟, 로봇
        self._cube = scene.add(
            DynamicCuboid(
                name="cube",
                prim_path="/World/Cube",
                position=settings.CUBE_INITIAL_POSITION,
                scale=settings.CUBE_SIZE,
                size=1.0,
                color=np.array([0, 0, 1]),
            )
        )
        self._task_objects[self._cube.name] = self._cube

    def set_robot(self) -> SingleManipulator:
        return create_robot()

    def get_params(self) -> dict:
        params = tasks.FollowTarget.get_params(self)
        params["cube_name"] = {"value": self._cube.name, "modifiable": False}
        return params

    def get_observations(self) -> dict:
        # 부모 클래스는 타겟 높이를 0.035 m 아래로 내리지 못하게 막는데, 큐브를 잡는 높이와 가까워서 직접 구현
        target_position, target_orientation = self._target.get_local_pose()
        cube_position, _ = self._cube.get_local_pose()
        return {
            self._robot.name: {"joint_positions": np.array(self._robot.get_joints_state().positions)},
            self._target.name: {"position": np.array(target_position), "orientation": np.array(target_orientation)},
            self._cube.name: {"position": np.array(cube_position)},
        }

    def post_reset(self) -> None:
        go_home(self._robot)
        tasks.FollowTarget.post_reset(self)


def add_obstacles(scene: Scene) -> List[VisualCuboid]:
    """m0609_settings.yaml 의 obstacles 로 장애물을 만들고, RRT 에 넘길 박스 목록을 돌려준다.

    장애물마다 두 개를 만든다.
    - 실제 벽(FixedCuboid, 회색): 로봇과 큐브가 물리적으로 부딪힘
    - 계획용 박스(VisualCuboid, 보이지 않음, 충돌 없음): obstacle_margin 만큼 키워 RRT 에 넘김
    world.reset() 전에 호출해야 한다.
    """
    planner_boxes = []
    for obstacle in settings.OBSTACLES:
        name = obstacle["name"]
        position = np.array(obstacle["position"], dtype=float)
        size = np.array(obstacle["size"], dtype=float)
        scene.add(
            FixedCuboid(
                prim_path=f"/World/Obstacles/{name}",
                name=name,
                position=position,
                scale=size,
                size=1.0,
                color=np.array([0.5, 0.5, 0.5]),
            )
        )
        planner_boxes.append(
            scene.add(
                VisualCuboid(
                    prim_path=f"/World/PlannerObstacles/{name}",
                    name=f"{name}_planner",
                    position=position,
                    scale=size + 2 * settings.OBSTACLE_MARGIN,
                    size=1.0,
                    visible=False,
                )
            )
        )
    return planner_boxes


def use_default_light() -> None:
    """뷰포트 Lighting 메뉴의 "Default" 조명을 켠다 (GUI 에서 Lighting > Default 를 고른 것과 같음).

    로봇 USD 에서 로봇 prim 만 가져오므로 USD 안의 조명은 장면에 들어오지 않는다. 그래서 실행할 때 켜 준다.
    조명 기능(omni.kit.viewport.menubar.lighting)이 없는 실행 환경이면 경고만 출력하고 넘어간다.
    """
    try:
        omni.kit.commands.execute("SetLightingMenuModeCommand", lighting_mode="Default")
    except Exception as error:
        print(f"[pick_place_scenes] Default 조명을 켜지 못했습니다: {error}")
