"""
RRT 로 장애물을 피해 가는 경로 만들기

    isaac_python 11_rrt_path.py

직선 이동 실습과 장면도 목표도 같다. 경로를 만드는 방법만 바뀐다.

    직선 보간   매 프레임 목표를 만든다. 장애물을 모른다
    Lula RRT    시작 시 경로를 한 번 받는다. 장애물을 피한다

진행
    HOLD   준비 자세로 정지
    PLAN   RRT 로 경로를 계산하고 그린다
    HOLD   정지
    MOVE   받은 경로대로 움직인다
    DONE   도착

화면에 그리는 것
    회색 선   직선이었다면 갔을 길
    초록 선   RRT 가 찾은 길
    파랑 점   waypoint
    주황 점   실제 손끝 자취
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from pathlib import Path
import time

import numpy as np
import omni.usd
from pxr import Usd, UsdGeom, UsdPhysics

from isaacsim.core.api import World
from isaacsim.core.api.objects.cuboid import VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import (
    LulaKinematicsSolver,
    PathPlannerVisualizer,
)
from isaacsim.robot_motion.motion_generation.lula import RRT


# ══════════════════════════════════════════════════════════════
#  SWITCH
# ══════════════════════════════════════════════════════════════
SHOW_OBSTACLE     = True     # 벽을 세울지
SHOW_STRAIGHT_REF = True     # 직선이었다면 갔을 길 (회색 선)
DRAW_PLANNED_PATH = True     # RRT 경로 (초록 선)
DRAW_ACTUAL_PATH  = True     # 실제 자취 (주황 점)

HOLD_BEFORE_PLAN = 5.0       # 준비 자세 정지 시간 (초)
HOLD_AFTER_PLAN  = 5.0       # 경로를 그린 뒤 정지 시간 (초)


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")
RRT_CONFIG_PATH  = str(M0609_DIR / "descriptor/m0609_rrt_config.yaml")


# ══════════════════════════════════════════════════════════════
#  로봇 설정
# ══════════════════════════════════════════════════════════════
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"

ARM_JOINTS = ["joint_1", "joint_2", "joint_3",
              "joint_4", "joint_5", "joint_6"]

# IK 결과를 로봇이 따라가게 하는 Drive 이득
DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

ROBOT_BASE_POS  = np.array([0.0, 0.0, 0.0])
ROBOT_BASE_QUAT = np.array([1.0, 0.0, 0.0, 0.0])

# 준비 자세 — joint_3 과 joint_5 가 90 도
READY_JOINTS_DEG = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]


# ══════════════════════════════════════════════════════════════
#  그리퍼 설정
# ══════════════════════════════════════════════════════════════
GRIPPER_JOINTS = ["finger_joint", "right_inner_knuckle_joint"]

GRIPPER_OPEN_POS  = 0.0
GRIPPER_CLOSE_POS = 0.8


# ══════════════════════════════════════════════════════════════
#  TCP 오프셋
# ══════════════════════════════════════════════════════════════
# 플랜지(link_6) 에서 손가락 끝까지의 거리.
# IK 는 플랜지를 풀므로 목표를 넘길 때 이 값만큼 변환한다.
FINGER_PAD_TIP_Z = 0.19671
TCP_OFFSET = np.array([0.0, 0.0, FINGER_PAD_TIP_Z])


# ══════════════════════════════════════════════════════════════
#  목표 — 빨간 큐브
# ══════════════════════════════════════════════════════════════
# z = size/2 이면 큐브 바닥이 바닥면에 닿는다.
TARGET_PRIM_PATH = "/World/TargetCube"
TARGET_SIZE      = 0.05
TARGET_POSITION  = np.array([0.63, 0.00, TARGET_SIZE / 2.0])
TARGET_COLOR     = np.array([0.90, 0.10, 0.10])

# 손가락 끝이 큐브 윗면에 닿게 한다
TOUCH_Z = TARGET_POSITION[2] + TARGET_SIZE / 2.0


# ══════════════════════════════════════════════════════════════
#  장애물 — SHOW_OBSTACLE 이 True 일 때만 생성
# ══════════════════════════════════════════════════════════════
# 로봇과 큐브 사이를 x 로 가로막는 벽.
#   x  0.460 ~ 0.500
#   y -0.150 ~ 0.150
#   z  0.000 ~ 0.300
OBSTACLE_PRIM_PATH = "/World/Wall"
OBSTACLE_POSITION  = np.array([0.48, 0.0, 0.15])
OBSTACLE_SCALE     = np.array([0.04, 0.30, 0.30])
OBSTACLE_COLOR     = np.array([0.15, 0.15, 0.60])


# ══════════════════════════════════════════════════════════════
#  이동
# ══════════════════════════════════════════════════════════════
APPROACH_ROLL_DEG  = 180.0  # 그리퍼가 아래를 보게 한다
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


# ══════════════════════════════════════════════════════════════
#  RRT
# ══════════════════════════════════════════════════════════════
# YAML 의 max_iterations 를 덮어쓴다.
# 크면 계획하는 동안 화면이 멈춘다.
RRT_MAX_ITERATIONS = 5000

# 희소한 RRT 출력을 보간하는 간격 (관절 공간 L2 노름)
RRT_CSPACE_DIST = 0.01


# ══════════════════════════════════════════════════════════════
#  시각화
# ══════════════════════════════════════════════════════════════
# 색은 (r, g, b, a), 각 값 0.0 ~ 1.0
# 굵기와 크기는 화면 픽셀 단위
PLANNED_COLOR = (0.1, 0.9, 0.3, 1.0)    # 초록
PLANNED_WIDTH = 3.0

ACTUAL_COLOR  = (1.0, 0.55, 0.0, 1.0)   # 주황
ACTUAL_SIZE   = 6.0
ACTUAL_EVERY  = 3                       # 몇 프레임마다 점을 찍을지

GOAL_COLOR    = (1.0, 1.0, 0.2, 1.0)    # 노랑
GOAL_SIZE     = 14.0

REF_COLOR     = (0.6, 0.6, 0.6, 1.0)    # 회색 — 직선 비교용
REF_WIDTH     = 2.0

WAYPOINT_COLOR = (0.2, 0.5, 1.0, 1.0)   # 파랑
WAYPOINT_SIZE  = 7.0
WAYPOINT_EVERY = 10                     # 몇 개마다 하나씩 찍을지


def get_debug_draw():
    """디버그 드로잉 인터페이스를 얻는다. 없으면 None 을 돌려준다"""
    try:
        from isaacsim.util.debug_draw import _debug_draw
        return _debug_draw.acquire_debug_draw_interface()
    except Exception as exc:
        print(f"   debug draw   unavailable — {exc}")
        return None


class PathDrawer:
    """
    계획 경로와 실제 자취를 그린다.

    한 번 그린 지오메트리는 clear 를 부를 때까지 남는다.
    """

    def __init__(self, draw):
        self._draw = draw
        self._frame = 0

    def clear(self):
        if self._draw is None:
            return
        self._draw.clear_lines()
        self._draw.clear_points()
        self._frame = 0

    def goal_marker(self, goal):
        """목표 지점 표식"""
        if self._draw is None:
            return
        self._draw.draw_points([tuple(goal)], [GOAL_COLOR], [GOAL_SIZE])

    def reference_line(self, start, goal):
        """직선이었다면 갔을 길. 비교용 회색 선"""
        if self._draw is None or not SHOW_STRAIGHT_REF:
            return
        self._draw.draw_lines(
            [tuple(start)], [tuple(goal)], [REF_COLOR], [REF_WIDTH]
        )

    def planned_polyline(self, points):
        """RRT 경로를 선분 여러 개로 잇는다"""
        if self._draw is None or not DRAW_PLANNED_PATH or len(points) < 2:
            return

        starts = [tuple(p) for p in points[:-1]]
        ends = [tuple(p) for p in points[1:]]
        self._draw.draw_lines(
            starts, ends, [PLANNED_COLOR] * len(starts), [PLANNED_WIDTH] * len(starts)
        )

        marks = points[::WAYPOINT_EVERY]
        self._draw.draw_points(
            [tuple(p) for p in marks],
            [WAYPOINT_COLOR] * len(marks),
            [WAYPOINT_SIZE] * len(marks),
        )

    def trace(self, tcp):
        """실제 손끝 위치를 점으로 남긴다"""
        if self._draw is None or not DRAW_ACTUAL_PATH:
            return
        self._frame += 1
        if self._frame % ACTUAL_EVERY:
            return
        self._draw.draw_points([tuple(tcp)], [ACTUAL_COLOR], [ACTUAL_SIZE])


# ══════════════════════════════════════════════════════════════
#  회전 유틸
# ══════════════════════════════════════════════════════════════
def quat_mul(a, b):
    """쿼터니언 곱. 순서는 (w, x, y, z)"""
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def quat_from_axis(axis, deg):
    """회전축과 각도(도)로 쿼터니언을 만든다"""
    half = np.radians(deg) / 2.0
    a = np.array(axis, dtype=float)
    a = a / np.linalg.norm(a)
    return np.concatenate([[np.cos(half)], a * np.sin(half)])


def make_target_quat(roll_deg, pitch_deg, yaw_deg):
    """각도 세 개로 목표 자세를 만든다"""
    q = quat_mul(quat_from_axis([1, 0, 0], roll_deg),
                 quat_from_axis([0, 1, 0], pitch_deg))
    q = quat_mul(q, quat_from_axis([0, 0, 1], yaw_deg))
    return q / np.linalg.norm(q)


def quat_to_matrix(q):
    """쿼터니언을 회전행렬로 바꾼다. 각 열이 로컬 축의 월드 방향이다"""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


# ══════════════════════════════════════════════════════════════
#  TCP 변환
# ══════════════════════════════════════════════════════════════
def tcp_to_flange(tcp_pos, quat):
    """손가락 끝 목표를 플랜지 목표로 바꾼다"""
    R = quat_to_matrix(quat)
    return np.array(tcp_pos) - R @ TCP_OFFSET


def get_tcp_pose(robot):
    """현재 플랜지 pose 로부터 손가락 끝의 월드 위치를 구한다"""
    pos, quat = robot.end_effector.get_world_pose()
    return pos + quat_to_matrix(quat) @ TCP_OFFSET


# ══════════════════════════════════════════════════════════════
#  RRT 이동
# ══════════════════════════════════════════════════════════════
class RrtMove:
    """
    RRT 가 만든 경로를 하나씩 소비한다.

    직선 보간은 매 프레임 목표를 새로 만들지만,
    RRT 는 계획을 통째로 받아 두고 꺼내 쓴다.
    이 차이가 사전 계획이라는 말의 실체다.
    """

    def __init__(self, robot, planner, goal):
        self._robot = robot
        self._planner = planner
        self.goal = np.array(goal, dtype=float)
        self.reset()

    def reset(self):
        self.start = None
        self.plan = None
        self.points = []
        self.done = False
        self.failed = False

    def begin(self):
        """계획을 한 번 받는다. 이 호출은 블로킹이다"""
        self.start = get_tcp_pose(self._robot)
        print(f"   start        {vec(self.start)}")
        print(f"   goal         {vec(self.goal)}")

        self.plan, self.points = self._planner(self.goal)
        if not self.plan:
            self.failed = True
            self.done = True

        return self.start, self.goal

    def next_action(self):
        """이번 프레임에 보낼 액션을 하나 꺼낸다"""
        if self.done or not self.plan:
            return None
        action = self.plan.pop(0)
        if not self.plan:
            self.done = True
            print(f"   arrived      {vec(get_tcp_pose(self._robot))}")
        return action


class Sequence:
    """
    단계를 나눠 진행한다.

        HOLD1   준비 자세로 정지
        PLAN    RRT 로 경로를 계산하고 그린다
        HOLD2   정지
        MOVE    받은 경로대로 움직인다
        DONE    도착

    단계를 끊어야 계획과 실행이 다른 일이라는 것이 드러난다.
    """

    def __init__(self, mover, drawer):
        self._mover = mover
        self._drawer = drawer
        self.reset()

    def reset(self):
        self.phase = "HOLD1"
        self._t0 = time.time()
        self._last_count = None

    def _elapsed(self):
        return time.time() - self._t0

    def _goto(self, phase):
        self.phase = phase
        self._t0 = time.time()
        self._last_count = None

    def _countdown(self, total):
        """남은 시간을 1 초마다 한 번씩 찍는다"""
        left = int(np.ceil(total - self._elapsed()))
        if left != self._last_count and left > 0:
            print(f"   {self.phase:6s} {left} ...")
            self._last_count = left

    def update(self):
        """
        이번 프레임에 팔을 어떻게 할지 알려준다.

            "hold"  준비 자세를 유지한다
            "move"  보간 목표를 따라간다
        """
        if self.phase == "HOLD1":
            self._countdown(HOLD_BEFORE_PLAN)
            if self._elapsed() >= HOLD_BEFORE_PLAN:
                self._goto("PLAN")
            return "hold"

        if self.phase == "PLAN":
            section("PLAN")
            start, goal = self._mover.begin()

            self._drawer.reference_line(start, goal)
            self._drawer.goal_marker(goal)

            if self._mover.failed:
                print("   no plan — stopping here")
                self._goto("DONE")
                return "hold"

            self._drawer.planned_polyline(self._mover.points)
            report_path(self._mover.points)
            print()
            self._goto("HOLD2")
            return "hold"

        if self.phase == "HOLD2":
            self._countdown(HOLD_AFTER_PLAN)
            if self._elapsed() >= HOLD_AFTER_PLAN:
                section("MOVE")
                self._goto("MOVE")
            return "hold"

        if self.phase == "MOVE":
            if self._mover.done:
                self._goto("DONE")
            return "move"

        return "move"


def report_path(points):
    """계획 경로가 벽을 넘어가는지 확인한다"""
    pts = np.asarray(points)

    print(f"   waypoints    {len(pts)}")
    print(f"   tcp z range  {pts[:, 2].min():+.3f} ~ {pts[:, 2].max():+.3f}")

    if not SHOW_OBSTACLE:
        print("   wall         none")
        return

    lo = OBSTACLE_POSITION - OBSTACLE_SCALE / 2.0
    hi = OBSTACLE_POSITION + OBSTACLE_SCALE / 2.0
    print(f"   wall         x {lo[0]:+.3f}~{hi[0]:+.3f}"
          f"  z {lo[2]:+.3f}~{hi[2]:+.3f}")

    over = pts[(pts[:, 0] >= lo[0]) & (pts[:, 0] <= hi[0])]
    if len(over) == 0:
        print("   crossing     none  (옆으로 돌아감)")
        return

    lowest = float(over[:, 2].min())
    through = lo[2] <= lowest <= hi[2]
    print(f"   crossing     {len(over)} points  lowest tcp z {lowest:+.3f}")
    print(f"   verdict      {'THROUGH WALL' if through else 'clears'}")


# ══════════════════════════════════════════════════════════════
#  씬 구성 — Task
# ══════════════════════════════════════════════════════════════
def find_prim_path(root_path, name):
    """USD 계층에서 이름으로 prim 경로를 찾는다"""
    stage = omni.usd.get_context().get_stage()
    root = stage.GetPrimAtPath(root_path)
    if not root.IsValid():
        return None

    for prim in Usd.PrimRange(root):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


class M0609Task(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._robot = None
        self._target = None
        self._obstacle = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._setup_arm_drives()
        self._register_robot(scene)
        self._add_target(scene)
        self._add_obstacle(scene)
        print("   scene        ready")

    def _load_usd(self):
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()

        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()

        print("   USD          loaded")

    def _setup_arm_drives(self):
        """IK 결과를 로봇이 따라가도록 팔 관절의 Drive 를 강화한다"""
        stage = omni.usd.get_context().get_stage()
        count = 0

        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            if prim.GetName() not in ARM_JOINTS:
                continue
            for drive_type in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, drive_type)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    count += 1

        print(f"   arm drives   {count}")

    def _register_robot(self, scene):
        ee_path = find_prim_path(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found under {ROBOT_PRIM_PATH}")

        gripper = ParallelGripper(
            end_effector_prim_path=ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array([GRIPPER_OPEN_POS] * 2),
            joint_closed_positions=np.array([GRIPPER_CLOSE_POS] * 2),
            action_deltas=None,
        )

        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=ee_path,
                gripper=gripper,
            )
        )
        print(f"   EE frame     {ee_path}")

    def _add_target(self, scene):
        """목표 큐브. 물리 충돌체가 아니라 표식이다"""
        self._target = scene.add(
            VisualCuboid(
                TARGET_PRIM_PATH,
                name="target_cube",
                position=TARGET_POSITION,
                size=TARGET_SIZE,
                color=TARGET_COLOR,
            )
        )
        print(f"   target       {vec(TARGET_POSITION)}  size {TARGET_SIZE}")

    def _add_obstacle(self, scene):
        """벽. 물리 충돌체가 아니므로 팔이 부딪혀도 장면이 망가지지 않는다"""
        if not SHOW_OBSTACLE:
            print("   obstacle     none  (SHOW_OBSTACLE = False)")
            return

        self._obstacle = scene.add(
            VisualCuboid(
                OBSTACLE_PRIM_PATH,
                name="wall",
                position=OBSTACLE_POSITION,
                size=1.0,
                scale=OBSTACLE_SCALE,
                color=OBSTACLE_COLOR,
            )
        )
        lo = OBSTACLE_POSITION - OBSTACLE_SCALE / 2.0
        hi = OBSTACLE_POSITION + OBSTACLE_SCALE / 2.0
        print(f"   obstacle     x {lo[0]:+.3f}~{hi[0]:+.3f}"
              f"  y {lo[1]:+.3f}~{hi[1]:+.3f}"
              f"  z {lo[2]:+.3f}~{hi[2]:+.3f}")

    @property
    def robot(self):
        return self._robot

    @property
    def obstacle(self):
        return self._obstacle


def init_gripper(robot, world):
    """그리퍼는 Articulation 초기화 이후에 따로 초기화한다"""
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )


def set_ready_pose(robot):
    """준비 자세로 보낸다"""
    q = np.zeros(robot.num_dof)
    q[:6] = np.deg2rad(READY_JOINTS_DEG)
    robot.set_joint_positions(q)


# ══════════════════════════════════════════════════════════════
#  RRT
# ══════════════════════════════════════════════════════════════
def create_rrt(robot, obstacle):
    """
    RRT 와 PathPlannerVisualizer 를 만든다.

    robot_description_path 와 urdf_path 는 IK 와 같은 파일을 쓴다.
    RRT 전용인 것은 rrt_config_path 하나뿐이다.
    """
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )

    rrt = RRT(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
        rrt_config_path=RRT_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )

    rrt.set_robot_base_pose(
        robot_position=ROBOT_BASE_POS,
        robot_orientation=ROBOT_BASE_QUAT,
    )

    if obstacle is not None:
        # 등록만 한다. 실제 위치 조회는 update_world 에서 일어난다
        rrt.add_obstacle(obstacle)
        rrt.update_world()

    rrt.set_max_iterations(RRT_MAX_ITERATIONS)

    visualizer = PathPlannerVisualizer(robot, rrt)

    print(f"   active       {', '.join(rrt.get_active_joints())}")
    print(f"   max iter     {RRT_MAX_ITERATIONS}")
    print(f"   cspace dist  {RRT_CSPACE_DIST}")

    return rrt, visualizer, lula


def make_planner(rrt, visualizer, lula, target_quat):
    """
    경로를 계산해 (액션 리스트, TCP 점 리스트) 를 돌려주는 함수를 만든다.

    목표는 task-space 로 준다.
    set_cspace_target 을 쓰면 장애물이 충돌 검사에 반영되지 않는다.
    """

    def plan_to(target_tcp):
        flange_target = tcp_to_flange(target_tcp, target_quat)

        rrt.update_world()
        rrt.set_end_effector_target(flange_target, target_quat)

        t0 = time.time()
        plan = visualizer.compute_plan_as_articulation_actions(
            max_cspace_dist=RRT_CSPACE_DIST
        )
        elapsed = time.time() - t0

        if not plan:
            print(f"   rrt plan     FAILED  {elapsed:.2f} s"
                  f"  iter {RRT_MAX_ITERATIONS}")
            return None, []

        print(f"   rrt plan     {len(plan)} actions  {elapsed:.2f} s")
        return plan, plan_to_tcp_points(plan, lula)

    return plan_to


def plan_to_tcp_points(plan, lula):
    """
    계획의 관절각을 순기구학으로 풀어 손끝 위치로 바꾼다.

    RRT 가 주는 것은 관절각이지 좌표가 아니다.
    그림을 그리려면 이 변환이 필요하다.
    """
    points = []
    for action in plan:
        q = action.joint_positions
        if q is None:
            continue
        pos, rot = lula.compute_forward_kinematics(EE_LINK_NAME, q[:6])
        points.append(np.asarray(pos) + np.asarray(rot) @ TCP_OFFSET)
    return points


# ══════════════════════════════════════════════════════════════
#  출력
# ══════════════════════════════════════════════════════════════
def section(title):
    print(f"\n{'─' * 66}")
    print(f" {title}")
    print(f"{'─' * 66}")


def vec(v, digits=3):
    """벡터를 고정폭으로 찍는다"""
    return "[" + " ".join(f"{x:+.{digits}f}" for x in v) + "]"


def print_plan(goal, target_quat):
    """목표와 접근 자세를 확인한다"""
    R = quat_to_matrix(target_quat)

    section("PLAN")
    print(f"   goal tcp     {vec(goal)}")
    print(f"   goal flange  {vec(tcp_to_flange(goal, target_quat))}")
    print(f"   tool   +Z    {vec(R @ np.array([0, 0, 1]))}   approach direction")
    print(f"   ready pose   {READY_JOINTS_DEG} deg")


# ══════════════════════════════════════════════════════════════
#  메인
# ══════════════════════════════════════════════════════════════
LOG_INTERVAL = 60


def main():
    world = World(stage_units_in_meters=1.0)

    section("SCENE")
    task = M0609Task(name="m0609_task")
    world.add_task(task)
    world.reset()

    robot = task.robot
    robot.initialize()
    init_gripper(robot, world)
    set_ready_pose(robot)
    for _ in range(30):
        world.step(render=True)

    section("RRT")
    rrt, visualizer, lula = create_rrt(robot, task.obstacle)

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    goal_tcp = np.array([TARGET_POSITION[0], TARGET_POSITION[1], TOUCH_Z])
    planner = make_planner(rrt, visualizer, lula, target_quat)

    print_plan(goal_tcp, target_quat)

    section("DRAW")
    drawer = PathDrawer(get_debug_draw())
    print(f"   reference    {'on' if SHOW_STRAIGHT_REF else 'off'}   gray line")
    print(f"   planned      {'on' if DRAW_PLANNED_PATH else 'off'}   green line")
    print(f"   actual       {'on' if DRAW_ACTUAL_PATH else 'off'}   orange dots")

    section("RUN")
    print("   press Play in the viewport")
    print(f"   HOLD {HOLD_BEFORE_PLAN:.0f}s  ->  PLAN  ->"
          f"  HOLD {HOLD_AFTER_PLAN:.0f}s  ->  MOVE\n")

    mover = RrtMove(robot, planner, goal_tcp)
    seq = Sequence(mover, drawer)
    was_playing = False
    step = 0

    while simulation_app.is_running():
        world.step(render=True)
        time.sleep(0.005)

        is_playing = world.is_playing()

        # Play 를 누른 순간 준비 자세로 되돌리고 이전 그림을 지운다
        if is_playing and not was_playing:
            world.reset()
            robot.initialize()
            init_gripper(robot, world)
            set_ready_pose(robot)
            for _ in range(5):
                world.step(render=True)

            mover.reset()
            drawer.clear()
            seq.reset()

            step = 0
            section("HOLD")
            print(f"   ready tcp    {vec(get_tcp_pose(robot))}")

        if is_playing:
            mode = seq.update()

            if mode == "hold":
                # 관절각을 직접 넣어 중력으로 처지지 않게 한다
                set_ready_pose(robot)
                robot.apply_action(robot.gripper.forward(action="open"))
                step = 0
            else:
                action = mover.next_action()
                if action is not None:
                    robot.apply_action(action)

                robot.apply_action(robot.gripper.forward(action="open"))

                tcp = get_tcp_pose(robot)
                drawer.trace(tcp)

                if step % LOG_INTERVAL == 0:
                    left = len(mover.plan) if mover.plan else 0
                    print(f"   tcp {vec(tcp)}   left {left}")
                step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()