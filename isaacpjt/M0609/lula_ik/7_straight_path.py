"""
직선 이동과 경로 시각화

    isaac_python 7_straight_path.py

직선 보간으로 만든 목표를 IK 로 풀어 큐브까지 이동한다.
계획 경로와 실제 자취를 함께 그려 두 경로를 비교한다.

진행
    HOLD   준비 자세로 정지
    DRAW   계획 경로를 그린다
    HOLD   정지
    MOVE   경로대로 움직인다
    DONE   도착

SHOW_OBSTACLE 을 True 로 바꾸면 벽이 생긴다.
직선 보간에는 주변 물체를 조회하는 단계가 없으므로 벽을 그대로 통과한다.
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
    ArticulationKinematicsSolver,
)


# ══════════════════════════════════════════════════════════════
#  SWITCH
# ══════════════════════════════════════════════════════════════
SHOW_OBSTACLE     = True    # 벽을 세울지
DRAW_PLANNED_PATH = True     # 계획 경로 (초록 선)
DRAW_ACTUAL_PATH  = True     # 실제 자취 (주황 점)

HOLD_BEFORE_DRAW = 5.0       # 준비 자세 정지 시간 (초)
HOLD_AFTER_DRAW  = 5.0       # 경로를 그린 뒤 정지 시간 (초)


# ══════════════════════════════════════════════════════════════
#  경로
# ══════════════════════════════════════════════════════════════
THIS_DIR  = Path(__file__).resolve().parent
M0609_DIR = THIS_DIR.parent

USD_PATH         = str(M0609_DIR / "Collected_m0609_camera_cube/m0609_camera_cube.usd")
URDF_PATH        = str(M0609_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
DESCRIPTION_PATH = str(M0609_DIR / "descriptor/m0609_description.yaml")


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
TCP_SPEED = 0.004           # 한 스텝에 움직이는 거리 (m)
MIN_STEPS = 60
MAX_STEPS = 600

APPROACH_ROLL_DEG  = 180.0  # 그리퍼가 아래를 보게 한다
APPROACH_PITCH_DEG = 0.0
GRIPPER_YAW_DEG    = 0.0


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

    def planned_line(self, start, goal):
        """계획 경로. 이동 시작 시 한 번만 그린다"""
        if self._draw is None or not DRAW_PLANNED_PATH:
            return
        self._draw.draw_lines(
            [tuple(start)], [tuple(goal)], [PLANNED_COLOR], [PLANNED_WIDTH]
        )

    def goal_marker(self, goal):
        """목표 지점 표식"""
        if self._draw is None:
            return
        self._draw.draw_points([tuple(goal)], [GOAL_COLOR], [GOAL_SIZE])

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
#  직선 보간
# ══════════════════════════════════════════════════════════════
def steps_for(start, goal):
    """구간 길이를 속도로 나눠 스텝 수를 정한다"""
    dist = float(np.linalg.norm(goal - start))
    return int(np.clip(dist / TCP_SPEED, MIN_STEPS, MAX_STEPS)), dist


def lerp(start, goal, alpha):
    """시작점에서 목표점까지 선형 보간"""
    return start + alpha * (goal - start)


class StraightMove:
    """
    시작점에서 목표점까지 직선으로 간다.

    이 클래스가 경로 계획기 자리를 대신한다.
    주변 물체를 조회하는 단계는 없다.
    """

    def __init__(self, robot, goal):
        self._robot = robot
        self.goal = np.array(goal, dtype=float)
        self.reset()

    def reset(self):
        self.start = None
        self.step = 0
        self.n_steps = MIN_STEPS
        self.done = False

    def begin(self):
        """현재 위치를 시작점으로 잡는다"""
        self.start = get_tcp_pose(self._robot)
        self.n_steps, dist = steps_for(self.start, self.goal)
        print(f"   start        {vec(self.start)}")
        print(f"   goal         {vec(self.goal)}")
        print(f"   distance     {dist:.4f} m   {self.n_steps} steps")
        return self.start, self.goal

    def current_target(self):
        """이번 스텝의 TCP 목표"""
        if self.start is None:
            return self.goal
        alpha = min(1.0, self.step / float(self.n_steps))
        return lerp(self.start, self.goal, alpha)

    def advance(self):
        if self.done:
            return
        self.step += 1
        if self.step >= self.n_steps:
            self.done = True
            print(f"   arrived      {vec(get_tcp_pose(self._robot))}")


class Sequence:
    """
    단계를 나눠 진행한다.

        HOLD1   준비 자세로 정지
        DRAW    계획 경로를 그린다
        HOLD2   정지
        MOVE    경로대로 움직인다
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
            self._countdown(HOLD_BEFORE_DRAW)
            if self._elapsed() >= HOLD_BEFORE_DRAW:
                self._goto("DRAW")
            return "hold"

        if self.phase == "DRAW":
            section("DRAW")
            start, goal = self._mover.begin()
            self._drawer.planned_line(start, goal)
            self._drawer.goal_marker(goal)
            report_path(start, goal)
            print()
            self._goto("HOLD2")
            return "hold"

        if self.phase == "HOLD2":
            self._countdown(HOLD_AFTER_DRAW)
            if self._elapsed() >= HOLD_AFTER_DRAW:
                section("MOVE")
                self._goto("MOVE")
            return "hold"

        if self.phase == "MOVE":
            if self._mover.done:
                self._goto("DONE")
            return "move"

        return "move"


def report_path(start, goal):
    """계획한 직선이 벽을 지나는지 확인한다"""
    if not SHOW_OBSTACLE:
        print("   wall         none")
        return

    lo = OBSTACLE_POSITION - OBSTACLE_SCALE / 2.0
    hi = OBSTACLE_POSITION + OBSTACLE_SCALE / 2.0

    print(f"   wall         x {lo[0]:+.3f}~{hi[0]:+.3f}"
          f"  z {lo[2]:+.3f}~{hi[2]:+.3f}")

    denom = goal[0] - start[0]
    if abs(denom) < 1e-9:
        print("   crossing     none")
        return

    for xf in (lo[0], hi[0]):
        t = (xf - start[0]) / denom
        if not (0.0 <= t <= 1.0):
            continue
        p = start + t * (goal - start)
        through = lo[2] <= p[2] <= hi[2]
        print(f"   crossing     x {xf:+.3f}  tcp z {p[2]:+.3f}"
              f"   {'THROUGH WALL' if through else 'clears'}")


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
#  IK 솔버
# ══════════════════════════════════════════════════════════════
def create_ik_solver(robot):
    """Lula 계산기를 만들고 로봇과 연결한다"""
    lula = LulaKinematicsSolver(
        robot_description_path=DESCRIPTION_PATH,
        urdf_path=URDF_PATH,
    )

    lula.set_robot_base_pose(
        robot_position=ROBOT_BASE_POS,
        robot_orientation=ROBOT_BASE_QUAT,
    )

    print(f"   controlled   {', '.join(lula.get_joint_names())}")

    return ArticulationKinematicsSolver(
        robot_articulation=robot,
        kinematics_solver=lula,
        end_effector_frame_name=EE_LINK_NAME,
    )


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

    section("SOLVER")
    ik_solver = create_ik_solver(robot)

    target_quat = make_target_quat(
        APPROACH_ROLL_DEG, APPROACH_PITCH_DEG, GRIPPER_YAW_DEG
    )
    goal_tcp = np.array([TARGET_POSITION[0], TARGET_POSITION[1], TOUCH_Z])
    print_plan(goal_tcp, target_quat)

    section("DRAW")
    drawer = PathDrawer(get_debug_draw())
    print(f"   planned      {'on' if DRAW_PLANNED_PATH else 'off'}   green line")
    print(f"   actual       {'on' if DRAW_ACTUAL_PATH else 'off'}   orange dots")

    section("RUN")
    print("   press Play in the viewport")
    print(f"   HOLD {HOLD_BEFORE_DRAW:.0f}s  ->  DRAW  ->"
          f"  HOLD {HOLD_AFTER_DRAW:.0f}s  ->  MOVE\n")

    mover = StraightMove(robot, goal_tcp)
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
                target_tcp = mover.current_target()
                flange_target = tcp_to_flange(target_tcp, target_quat)

                action, solved = ik_solver.compute_inverse_kinematics(
                    target_position=flange_target,
                    target_orientation=target_quat,
                )
                if solved:
                    robot.apply_action(action)

                robot.apply_action(robot.gripper.forward(action="open"))

                tcp = get_tcp_pose(robot)
                drawer.trace(tcp)
                mover.advance()

                if step % LOG_INTERVAL == 0:
                    err = float(np.linalg.norm(tcp - target_tcp))
                    tag = "ok" if solved else "IK FAILED"
                    print(f"   tcp {vec(tcp)}   cmd {vec(target_tcp)}"
                          f"   err {err:.4f}   {tag}")
                step += 1

        was_playing = is_playing

    simulation_app.close()


if __name__ == "__main__":
    main()