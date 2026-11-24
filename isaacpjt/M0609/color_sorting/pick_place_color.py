"""[색상 분류 미션] 랜덤 위치의 파랑/초록 큐브를 집어 같은 색 마커 위에 놓기 (PC A, Isaac Sim)

실행: isaac_python pick_place_color.py
      창에서 Play(▶) 를 누른다. PC B 에서 색상 감지 노드(m0609_color_detector)가 /color_id 를 발행해야 한다
      Stop -> Play 하면 처음부터 다시 시작한다

흐름 (한 라운드)
      두 큐브는 대기 위치에 있다. 그중 하나를 골라 pick 영역의 랜덤 위치로 옮긴다
      -> 큐브 위에서 손목 카메라로 내려다본다 (씬의 Camera Graph 가 /rgb 발행)
      -> PC B 가 보낸 /color_id (1: 파랑, 2: 초록) 를 받는다
      -> IK 로 집고, 같은 색 마커 위에 놓는다 -> 놓은 큐브를 대기 위치로 되돌리고 다음 라운드
"""

from pathlib import Path

# ============================================================
# 1. 앱 시작 (Isaac 모듈보다 먼저)
# ============================================================
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ============================================================
# 2. Isaac 모듈 import (앱을 켠 뒤에)
# ============================================================
import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.controllers import BaseController
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.extensions import enable_extension
from isaacsim.core.utils.prims import get_prim_at_path
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver

# ROS2 Bridge 확장을 켠 뒤에 rclpy 를 import 한다 (Isaac Sim 에 들어 있는 rclpy)
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()
import rclpy
from std_msgs.msg import Int32

# 파일 경로
M0609_DIR = Path(__file__).resolve().parents[1]                            # isaacpjt/M0609 (이 파일이 있는 color_sorting 의 상위 폴더)
SCENE_USD = f"{M0609_DIR}/scenes/pick_place_cube_ros.usd"                  # GUI ROS 강의에서 만든 씬 (Clock Graph + Camera Graph)
URDF_PATH = f"{M0609_DIR}/robot/m0609_rg2.urdf"                            # IK 용: 링크 길이, 관절 축, 관절 한계
DESCRIPTION_PATH = f"{M0609_DIR}/descriptor/m0609_robot_description.yaml"  # IK 용: 움직일 관절 (팔 6개)
ROBOT_PRIM = "/World/m0609_rg2_d455"                                       # 씬에 들어 있는 로봇

# 로봇 관절
ARM_JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
HOME_DEG = [0, 0, 90, 0, 90, 0]

# IK 목표 설정 (L8 과 같음)
TCP_FRAME = "gripper_center"  # IK 로 위치를 맞출 기준점: 그리퍼 손가락 사이 (URDF 의 프레임 이름)
DOWN_ORIENTATION = euler_angles_to_quat(np.radians([0, 180, 0]))  # TCP 자세: 그리퍼가 아래를 향함 (쿼터니언)

# 큐브
CUBE_SIZE = 0.0515                      # 큐브 한 변 (m)
CUBE_Z = CUBE_SIZE / 2                  # 바닥에 놓인 큐브 중심 높이 (m)
PICK_AREA_X = (0.40, 0.50)              # 큐브가 나타날 pick 영역 x 범위 (m)
PICK_AREA_Y = (0.10, 0.25)              # 큐브가 나타날 pick 영역 y 범위 (m)
CUBES = {                               # 색 번호: (이름, 색 [R, G, B], 대기 위치 [x, y]). 대기 위치는 카메라에 안 보이는 곳
    1: ("blue_cube", [0.0, 0.0, 1.0], [-0.30, 0.50]),
    2: ("green_cube", [0.0, 0.8, 0.0], [-0.30, 0.60]),
}

# 놓을 위치 (색 번호마다 마커 하나)
MARKERS = {                             # 색 번호: (이름, 색, 위치 [x, y])
    1: ("blue_marker", [0.0, 0.0, 1.0], [0.35, -0.30]),
    2: ("green_marker", [0.0, 0.8, 0.0], [0.50, -0.30]),
}
COLOR_NAMES = {1: "파랑", 2: "초록"}

# 단계별 높이 (TCP, m)
OBSERVE_HEIGHT = 0.30                   # 카메라로 큐브를 내려다보는 높이
ABOVE_HEIGHT = 0.20                     # 큐브 위 / 마커 위
GRASP_HEIGHT = CUBE_Z + 0.015           # 집을 때와 놓을 때: 큐브 중심보다 1.5 cm 위 (L8 과 같음)

# 단계 진행
TOLERANCE_DEG = 1.0     # 팔 관절이 모두 목표와 이만큼 안으로 들어오면 "도착"
GRIPPER_FRAMES = 60     # 그리퍼 단계는 이만큼 기다린 뒤 다음 단계로 (60 프레임 = 1 초)
COLOR_FRAMES = 30       # 같은 색 결과가 이만큼 연속으로 오면 색을 확정한다
COLOR_TIMEOUT = 600     # 이만큼 기다려도 색 결과가 없으면 안내를 출력한다 (10 초)
COLOR_TOPIC = "color_id"

rng = np.random.default_rng()  # 큐브 색과 위치를 고르는 난수 생성기


# ------------------------------------------------------------
# (가) IK: TCP 목표 위치 -> 팔 관절 6개 각도 (L8 과 같음)
# ------------------------------------------------------------
ik_solver = LulaKinematicsSolver(
    robot_description_path=DESCRIPTION_PATH,          # 로봇 설명 파일
    urdf_path=URDF_PATH,                              # URDF 파일
)


def solve_ik(position):
    """TCP 가 position 에서 아래를 보게 하는 팔 관절 6개 각도 (deg) 를 돌려준다"""
    joints, success = ik_solver.compute_inverse_kinematics(
        TCP_FRAME,                                    # 목표에 맞출 프레임
        np.array(position, dtype=float),              # 목표 위치 [x, y, z] (m)
        DOWN_ORIENTATION,                             # 목표 자세 (쿼터니언 [w, x, y, z])
        warm_start=np.radians(HOME_DEG),              # 계산을 시작할 관절값 (rad). 여기서 가까운 답을 찾는다
    )
    if not success:
        raise RuntimeError(f"IK 해를 찾지 못했다: {position}")
    return np.degrees(joints)                         # rad -> deg (ARM_JOINTS 순서)


# 마커 위치는 바뀌지 않으므로 놓는 자세는 처음에 한 번만 계산한다
PLACE_POSES = {
    color: {
        "above": solve_ik([x, y, ABOVE_HEIGHT]),      # 마커 위
        "place": solve_ik([x, y, GRASP_HEIGHT]),      # 마커 위에 내려놓는 자세
    }
    for color, (_, _, (x, y)) in MARKERS.items()
}


def make_steps(cube_position):
    """큐브 위치로 한 라운드의 단계 목록을 만든다. 놓는 자세는 색을 받은 뒤 정해지므로 이름("above", "place")으로 적는다"""
    x, y, _ = cube_position
    observe_deg = solve_ik([x, y, OBSERVE_HEIGHT])    # 카메라로 내려다보는 자세
    above_deg = solve_ik([x, y, ABOVE_HEIGHT])        # 큐브 위 자세
    pick_deg = solve_ik([x, y, GRASP_HEIGHT])         # 큐브를 집는 자세
    # (이름, 종류, 값)
    #   종류 "arm"     : 값 = 팔 관절 목표 (deg) 또는 놓는 자세 이름 ("above", "place"). 도착하면 다음 단계
    #   종류 "color"   : 값 없음. 같은 색 결과가 COLOR_FRAMES 만큼 연속으로 오면 다음 단계
    #   종류 "gripper" : 값 = "open" 또는 "close". GRIPPER_FRAMES 가 지나면 다음 단계
    return [
        ("관찰 자세로", "arm", observe_deg),
        ("색 확인", "color", None),
        ("큐브 위로", "arm", above_deg),
        ("내려가기", "arm", pick_deg),
        ("그리퍼 닫기", "gripper", "close"),
        ("들어 올리기", "arm", above_deg),
        ("마커 위로", "arm", "above"),
        ("내려놓기", "arm", "place"),
        ("그리퍼 열기", "gripper", "open"),
        ("올라가기", "arm", "above"),
    ]


# ------------------------------------------------------------
# (나) Task: 씬의 로봇 + 파랑/초록 큐브 + 마커
# ------------------------------------------------------------
class ColorSortTask(BaseTask):
    """열어 둔 씬(로봇, 바닥, 카메라 그래프)에 큐브 2 개와 마커 2 개를 더하고, 시작 자세를 맞추고, 관측값을 돌려준다"""

    # 언제: 첫 world.reset() 때 한 번
    # 할 일: 장면에 물체 추가 (바닥과 로봇은 씬에 이미 있다)
    def set_up_scene(self, scene):
        super().set_up_scene(scene)

        # 씬의 빨간 큐브(red_block)는 쓰지 않으므로 끈다 (카메라와 색상 감지에 섞이지 않게)
        red_block = get_prim_at_path("/World/red_block")
        if red_block.IsValid():
            red_block.SetActive(False)

        # 그리퍼와 로봇 객체 (L7, L8 과 같음. 로봇은 씬에 있으므로 참조를 다시 넣지 않는다)
        gripper = ParallelGripper(
            end_effector_prim_path=f"{ROBOT_PRIM}/gripper_body",  # 그리퍼 본체 prim
            joint_prim_names=["finger_joint"],        # 구동 관절 이름 (RG2 는 하나)
            joint_opened_positions=np.array([0.0]),   # 열림 각도 (rad)
            joint_closed_positions=np.array([1.0]),   # 닫힘 각도 (rad). 큐브를 잡으면 그 전에 멈춘다
            use_mimic_joints=True,                    # 나머지 손가락은 USD 의 mimic 관절이 따라 움직임
        )
        self._robot = scene.add(
            SingleManipulator(prim_path=ROBOT_PRIM, name="m0609",
                              end_effector_prim_path=f"{ROBOT_PRIM}/gripper_body", gripper=gripper)
        )
        self._task_objects[self._robot.name] = self._robot

        # 큐브 2 개: 처음에는 카메라에 안 보이는 대기 위치에 둔다
        self._cubes = {}
        for color, (name, rgb, (x, y)) in CUBES.items():
            self._cubes[color] = scene.add(
                DynamicCuboid(
                    prim_path=f"/World/{name}", name=name,
                    position=np.array([x, y, CUBE_Z]),        # 큐브 중심 위치 (m)
                    scale=np.array([CUBE_SIZE] * 3), size=1.0,  # 실제 크기 = 0.0515 m
                    color=np.array(rgb),
                )
            )
            self._task_objects[name] = self._cubes[color]

        # 마커 2 개: 놓을 위치를 보여 주는 얇은 판 (충돌 없음)
        for color, (name, rgb, (x, y)) in MARKERS.items():
            self._task_objects[name] = scene.add(
                VisualCuboid(
                    prim_path=f"/World/{name}", name=name,
                    position=np.array([x, y, 0.001]),
                    scale=np.array([0.08, 0.08, 0.002]), size=1.0,  # 가로 8 cm, 두께 2 mm
                    color=np.array(rgb),
                )
            )

    # 언제: 매 world.reset() 끝에 (Stop -> Play 때도)
    # 할 일: 팔은 홈 자세, 그리퍼는 열림 (큐브는 world.reset() 이 대기 위치로 되돌린다)
    def post_reset(self):
        home = np.zeros(self._robot.num_dof)
        for name, angle in zip(ARM_JOINTS, np.radians(HOME_DEG)):
            home[self._robot.get_dof_index(name)] = angle
        self._robot.set_joint_positions(home)
        self._robot.gripper.set_joint_positions(self._robot.gripper.joint_opened_positions)

    # 할 일: 큐브 하나를 position 으로 순간이동 (멈춘 상태로)
    def _move_cube(self, color, position):
        cube = self._cubes[color]
        cube.set_world_pose(position=np.array(position), orientation=np.array([1.0, 0.0, 0.0, 0.0]))
        cube.set_linear_velocity(np.zeros(3))   # 옮기기 전의 속도가 남지 않게
        cube.set_angular_velocity(np.zeros(3))

    # 할 일: 모든 큐브를 대기 위치로 되돌린다 (한 사이클이 끝날 때마다)
    def park_cubes(self):
        for color, (_, _, (x, y)) in CUBES.items():
            self._move_cube(color, [x, y, CUBE_Z])

    # 할 일: 대기 중인 큐브 하나를 pick 영역의 랜덤 위치로 옮기고, 그 위치를 돌려준다
    def spawn_cube(self, color):
        position = np.array([rng.uniform(*PICK_AREA_X), rng.uniform(*PICK_AREA_Y), CUBE_Z])
        self._move_cube(color, position)
        return position

    # 언제: world.get_observations() 를 부를 때
    # 돌려줄 값: {물체 이름: {값 이름: 값}}
    def get_observations(self):
        observations = {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),  # 관절 12개의 지금 각도 (rad)
            },
        }
        for cube in self._cubes.values():
            observations[cube.name] = {
                "position": cube.get_world_pose()[0],                  # 큐브 중심 위치 [x, y, z] (m)
            }
        return observations

    # 언제: 메인 루프가 물체 이름을 알아낼 때 (task.get_params())
    # 돌려줄 값: {파라미터 이름: {"value": 값}}
    def get_params(self):
        return {
            "robot_name": {"value": self._robot.name},                                         # "m0609"
            "cube_names": {"value": {color: cube.name for color, cube in self._cubes.items()}},  # {1: "blue_cube", 2: "green_cube"}
        }


# ------------------------------------------------------------
# (다) Controller: 한 라운드의 단계를 차례로 진행한다 (L7, L8 의 StepController + "color" 단계)
# ------------------------------------------------------------
class ColorPickController(BaseController):
    """단계 목록을 진행한다. "color" 단계에서 받은 색으로 놓을 마커를 정한다"""

    def __init__(self, name, gripper, arm_indices):
        super().__init__(name)
        self._gripper = gripper          # 그리퍼 객체 (forward("open" / "close") 로 명령을 만들어 준다)
        self._arm_indices = arm_indices  # 명령을 보낼 관절이 관절 배열의 몇 번째인지 (팔 6개)
        self.reset()

    # 언제: 처음 만들 때, 그리고 메인 루프가 world.reset() 할 때
    def reset(self):
        self.steps = []        # 이번 라운드의 단계 목록 (라운드를 시작할 때 채운다)
        self.step = 0          # 지금 단계 번호
        self.color = None      # 이번 라운드에서 확정한 색 번호 (1: 파랑, 2: 초록)
        self._step_frames = 0  # 지금 단계에서 지난 프레임 수
        self._same_color = 0   # 같은 색 결과가 연속으로 온 프레임 수
        self._last_color = None

    # 새 라운드: 단계 목록을 바꾸고 처음부터
    def start(self, steps):
        self.reset()
        self.steps = steps

    # 모든 단계를 끝냈는지
    def is_done(self):
        return self.step >= len(self.steps)

    # 언제: 매 프레임 메인 루프가 부른다
    # 받는 값: 팔 관절 6개의 지금 각도 (rad), 가장 최근에 받은 색 번호 (없으면 None)
    def forward(self, current_arm, received_color):
        if self.is_done():
            return None
        name, kind, value = self.steps[self.step]
        self._step_frames += 1

        # 1) 이번 단계의 명령과 "끝났는지"
        if kind == "arm":
            if isinstance(value, str):                 # "above", "place": 받은 색의 마커 자세
                value = PLACE_POSES[self.color][value]
            action = ArticulationAction(joint_positions=np.radians(value), joint_indices=self._arm_indices)
            finished = np.max(np.abs(np.degrees(current_arm) - value)) < TOLERANCE_DEG
        elif kind == "color":
            action = None                              # 팔은 관찰 자세 그대로
            if received_color in COLOR_NAMES and received_color == self._last_color:
                self._same_color += 1
            else:
                self._same_color = 0
            self._last_color = received_color
            if self._step_frames == COLOR_TIMEOUT:
                print("색 결과를 받지 못함: PC B 의 색상 감지 노드와 ROS_DOMAIN_ID 를 확인한다")
            finished = self._same_color >= COLOR_FRAMES
            if finished:
                self.color = received_color
        else:
            action = self._gripper.forward(action=value)
            finished = self._step_frames >= GRIPPER_FRAMES
        # 2) 끝났으면 다음 단계로
        if finished:
            self.step += 1
            self._step_frames = 0
        return action


# ------------------------------------------------------------
# (라) ROS2: PC B 가 보내는 색 번호 받기
# ------------------------------------------------------------
received_color = None  # 가장 최근에 받은 색 번호 (0: 못 찾음, 1: 파랑, 2: 초록)


def on_color(msg):
    """색 번호를 받으면 저장만 한다. 사용은 메인 루프의 Controller 가 한다 (R3 과 같은 구조)"""
    global received_color
    received_color = msg.data


rclpy.init()
node = rclpy.create_node("color_sorter")                               # ROS2 노드 이름
node.create_subscription(Int32, COLOR_TOPIC, on_color, 10)             # /color_id 구독

# ------------------------------------------------------------
# (마) 등록과 준비: 씬을 열고 Task 를 등록한다
# ------------------------------------------------------------
open_stage(SCENE_USD)                                                  # 로봇, 바닥, Clock Graph, Camera Graph 가 들어 있는 씬
world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 120.0, rendering_dt=1.0 / 60.0)
task = ColorSortTask(name="color_sort")
world.add_task(task)
world.reset()  # 여기서 task.set_up_scene() -> task.post_reset() 이 불린다

params = task.get_params()
robot_name = params["robot_name"]["value"]            # "m0609"
cube_names = params["cube_names"]["value"]            # {1: "blue_cube", 2: "green_cube"}: 관측값에서 큐브 위치를 꺼낼 때 쓴다
robot = world.scene.get_object(robot_name)            # 이름으로 장면에서 로봇 객체를 찾는다 (apply_action 용)
arm = [robot.get_dof_index(name) for name in ARM_JOINTS]  # 팔 관절 6개가 관절 배열(12개)의 몇 번째인지
controller = ColorPickController("color_pick_controller", gripper=robot.gripper, arm_indices=arm)


def start_round():
    """모든 큐브를 대기 위치에 두고, 그중 하나를 골라 pick 영역에 놓은 뒤 그 위치로 단계 목록을 만든다"""
    task.park_cubes()                        # 한 사이클이 끝나면 모든 큐브는 대기 위치로
    color = int(rng.choice(list(CUBES)))     # 대기 중인 두 개 중 하나
    position = task.spawn_cube(color)
    controller.start(make_steps(position))
    print(f"새 라운드 | 큐브 x {position[0]:.2f} y {position[1]:.2f} m (색은 카메라로 확인)")


start_round()  # 첫 라운드 준비 (큐브 하나를 pick 영역에 놓는다)
world.pause()  # 화면 왼쪽의 Play(▶) 를 누르면 시작
print("Play 를 누르면 시작한다. PC B 에서 색상 감지 노드를 실행해 둔다")

# ============================================================
# 3. 메인 루프 (창이 열려 있는 동안 한 프레임씩 진행)
# ============================================================
reset_needed = False
last_step = 0  # 바로 전 프레임의 단계 번호 (단계가 바뀌면 출력)
rounds = 0     # 끝낸 라운드 수
while simulation_app.is_running():
    world.step(render=True)
    rclpy.spin_once(node, timeout_sec=0.0)  # 도착한 색 번호가 있으면 on_color 실행 (기다리지 않음)

    # 1) Stop 을 감지해 둔다
    if world.is_stopped() and not reset_needed:
        reset_needed = True
    if not world.is_playing():
        continue
    # 2) Stop 다음 첫 Play 프레임: 장면과 컨트롤러를 함께 처음으로, 첫 라운드 다시 준비
    if reset_needed:
        world.reset()
        controller.reset()
        rounds = 0
        last_step = 0
        reset_needed = False
        print("reset 완료: 처음부터 다시 시작")
        start_round()

    # 3) 관측 (Task)
    observations = world.get_observations()
    current_arm = observations[robot_name]["joint_positions"][arm]  # 12개 중 팔 6개만
    # 4) 계산 (Controller)
    action = controller.forward(current_arm, received_color)
    # 5) 명령 ("색 확인" 단계는 None: 팔은 관찰 자세 그대로)
    if action is not None:
        robot.apply_action(action)

    # 단계가 끝났을 때 한 번 출력
    if controller.step != last_step:
        name = controller.steps[last_step][0]
        if name == "색 확인":
            print(f"  색 확인 끝 | {COLOR_NAMES[controller.color]} -> {MARKERS[controller.color][0]} 에 놓는다")
        last_step = controller.step

    # 6) 라운드가 끝나면 다음 라운드
    if controller.is_done():
        rounds += 1
        x, y, z = observations[cube_names[controller.color]]["position"]  # 놓은 큐브 위치 (관측값)
        print(f"  라운드 {rounds} 완료 | {COLOR_NAMES[controller.color]} 큐브 x {x:.2f} y {y:.2f} z {z:.3f} m")
        start_round()
        last_step = 0

# ============================================================
# 4. 종료
# ============================================================
node.destroy_node()
rclpy.shutdown()
simulation_app.close()
