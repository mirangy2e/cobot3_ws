# -*- coding: utf-8 -*-
"""
[step04-A] RMPflow 기본 동작 — orientation 자유
=================================================
목표: RMPflow로 큐브를 집는 기본 시퀀스를 확인한다.
     그리퍼 방향은 RMPflow가 자동으로 결정한다.

구성:
    - 공식 Franka (Isaac 에셋)
    - ground plane
    - 파란 큐브 (집을 대상)

시퀀스:
    1. OPEN_GRIPPER  : 그리퍼 열기
    2. PRE_PICK      : 큐브 위 40cm 접근
    3. PICK          : 큐브 위치로 하강
    4. CLOSE_GRIPPER : 그리퍼 닫기
    5. LIFT          : 들어올리기
    6. DONE          : 완료

실행:
    ~/isaacsim/python.sh ~/cobot3_ws/isaacpjt/practice/step04a_rmpflow_basic.py

포인트:
    - target_end_effector_orientation=None
      → RMPflow가 방향을 자유롭게 선택
      → 큐브는 집히지만 방향이 매번 달라질 수 있음
"""

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import RMPFlowController

# ================================================================
# 파라미터
# ================================================================
PHYSICS_DT   = 1.0 / 200.0
RENDERING_DT = 20.0 / 200.0

# 큐브 위치
CUBE_POS = np.array([0.4, 0.0, 0.025])

# 높이
PRE_PICK_Z = 0.40
PICK_Z     = 0.035
LIFT_Z     = 0.50

# FSM
STALL_FRAMES       = 150
STALL_DELTA        = 0.003
STATE_TIMEOUT      = 2000
OPEN_GRIPPER_WAIT  = 80
CLOSE_GRIPPER_WAIT = 300

GRIPPER_JOINT_INDICES = np.array([7, 8])
GRIPPER_OPEN          = np.array([0.04, 0.04])
GRIPPER_CLOSED        = np.array([0.00, 0.00])

# 초기 준비자세 (Franka 기본 홈)
#   joint: [j1  j2       j3   j4       j5   j6      j7      f_l   f_r ]
JOINT_HOME = np.array([0.0, -1.5708, 0.0, -1.5708, 0.0, 1.5708, 0.7854, 0.04, 0.04])

# ================================================================
# World / 씬
# ================================================================
my_world = World(
    stage_units_in_meters=1.0,
    physics_dt=PHYSICS_DT,
    rendering_dt=RENDERING_DT,
)

my_world.scene.add_default_ground_plane()

my_franka = my_world.scene.add(
    Franka(prim_path="/World/Franka", name="my_franka")
)

cube = my_world.scene.add(
    DynamicCuboid(
        prim_path="/World/cube",
        name="cube",
        position=CUBE_POS,
        size=0.05,
        color=np.array([0.0, 0.0, 1.0]),
    )
)

my_world.reset()

# ================================================================
# 컨트롤러
# ================================================================
my_controller = RMPFlowController(
    name="rmp_ctrl",
    robot_articulation=my_franka,
    physics_dt=PHYSICS_DT,
)
articulation_ctrl = my_franka.get_articulation_controller()

print("✅ RMPflow 컨트롤러 생성 완료")

# ================================================================
# 헬퍼
# ================================================================
_stall_pos   = None
_stall_count = 0

def reset_stall():
    global _stall_pos, _stall_count
    _stall_pos = None; _stall_count = 0

def is_stalled():
    global _stall_pos, _stall_count
    try:
        pos, _ = my_franka.end_effector.get_world_pose()
        if _stall_pos is None or np.linalg.norm(pos - _stall_pos) > STALL_DELTA:
            _stall_pos = pos.copy(); _stall_count = 0; return False
        _stall_count += 1
        return _stall_count >= STALL_FRAMES
    except:
        return False

def get_hand_pos():
    pos, _ = my_franka.end_effector.get_world_pose()
    return pos

def apply_arm(target):
    """
    orientation=None → RMPflow가 방향을 자유롭게 결정
    """
    action = my_controller.forward(
        target_end_effector_position=target,
        target_end_effector_orientation=None,   # ← step04-A 핵심
    )
    if action is not None:
        articulation_ctrl.apply_action(action)

def open_gripper():
    my_franka.apply_action(
        ArticulationAction(joint_positions=GRIPPER_OPEN,
                           joint_indices=GRIPPER_JOINT_INDICES)
    )

def close_gripper():
    my_franka.apply_action(
        ArticulationAction(joint_positions=GRIPPER_CLOSED,
                           joint_indices=GRIPPER_JOINT_INDICES)
    )

def reinit_rmpflow():
    robot_pos, robot_ori = my_franka.get_world_pose()
    my_controller.rmp_flow.set_robot_base_pose(robot_pos, robot_ori)

def scene_reset():
    my_world.reset()
    my_controller.reset()
    reinit_rmpflow()
    # 준비자세 적용
    my_franka.set_joint_positions(JOINT_HOME)

# ================================================================
# FSM
# ================================================================
STATE_OPEN_GRIPPER  = "OPEN_GRIPPER"
STATE_PRE_PICK      = "PRE_PICK"
STATE_PICK          = "PICK"
STATE_CLOSE_GRIPPER = "CLOSE_GRIPPER"
STATE_LIFT          = "LIFT"
STATE_DONE          = "DONE"

current_state = STATE_OPEN_GRIPPER
state_counter = 0

print("\n" + "=" * 55)
print("  ▶  step04-A : RMPflow (orientation 자유)")
print(f"  큐브 위치: {CUBE_POS}")
print("=" * 55 + "\n")

# ================================================================
# 메인 루프
# ================================================================
reset_needed = False

while simulation_app.is_running():
    my_world.step(render=True)

    if my_world.is_stopped() and not reset_needed:
        reset_needed = True

    if my_world.is_playing():

        if my_world.current_time_step_index == 0:
            scene_reset()
            current_state = STATE_OPEN_GRIPPER
            state_counter = 0
            reset_stall()

        if reset_needed:
            scene_reset()
            current_state = STATE_OPEN_GRIPPER
            state_counter = 0
            reset_stall()
            reset_needed  = False
            print("🔄 리셋")

        state_counter += 1

        # 1. 그리퍼 열기
        if current_state == STATE_OPEN_GRIPPER:
            open_gripper()
            apply_arm(np.array([CUBE_POS[0], CUBE_POS[1], LIFT_Z]))
            if state_counter == 1:
                print("[OPEN_GRIPPER] 그리퍼 열기")
            if state_counter >= OPEN_GRIPPER_WAIT:
                current_state = STATE_PRE_PICK
                state_counter = 0; reset_stall()
                print("  → PRE_PICK")

        # 2. PRE_PICK
        elif current_state == STATE_PRE_PICK:
            target = np.array([CUBE_POS[0], CUBE_POS[1], PRE_PICK_Z])
            apply_arm(target); open_gripper()
            if state_counter == 1:
                print(f"[PRE_PICK] 목표 z={PRE_PICK_Z}")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] panda_hand z={get_hand_pos()[2]:.3f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                current_state = STATE_PICK
                state_counter = 0; reset_stall()
                print("  → PICK")

        # 3. PICK
        elif current_state == STATE_PICK:
            target = np.array([CUBE_POS[0], CUBE_POS[1], PICK_Z])
            apply_arm(target); open_gripper()
            if state_counter == 1:
                print(f"[PICK] 목표 z={PICK_Z}")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] panda_hand z={get_hand_pos()[2]:.3f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                print(f"   수렴: panda_hand z={get_hand_pos()[2]:.4f}")
                current_state = STATE_CLOSE_GRIPPER
                state_counter = 0; reset_stall()
                print("  → CLOSE_GRIPPER")

        # 4. CLOSE_GRIPPER
        elif current_state == STATE_CLOSE_GRIPPER:
            hand = get_hand_pos()
            apply_arm(np.array([CUBE_POS[0], CUBE_POS[1], hand[2]]))
            close_gripper()
            if state_counter == 1:
                print("[CLOSE_GRIPPER] 그리퍼 닫기")
            if state_counter >= CLOSE_GRIPPER_WAIT:
                current_state = STATE_LIFT
                state_counter = 0; reset_stall()
                print("  → LIFT")

        # 5. LIFT
        elif current_state == STATE_LIFT:
            target = np.array([CUBE_POS[0], CUBE_POS[1], LIFT_Z])
            apply_arm(target); close_gripper()
            if state_counter == 1:
                print(f"[LIFT] 목표 z={LIFT_Z}")
            if state_counter % 100 == 0:
                print(f"   [{state_counter:4d}f] panda_hand z={get_hand_pos()[2]:.3f}")
            if is_stalled() or state_counter >= STATE_TIMEOUT:
                print(f"   ✅ LIFT 완료: panda_hand z={get_hand_pos()[2]:.4f}")
                current_state = STATE_DONE
                state_counter = 0
                print("\n🎉 시퀀스 완료!")

        # 6. DONE
        elif current_state == STATE_DONE:
            close_gripper()

print("\n⏹  종료")
simulation_app.close()
