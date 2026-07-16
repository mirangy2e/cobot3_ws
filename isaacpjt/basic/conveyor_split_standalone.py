#!/usr/bin/env python3
# conveyor_split_standalone.py
# Event-driven: set direction -> spawn cube -> wait for fall -> toggle -> repeat.
# Forces the Sorter ActionGraph to evaluate each step so binary_switch actually
# drives the roller rotation (write_prim_attribute -> Group_00..11).
# Run with isaac_python (do NOT source ROS2 setup.bash).

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

# Enable the conveyor extension so the IsaacConveyor OmniGraph node type is registered.
# Without this, conveyor_belt_01 loads as an unknown/empty node in standalone mode.
from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.asset.gen.conveyor")
simulation_app.update()

import time
import numpy as np
import omni.graph.core as og
from isaacsim.core.api import World
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.stage import open_stage
from isaacsim.core.utils.prims import is_prim_path_valid

# --- Configuration -----------------------------------------------------------
USD_PATH = "/home/rokey/cobot3_ws/isaacpjt/basic/Collected_Conveyor_split/conveyor_split.usd"
GRAPH_PATH = "/World/ConveyorTrack/Sorter/ActionGraph"
SWITCH_ATTR = GRAPH_PATH + "/binary_switch.inputs:value"
CUBE_PATH = "/World/Cube"

PHYSICS_DT = 1.0 / 60.0

CUBE_START_POS = np.array([-0.45185, 0.07358, 1.26558])
CUBE_START_ORI = np.array([1.0, 0.0, 0.0, 0.0])  # (w, x, y, z)
RESPAWN_Z_THRESHOLD = 0.5
RESPAWN_COOLDOWN_STEPS = 30
# -----------------------------------------------------------------------------

open_stage(usd_path=USD_PATH)
world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT)
world.reset()

if not is_prim_path_valid(CUBE_PATH):
    raise RuntimeError(f"Cube prim not found at {CUBE_PATH}")
cube = SingleRigidPrim(prim_path=CUBE_PATH, name="cube")
cube.initialize()

# Grab the Sorter ActionGraph so we can force-evaluate it each step
sorter_graph = og.get_graph_by_path(GRAPH_PATH)
print(f"[conveyor] Sorter graph valid: {sorter_graph.is_valid()}")

def set_switch(value: bool):
    og.Controller.attribute(SWITCH_ATTR).set(value)
    print(f"[conveyor] direction = {'DIVERT' if value else 'STRAIGHT'}")

def respawn_cube():
    cube.set_world_pose(position=CUBE_START_POS, orientation=CUBE_START_ORI)
    cube.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
    cube.set_angular_velocity(np.array([0.0, 0.0, 0.0]))
    print("[conveyor] cube spawned")

# Event-driven loop
current_state = False  # STRAIGHT
set_switch(current_state)
respawn_cube()
# evaluate once so the roller rotation matches the initial switch state
og.Controller.evaluate_sync(sorter_graph)
cooldown = RESPAWN_COOLDOWN_STEPS

print("[conveyor] Started: toggle -> spawn -> wait for fall -> repeat")

while simulation_app.is_running():
    frame_start = time.time()

    world.step(render=True)
    # Force the Sorter ActionGraph to evaluate so binary_switch drives the rollers
    og.Controller.evaluate_sync(sorter_graph)

    if cooldown > 0:
        cooldown -= 1
    else:
        pos, _ = cube.get_world_pose()
        if pos[2] < RESPAWN_Z_THRESHOLD:
            current_state = not current_state
            set_switch(current_state)
            respawn_cube()
            cooldown = RESPAWN_COOLDOWN_STEPS

    # Real-time pacing
    elapsed = time.time() - frame_start
    if elapsed < PHYSICS_DT:
        time.sleep(PHYSICS_DT - elapsed)

simulation_app.close()