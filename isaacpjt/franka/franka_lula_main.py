from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

# ── SimulationApp 생성 이후에 sys.path 추가 ────────────
import os
import sys

PROJECT_PATH = "/home/rokey/cobot3_ws/isaacpjt/franka"
ISAAC_ROS2_PATH = "/home/rokey/isaacsim/exts/isaacsim.ros2.bridge/humble/rclpy"

sys.path.insert(0, PROJECT_PATH)
sys.path.insert(0, ISAAC_ROS2_PATH)

print(f"[DEBUG] sys.path[0]: {sys.path[0]}")
print(f"[DEBUG] sys.path[1]: {sys.path[1]}")

import rclpy
from isaacsim.core.api import World
from isaacsim.core.utils.stage import open_stage

scene_path = os.path.join(PROJECT_PATH, "scene/franka_pick_and_place_banana.usd")
open_stage(usd_path=scene_path)

world = World()
world.reset()

from controllers.lula_controller import LulaController
from controllers.gripper_controller import GripperController
from controllers.pick_place import PickPlaceTask
from franka_utils.ros2_bridge import ROS2Bridge

rclpy.init()
controller = LulaController(world)
gripper    = GripperController(world)
bridge     = ROS2Bridge()
task       = PickPlaceTask(controller, gripper, bridge)

while simulation_app.is_running():
    world.step(render=True)
    rclpy.spin_once(bridge, timeout_sec=0)
    task.update()

rclpy.shutdown()
simulation_app.close()