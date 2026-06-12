from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import time
import numpy as np
import omni.usd
from pxr import Usd, UsdGeom

from isaacsim.core.api import World
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.robot.manipulators.examples.franka import Franka

_THIS_DIR = Path(__file__).resolve().parent
USD_PATH        = str(_THIS_DIR / "Collected_no_franka/no_franka.usd")
FRANKA_PRIM     = "/World/franka"
FRANKA_POSITION = np.array([0.0, -0.5, 0.0])
FRANKA_ORIENT   = euler_angles_to_quat(np.array([0.0, 0.0, 90.0]), degrees=True)

my_world = World(stage_units_in_meters=1.0)

# 환경 USD 로드
stage = omni.usd.get_context().get_stage()
world_prim = stage.GetPrimAtPath("/World")
if not world_prim.IsValid():
    world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
world_prim.GetReferences().AddReference(USD_PATH)

# Franka 생성
franka = my_world.scene.add(
    Franka(
        prim_path=FRANKA_PRIM,
        name="franka",
        position=FRANKA_POSITION,
        orientation=FRANKA_ORIENT,
    )
)

my_world.reset()

while simulation_app.is_running():
    my_world.step(render=True)
    time.sleep(0.016)

simulation_app.close()