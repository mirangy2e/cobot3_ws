import omni.usd
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

cube = UsdGeom.Cube.Define(stage, "/World/mirancube2")
cube.GetSizeAttr().Set(0.1)

xform = UsdGeom.XformCommonAPI(cube.GetPrim())
xform.SetTranslate(Gf.Vec3d(0.3, 0.0, 0.05))