# tuna_sdg_anomaly.py
# anomalib(PatchCore)용 합성데이터 생성.
# Isaac Sim 5.1.0 Standalone. 실행: isaac_python tuna_sdg_anomaly.py
#
# 이상탐지는 "정상만 학습"하므로 데이터 구조가 검출과 다르다:
#   dataset/anomaly/
#      train/good/     <- 정상만 (학습용, 많이)
#      test/good/      <- 정상 (검증용)
#      test/defect/    <- 결함 (검출되는지 확인용)
#
# 검출(bbox)과 다른 점: 캔 1개만 화면에 크게 담는다(클로즈업).
#   -> 이상탐지는 대상이 프레임을 채워야 미세 이탈을 잘 잡는다.

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": True})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.replicator.core")
simulation_app.update()

import random, os, shutil, math, glob
import numpy as np
import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, Gf, Sdf

try:
    from isaacsim.core.prims import SingleGeometryPrim
    from isaacsim.core.api.materials import OmniPBR
except Exception:
    from omni.isaac.core.prims import GeometryPrim as SingleGeometryPrim
    from omni.isaac.core.materials import OmniPBR

# ---------------- 경로 ----------------
BASE = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/assets/tuna_can_model/usd_files"
NORMAL_USD = f"{BASE}/can_normal.usdc"
SEVERE_USD = f"{BASE}/can_demaged_severe.usdc"

OUT_ROOT = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/anomaly"

# 생성 매수
N_TRAIN_GOOD  = 250    # 학습용 정상 (많을수록 좋음)
N_TEST_GOOD   = 30     # 검증용 정상
N_TEST_DEFECT = 30     # 검증용 결함

# 캔 1개를 화면 중앙에 클로즈업
CAN_PATH = "/World/can"
CAN_Z    = 0.0
GROUND_Z = -0.045
JITTER   = 0.004            # 위치 미세 흔들림 (캔이 프레임 벗어나지 않게 작게)

CAM_H        = 0.135        # 캔이 프레임을 크게 채우도록 (작을수록 캔이 커짐)
CAM_FOCAL    = 24.0
CAM_CLIP     = (0.01, 1000.0)
CAM_TARGET   = (0.0, 0.0, 0.0)
TILT_MAX_DEG = 15.0         # 수직~15도 (실제 웹캠 각도 대응)
RESOLUTION   = (512, 512)   # anomalib 입력에 적당 (256~512)


def update(n=1):
    for _ in range(n):
        simulation_app.update()


def set_camera(cam_op, tilt_deg, azim_deg):
    th, ph = math.radians(tilt_deg), math.radians(azim_deg)
    tx, ty, tz = CAM_TARGET
    eye = Gf.Vec3d(tx + CAM_H * math.sin(th) * math.cos(ph),
                   ty + CAM_H * math.sin(th) * math.sin(ph),
                   tz + CAM_H * math.cos(th))
    view = Gf.Matrix4d()
    view.SetLookAt(eye, Gf.Vec3d(tx, ty, tz), Gf.Vec3d(0.0, 1.0, 0.0))
    cam_op.Set(view.GetInverse())


def setup_movable(prim):
    """기존 scale 보존하며 translate/rotate op 생성."""
    xform = UsdGeom.Xformable(prim)
    keep = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            keep = op.Get()
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    r_op = xform.AddRotateXYZOp()
    if keep is not None:
        xform.AddScaleOp().Set(keep)
    return t_op, r_op


def build_env(stage):
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))

    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/Dome"))
    dome.CreateIntensityAttr(1000.0)
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/Key"))
    key.CreateIntensityAttr(200.0)
    key.CreateAngleAttr(2.0)
    kx = UsdGeom.Xformable(key.GetPrim()); kx.ClearXformOpOrder()
    key_rot = kx.AddRotateXYZOp()
    key_rot.Set(Gf.Vec3f(-40.0, 0.0, 0.0))

    # 바닥: 검은 무광 재질.
    #   displayColor 만으로는 광택(specular)이 남아 정반사 hotspot 이 생긴다.
    #   -> OmniPBR 로 roughness=1.0(완전 무광), metallic=0 으로 설정해 반사를 없앤다.
    UsdGeom.Cube.Define(stage, Sdf.Path("/World/Ground")).CreateSizeAttr(1.0)
    ground = SingleGeometryPrim("/World/Ground", name="ground",
                                position=np.array([0.0, 0.0, GROUND_Z]),
                                scale=np.array([2.0, 2.0, 0.01]))
    ground_mat = OmniPBR(prim_path="/World/Looks/GroundMat", name="ground_mat",
                         color=np.array([0.0, 0.0, 0.0]))       # 완전 검정
    try:
        ground_mat.set_reflection_roughness(1.0)      # 완전 무광 -> 정반사 없음
        ground_mat.set_metallic_constant(0.0)
    except Exception as e:
        print(f"[WARN] roughness/metallic set failed: {e}")
    ground.apply_visual_material(ground_mat)

    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/TopCam"))
    cam.CreateClippingRangeAttr(Gf.Vec2f(CAM_CLIP[0], CAM_CLIP[1]))
    cam.CreateFocalLengthAttr(CAM_FOCAL)
    cx = UsdGeom.Xformable(cam.GetPrim()); cx.ClearXformOpOrder()
    cam_op = cx.AddTransformOp()
    cam_op.Set(Gf.Matrix4d(1.0))

    return dome, key, key_rot, ground_mat, cam_op, "/World/TopCam"


def randomize(t_op, r_op, dome, key, key_rot, ground_mat, cam_op):
    """무작위화: 캔 위치·yaw / 조명 / 배경 / 카메라 각도."""
    x = random.uniform(-JITTER, JITTER)
    y = random.uniform(-JITTER, JITTER)
    t_op.Set(Gf.Vec3d(x, y, CAN_Z))
    r_op.Set(Gf.Vec3f(0.0, 0.0, random.uniform(0.0, 360.0)))

    # 조명: 그림자 최소화. key(방향광)가 그림자를 만들므로 낮추고, dome(환경광)으로 밝힌다.
    key_rot.Set(Gf.Vec3f(-30.0, 0.0, random.uniform(0.0, 360.0)))
    key.GetIntensityAttr().Set(random.uniform(150.0, 300.0))    # 낮게 -> 그림자 거의 없음
    dome.GetIntensityAttr().Set(random.uniform(900.0, 1200.0))  # 환경광으로 캔 밝기 확보

    # 배경: 완전 검정 유지. dome이 세도 재질이 순흑이면 어둡게 남는다.
    base = random.uniform(0.0, 0.015)
    ground_mat.set_color(np.array([base, base, base]))

    set_camera(cam_op, random.uniform(0.0, TILT_MAX_DEG), random.uniform(0.0, 360.0))


def capture_set(usd_path, out_dir, n_frames, rp, writer, stage, env):
    """지정 USD 캔 1개를 로드해 n_frames 장 생성 후 out_dir 로 정리."""
    dome, key, key_rot, ground_mat, cam_op, _ = env

    # 이전 캔 제거 후 새 캔 로드
    if stage.GetPrimAtPath(Sdf.Path(CAN_PATH)).IsValid():
        stage.RemovePrim(Sdf.Path(CAN_PATH))
        update(3)
    prim = stage.DefinePrim(Sdf.Path(CAN_PATH), "Xform")
    prim.GetReferences().AddReference(usd_path)
    update(30)                                   # reference 로딩 대기
    t_op, r_op = setup_movable(stage.GetPrimAtPath(Sdf.Path(CAN_PATH)))

    tmp = os.path.join(OUT_ROOT, "_tmp")
    if os.path.isdir(tmp):
        shutil.rmtree(tmp)
    os.makedirs(tmp, exist_ok=True)

    writer.initialize(output_dir=tmp, rgb=True)
    writer.attach([rp])
    update(15)                                   # 워밍업

    for i in range(n_frames):
        randomize(t_op, r_op, dome, key, key_rot, ground_mat, cam_op)
        update(6)                                # 변경 반영 대기
        rep.orchestrator.step(rt_subframes=4)
    writer.detach()
    update(5)

    # tmp의 rgb_*.png -> out_dir 로 이동
    os.makedirs(out_dir, exist_ok=True)
    files = sorted(glob.glob(os.path.join(tmp, "rgb_*.png")))
    for i, f in enumerate(files):
        shutil.move(f, os.path.join(out_dir, f"{i:04d}.png"))
    shutil.rmtree(tmp, ignore_errors=True)
    print(f"[SET] {out_dir}: {len(files)} images")


def main():
    if os.path.isdir(OUT_ROOT):
        shutil.rmtree(OUT_ROOT)
    os.makedirs(OUT_ROOT, exist_ok=True)

    ctx = omni.usd.get_context()
    ctx.new_stage()
    update(2)
    stage = ctx.get_stage()

    env = build_env(stage)
    cam_path = env[5]

    rp = rep.create.render_product(cam_path, RESOLUTION)
    writer = rep.WriterRegistry.get("BasicWriter")

    # 1) 학습용 정상 (PatchCore는 이것만 학습)
    capture_set(NORMAL_USD, os.path.join(OUT_ROOT, "train", "good"),
                N_TRAIN_GOOD, rp, writer, stage, env)
    # 2) 검증용 정상
    capture_set(NORMAL_USD, os.path.join(OUT_ROOT, "test", "good"),
                N_TEST_GOOD, rp, writer, stage, env)
    # 3) 검증용 결함
    capture_set(SEVERE_USD, os.path.join(OUT_ROOT, "test", "defect"),
                N_TEST_DEFECT, rp, writer, stage, env)

    print(f"[DONE] -> {OUT_ROOT}")


if __name__ == "__main__":
    main()
    simulation_app.close()