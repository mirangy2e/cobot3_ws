# =============================================================
# tuna_sdg.py
# Isaac Sim 5.1.0 Standalone. 실행: isaac_python tuna_sdg.py
#
# 구성: 수직 top-down 카메라 + 참치캔 4종(정상/경미/심함/스크래치)
#       이진 라벨(normal_can / defective_can), grazing key light로 결함 음영,
#       바닥 + 위치/yaw/조명 무작위화, 항상 프레임 안.
# =============================================================

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})   # 보면서 확인하려면 False

from isaacsim.core.utils.extensions import enable_extension
enable_extension("omni.replicator.core")
simulation_app.update()

import random, os, shutil, math
import numpy as np
import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, UsdShade, Gf, Sdf

try:
    from isaacsim.core.utils.semantics import add_update_semantics
except Exception:
    from omni.isaac.core.utils.semantics import add_update_semantics

# ---------------- 경로 (실제 파일명 반영: usd_files/*.usdc) ----------------
BASE = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/assets/tuna_can_model/usd_files"
NORMAL_USD   = f"{BASE}/can_normal.usdc"
SEVERE_USD   = f"{BASE}/can_demaged_severe.usdc"
SCRATCH_USD  = f"{BASE}/can_demaged_scratch.usdc"
# 제외: light(측면 경미 dent), decoboko(미세 표면 요철)
#   -> 실제 웹캠 soft light에서 정상과 구분 안 됨 -> 라벨 노이즈 되어 제거
# LIGHT_USD    = f"{BASE}/can_damage_light.usdc"
# DECOBOKO_USD = f"{BASE}/can_demaged_decoboko.usdc"

# (wrapper 경로, USD, 라벨)  이진: 정상=ok, 불량=ng.  ok:ng = 2:2 (1:1 균형)
CANS = [
    ("/World/can_normal",  NORMAL_USD,  "ok"),
    ("/World/can_normal2", NORMAL_USD,  "ok"),   # 정상 1개 더 -> 클래스 균형 1:1
    ("/World/can_severe",  SEVERE_USD,  "ng"),   # 림·뚜껑 변형(soft light에서도 보임)
    ("/World/can_scratch", SCRATCH_USD, "ng"),   # 스크래치(albedo -> 조명 무관하게 보임)
]

# 4개 슬롯(겹침 방지) + 프레임 안 유지. 매 프레임 셔플 + jitter + yaw
SLOT_CENTERS = [(-0.10, -0.09), (0.10, -0.09),
                (-0.10, 0.09), (0.10, 0.09)]
JITTER   = 0.012
CAN_Z    = 0.0
GROUND_Z = -0.045

OUT_DIR    = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/sdg_raw"
NUM_FRAMES = 200

# 수직 top-down 카메라: (0,0,H)에서 회전 없이 -Z(수직 하방) 바라봄
CAM_H      = 0.45
CAM_FOCAL  = 24.0        # 넓은 화각(약 47deg) -> 슬롯 전체 수용
CAM_CLIP   = (0.01, 1000.0)
RESOLUTION = (1280, 1280)   # YOLO용: 정사각형(letterbox 방지) + 결함 디테일 여유
CAM_TARGET   = (0.0, 0.0, 0.0)   # 카메라가 바라보는 중심(캔 영역)
TILT_MAX_DEG = 25.0              # 수직(0) ~ 이 각도까지 매 프레임 무작위 기울임


def update(n=1):
    for _ in range(n):
        simulation_app.update()


def set_camera(cam_op, tilt_deg, azim_deg):
    """수직(0deg)~기울임(tilt_deg)까지 target 주위 호에 카메라 배치 + look_at.
       실제 웹캠 각도 미정 -> 각도 무작위화로 배포 각도에 강인하게."""
    th = math.radians(tilt_deg)
    ph = math.radians(azim_deg)
    tx, ty, tz = CAM_TARGET
    eye = Gf.Vec3d(tx + CAM_H * math.sin(th) * math.cos(ph),
                   ty + CAM_H * math.sin(th) * math.sin(ph),
                   tz + CAM_H * math.cos(th))
    view = Gf.Matrix4d()
    view.SetLookAt(eye, Gf.Vec3d(tx, ty, tz), Gf.Vec3d(0.0, 1.0, 0.0))
    cam_op.Set(view.GetInverse())


def setup_movable(prim):
    xform = UsdGeom.Xformable(prim)
    keep_scale = None
    for op in xform.GetOrderedXformOps():
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            keep_scale = op.Get()
    xform.ClearXformOpOrder()
    t_op = xform.AddTranslateOp()
    r_op = xform.AddRotateXYZOp()
    if keep_scale is not None:
        xform.AddScaleOp().Set(keep_scale)
    return t_op, r_op


def build_env(stage):
    UsdGeom.Xform.Define(stage, Sdf.Path("/World"))

    # 조명: 선명한 대비(림 구겨짐 보이게). dome 낮추고 key 강화.
    dome = UsdLux.DomeLight.Define(stage, Sdf.Path("/World/Lights/Dome"))
    dome.CreateIntensityAttr(500.0)          # 앰비언트 낮춰 뿌연 느낌 제거
    key = UsdLux.DistantLight.Define(stage, Sdf.Path("/World/Lights/Key"))
    key.CreateIntensityAttr(1300.0)          # 방향광 강화 -> 선명 + 림 그림자
    key.CreateAngleAttr(1.0)                  # 각크기 작게 -> 윤곽/그림자 선명
    kx = UsdGeom.Xformable(key.GetPrim()); kx.ClearXformOpOrder()
    key_rot = kx.AddRotateXYZOp()          # 루프에서 방위각 갱신
    key_rot.Set(Gf.Vec3f(-40.0, 0.0, 0.0))  # 중간 각도(선명하되 그림자 과하지 않게)

    # 바닥: displayColor 방식 (머티리얼/바인딩 불필요, RTX 반영 확인됨)
    ground = UsdGeom.Cube.Define(stage, Sdf.Path("/World/Ground"))
    ground.CreateSizeAttr(1.0)
    gx = UsdGeom.Xformable(ground.GetPrim()); gx.ClearXformOpOrder()
    gx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, GROUND_Z))
    gx.AddScaleOp().Set(Gf.Vec3f(2.0, 2.0, 0.01))
    ground.GetDisplayColorAttr().Set([Gf.Vec3f(0.35, 0.33, 0.30)])
    ground_mat = ground   # 루프에서 ground_mat.GetDisplayColorAttr().Set(...)로 색 변경

    # 카메라: 매 프레임 수직~기울임으로 각도 무작위화 (TransformOp을 루프에서 갱신)
    cam = UsdGeom.Camera.Define(stage, Sdf.Path("/World/TopCam"))
    cam.CreateClippingRangeAttr(Gf.Vec2f(CAM_CLIP[0], CAM_CLIP[1]))
    cam.CreateFocalLengthAttr(CAM_FOCAL)
    cx = UsdGeom.Xformable(cam.GetPrim()); cx.ClearXformOpOrder()
    cam_op = cx.AddTransformOp()            # 루프에서 look_at 행렬로 각도 갱신
    cam_op.Set(Gf.Matrix4d(1.0))

    return dome, key, key_rot, cam_op, ground_mat, "/World/TopCam"


def main():
    if os.path.isdir(OUT_DIR):
        shutil.rmtree(OUT_DIR)
    os.makedirs(OUT_DIR, exist_ok=True)

    ctx = omni.usd.get_context()
    ctx.new_stage()
    update(2)
    stage = ctx.get_stage()

    dome, key, key_rot, cam_op, ground_mat, cam_path = build_env(stage)

    # 캔 AddReference
    for path, usd, label in CANS:
        prim = stage.DefinePrim(Sdf.Path(path), "Xform")
        prim.GetReferences().AddReference(usd)
    update(30)   # reference 로딩 대기
    print("[STEP] references loaded")

    # 시맨틱 + 이동/회전 op
    cans = []
    for path, usd, label in CANS:
        prim = stage.GetPrimAtPath(Sdf.Path(path))
        if not prim.IsValid():
            print(f"[ERR] prim not found: {path}")
            simulation_app.close(); return
        #시멘틱 라벨링
        add_update_semantics(prim, semantic_label=label, type_label="class")
        t_op, r_op = setup_movable(prim)
        cans.append({"t": t_op, "r": r_op})
    print("[STEP] semantics + movable ready")

    # render product + writer
    rp = rep.create.render_product(cam_path, RESOLUTION)
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(output_dir=OUT_DIR, rgb=True, bounding_box_2d_tight=True)
    writer.attach([rp])
    update(15)   # 워밍업
    print("[STEP] camera / writer ready")

    # 무작위화 + 캡처
    for i in range(NUM_FRAMES):
        slots = SLOT_CENTERS[:]
        random.shuffle(slots)                  # 캔-슬롯 배치 셔플
        for c, (sx, sy) in zip(cans, slots):
            x = sx + random.uniform(-JITTER, JITTER)
            y = sy + random.uniform(-JITTER, JITTER)
            yaw = random.uniform(0.0, 360.0)
            c["t"].Set(Gf.Vec3d(x, y, CAN_Z))
            c["r"].Set(Gf.Vec3f(0.0, 0.0, yaw))
        # 조명: 바닥이 안 날아가되 캔 대비는 유지하는 중간값
        key_rot.Set(Gf.Vec3f(-40.0, 0.0, random.uniform(0.0, 360.0)))
        key.GetIntensityAttr().Set(random.uniform(500.0, 800.0))
        dome.GetIntensityAttr().Set(random.uniform(250.0, 450.0))
        # 배경: displayColor로 어두운~중간 톤 무작위 (과노출로 흰색 날아가지 않게 낮게)
        base = random.uniform(0.18, 0.50)
        ground_mat.GetDisplayColorAttr().Set([Gf.Vec3f(base + 0.04, base, base - 0.05)])
        # 카메라: 수직(0)~TILT_MAX_DEG 사이로 기울임 + 방위각 무작위
        set_camera(cam_op, random.uniform(0.0, TILT_MAX_DEG), random.uniform(0.0, 360.0))

        update(6)                    # 색/조명/카메라 변경 반영 대기 (배경색 안 바뀌던 원인)
        rep.orchestrator.step(rt_subframes=4)
        print(f"[CAP] frame {i}")

    print(f"[DONE] {NUM_FRAMES} frames -> {OUT_DIR}")


if __name__ == "__main__":
    main()
    # 생성 완료 후에도 GUI 유지: 창을 직접 닫을 때까지 렌더 루프 (headless=False 여야 보임)
    print("[INFO] generation done. GUI stays open. Close the window to exit.")
    while simulation_app.is_running():
        simulation_app.update()
    # simulation_app.close()