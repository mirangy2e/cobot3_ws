# webcam_anomaly.py
# PatchCore(anomalib)로 실제 웹캠 실시간 이상탐지.
#
# 설계 규칙 (체크포인트에 전처리·후처리가 내장되어 있음):
#   1) 입력은 [0,1] float만 전달한다.
#      Resize·Normalize는 내장 PreProcessor의 몫 — 수동 정규화는 이중 적용이 된다.
#   2) 출력(score, anomaly map)은 재정규화 없이 그대로 쓴다.
#      내장 PostProcessor가 0~1 절대 스케일로 캘리브레이션함 (0.5 = 판정 경계).
#   3) 판정은 이중화: 이미지 score(안전망) OR 픽셀 contour(주 판정선).
#   4) 히트맵 표시는 고정 대비 창(DISP_LO~DISP_HI)으로 스트레칭 — 판정과 무관.
#
# 실행: python3 webcam_anomaly.py
# 조작:
#   [ / ]  : 픽셀 임계값 낮추기 / 높이기  (결함 마스크 민감도)
#   h      : heatmap 표시 on/off
#   s      : 현재 화면 저장
#   q      : 종료

import os, time, glob
import numpy as np
import cv2
import torch

import anomalib
from anomalib.models import Patchcore

# PyTorch 2.6+ 는 torch.load 의 weights_only 기본값이 True 라서
# anomalib 이 체크포인트에 저장한 객체(PrecisionType 등)가 차단된다.
# 우리가 직접 학습한 신뢰 가능한 체크포인트이므로 안전 목록에 등록한다.
_safe = []
for _name in ("PrecisionType", "TaskType", "LearningType"):
    _obj = getattr(anomalib, _name, None)
    if _obj is not None:
        _safe.append(_obj)
if _safe:
    torch.serialization.add_safe_globals(_safe)

# ---------------- 설정 ----------------
CKPT_PATH = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/results/Patchcore/tuna/latest/weights/lightning/model.ckpt"
RESULTS   = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/results"
SAVE_DIR  = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/anomaly_infer_out"

CAM_INDEX = 4              # ABKO QHD (/dev/video4)
REQ_W, REQ_H = 2560, 1440  # MJPG로 30fps (YUYV는 QHD 2fps)
INPUT_SIZE = 512           # 모델 내부에서 256으로 다시 resize되므로 크기는 유연
DISPLAY = 640              # 표시 크기

SCORE_THR = 0.5            # 이미지 판정 경계 (PostProcessor 캘리브레이션 기준)
THRESHOLD = 0.38           # 픽셀 판정선 (실물 튜닝 확정값, 실행 중 [ ] 로 조절)
THRESH_STEP = 0.02
MIN_AREA = 80              # 작은 노이즈 contour 제거

# 표시 전용 대비 창 (판정에는 사용하지 않음. 고정 상수라 절대 스케일 유지)
DISP_LO = 0.30             # 이 값 이하 -> 진한 파랑
DISP_HI = 0.45             # 이 값 이상 -> 진한 빨강

os.makedirs(SAVE_DIR, exist_ok=True)


def find_ckpt():
    if CKPT_PATH and os.path.exists(CKPT_PATH):
        return CKPT_PATH
    cands = sorted(glob.glob(os.path.join(RESULTS, "**", "*.ckpt"), recursive=True))
    if not cands:
        raise SystemExit(f"[ERR] no .ckpt found under {RESULTS}. run train_patchcore.py first")
    return cands[-1]


# 모델 로드
device = "cuda" if torch.cuda.is_available() else "cpu"
_ckpt = find_ckpt()
print(f"[INFO] using ckpt: {_ckpt}")
try:
    model = Patchcore.load_from_checkpoint(_ckpt)
except Exception as e:
    print(f"[WARN] safe load failed ({type(e).__name__}), retry with weights_only=False")
    _orig_load = torch.load
    torch.load = lambda *a, **kw: _orig_load(*a, **{**kw, "weights_only": False})
    try:
        model = Patchcore.load_from_checkpoint(_ckpt)
    finally:
        torch.load = _orig_load
model.eval().to(device)
print(f"[INFO] model loaded on {device}")

# 웹캠 열기
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQ_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQ_H)
if not cap.isOpened():
    raise SystemExit(f"[ERR] cannot open camera {CAM_INDEX}")

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] camera opened: {w}x{h}")
print("[INFO] [ / ] = pixel threshold,  h = heatmap,  s = save,  q = quit")


def preprocess(bgr):
    """BGR 프레임 -> [0,1] float 텐서.
       채널 순서(BGR->RGB)와 값 범위(/255)만 맞춘다.
       정규화/리사이즈는 모델 내장 PreProcessor가 수행하므로 여기서 하지 않는다."""
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (INPUT_SIZE, INPUT_SIZE)).astype(np.float32) / 255.0
    t = torch.from_numpy(rgb.transpose(2, 0, 1)).unsqueeze(0)
    return t.to(device)


def get_anomaly_map(out):
    """모델 출력에서 anomaly map(HxW)과 이미지 점수를 꺼낸다.
       anomalib 버전에 따라 출력 형태가 달라 방어적으로 처리.
       PostProcessor가 이미 0~1 캘리브레이션했으므로 재정규화하지 않는다."""
    amap, score = None, None
    if isinstance(out, dict):
        amap  = out.get("anomaly_map")
        score = out.get("pred_score")
    else:
        amap  = getattr(out, "anomaly_map", None)
        score = getattr(out, "pred_score", None)
        if amap is None and isinstance(out, (tuple, list)) and len(out) >= 1:
            amap = out[0]

    if amap is None:
        raise RuntimeError(f"cannot find anomaly_map in output: {type(out)}")

    amap = amap.detach().cpu().numpy()
    amap = np.squeeze(amap)                       # (H, W)
    score = float(score) if score is not None else float(amap.max())
    amap = np.clip(amap, 0.0, 1.0)                # 표시 안전용 클램프만
    return amap, score


def crop_square(img):
    """중앙 크롭으로 정사각형 생성 (학습 데이터와 종횡비 정합)."""
    fh, fw = img.shape[:2]
    s = min(fh, fw)
    y0, x0 = (fh - s) // 2, (fw - s) // 2
    return img[y0:y0 + s, x0:x0 + s]


show_heat = True
idx = 0
prev = time.time()

with torch.no_grad():
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[ERR] frame grab failed")
            break

        frame = crop_square(frame)
        frame = cv2.resize(frame, (DISPLAY, DISPLAY))

        # 추론
        out = model(preprocess(frame))
        amap, score = get_anomaly_map(out)
        amap = cv2.resize(amap, (DISPLAY, DISPLAY))

        vis = frame.copy()

        if show_heat:
            # 히트맵: 블러 먼저(피크 보존) -> 고정 창 대비 스트레칭 -> 결함부가 진한 빨강
            disp = cv2.GaussianBlur(amap, (21, 21), 0)
            disp = np.clip((disp - DISP_LO) / (DISP_HI - DISP_LO), 0.0, 1.0)
            heat = cv2.applyColorMap((disp * 255).astype(np.uint8), cv2.COLORMAP_JET)
            vis = cv2.addWeighted(vis, 0.5, heat, 0.5, 0)

        # 픽셀 임계값으로 마스크 생성 -> 결함 부위 외곽선
        mask = (amap > THRESHOLD).astype(np.uint8)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        n_defect = 0
        for c in contours:
            if cv2.contourArea(c) < MIN_AREA:
                continue
            cv2.drawContours(vis, [c], -1, (0, 0, 255), 2)
            n_defect += 1

        # 판정: 이미지 score(분포 밖 물체 대응) OR 픽셀 contour(결함 부위 검출)
        is_ng = (score > SCORE_THR) or (n_defect > 0)
        verdict = "NG" if is_ng else "OK"
        color = (0, 0, 255) if is_ng else (0, 200, 0)

        now = time.time()
        fps = 1.0 / max(1e-6, now - prev)
        prev = now

        cv2.putText(vis, f"{verdict}  score {score:.3f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)
        cv2.putText(vis, f"thr {THRESHOLD:.2f}  areas {n_defect}  FPS {fps:.1f}",
                    (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("PatchCore anomaly ([/] thr, h heat, s save, q quit)", vis)

        k = cv2.waitKey(1) & 0xFF
        if k == ord("q"):
            break
        elif k == ord("["):
            THRESHOLD = max(0.0, THRESHOLD - THRESH_STEP)
            print(f"[THR] {THRESHOLD:.2f}")
        elif k == ord("]"):
            THRESHOLD = min(1.0, THRESHOLD + THRESH_STEP)
            print(f"[THR] {THRESHOLD:.2f}")
        elif k == ord("h"):
            show_heat = not show_heat
        elif k == ord("s"):
            path = os.path.join(SAVE_DIR, f"anom_{idx:03d}.png")
            cv2.imwrite(path, vis)
            print(f"[SAVE] {path}")
            idx += 1

cap.release()
cv2.destroyAllWindows()
print("[DONE]")