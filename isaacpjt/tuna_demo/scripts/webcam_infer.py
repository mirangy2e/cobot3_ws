# webcam_infer.py
# best.pt(YOLO11)로 실제 웹캠 실시간 추론. Ubuntu + USB 웹캠(ABKO APC925 QHD).
#
# 카메라 설정 근거 (v4l2-ctl -d /dev/video4 --list-formats-ext 로 확인):
#   - MJPG 필수: YUYV로 열면 QHD가 2fps. MJPG면 같은 해상도에서 30fps.
#   - 2560x1440(QHD) @ MJPG = 30fps 지원
#   - 웹캠은 정사각형 모드가 없으므로, 받은 프레임을 중앙 크롭해 정사각형으로 만든다.
#     (학습 데이터가 1280x1280 정사각형이므로 형태를 맞춤)
#
# 실행: python3 webcam_infer.py
# 조작: s = 현재 화면 저장,  q = 종료

import os, time
import cv2
from ultralytics import YOLO

MODEL_PATH = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/runs/detect/tuna_v1/weights/best.pt"
CAM_INDEX  = 4          # ABKO QHD (/dev/video4). 안 열리면 5 또는 0
CONF       = 0.6        # 신뢰도 임계값
SAVE_DIR   = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/yolo_infer_out"

REQ_W, REQ_H = 1280, 960   # QHD (MJPG로 30fps). 부하 크면 1280, 960 으로 낮춰도 됨
DISPLAY_SIZE = 1280          # 화면 표시 크기 (추론은 원본 해상도로 수행)

# 클래스별 박스 색 (BGR)
COLORS = {"ok": (0, 200, 0), "ng": (0, 0, 255)}

os.makedirs(SAVE_DIR, exist_ok=True)

# 모델 로드
model = YOLO(MODEL_PATH)
names = model.names          # {0: 'ok', 1: 'ng'}
print(f"[INFO] model loaded: {MODEL_PATH}  classes={names}")

# 웹캠 열기 (MJPG 먼저 설정한 뒤 해상도 지정)
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQ_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQ_H)

if not cap.isOpened():
    print(f"[ERR] cannot open camera {CAM_INDEX}. try 5 or 0, check /dev/video*")
    raise SystemExit

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] camera {CAM_INDEX} opened: {w}x{h}  -> crop {min(w,h)}x{min(w,h)}  -> show {DISPLAY_SIZE}x{DISPLAY_SIZE}")
print("[INFO] s = save,  q = quit")


def crop_square(img):
    """중앙을 정사각형으로 크롭 (학습 데이터의 정사각형 형태와 일치시킴)."""
    fh, fw = img.shape[:2]
    s = min(fh, fw)
    y0, x0 = (fh - s) // 2, (fw - s) // 2
    return img[y0:y0 + s, x0:x0 + s]


idx = 0
prev = time.time()
while True:
    ok, frame = cap.read()
    if not ok:
        print("[ERR] frame grab failed")
        break

    frame = crop_square(frame)                  # 정사각형으로

    # 추론 (원본 해상도로 수행. imgsz는 학습값에 맞춰 내부에서 자동 리사이즈)
    results = model(frame, conf=CONF, verbose=False)

    ng_count = 0
    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        conf   = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        name  = names[cls_id]
        color = COLORS.get(name, (0, 255, 255))
        if name == "ng":
            ng_count += 1
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 4)
        cv2.putText(frame, f"{name} {conf:.2f}", (x1, max(30, y1 - 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

    # 표시용으로 축소
    disp = cv2.resize(frame, (DISPLAY_SIZE, DISPLAY_SIZE))

    # FPS + 불량 개수
    now = time.time()
    fps = 1.0 / max(1e-6, now - prev)
    prev = now
    cv2.putText(disp, f"FPS {fps:.1f}  NG {ng_count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    cv2.imshow("YOLO infer (s=save, q=quit)", disp)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("s"):
        path = os.path.join(SAVE_DIR, f"infer_{idx:03d}.png")
        cv2.imwrite(path, frame)                # 저장은 원본 해상도로
        print(f"[SAVE] {path}  ({frame.shape[1]}x{frame.shape[0]})")
        idx += 1
    elif k == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
print("[DONE]")