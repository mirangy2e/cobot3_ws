# webcam_infer.py
# best.pt(YOLO11)로 실제 웹캠 실시간 추론. Ubuntu + USB 웹캠(ABKO QHD).
# 실행: python webcam_infer.py
# 조작: s = 현재 화면 저장,  q = 종료

import os, time
import cv2
from ultralytics import YOLO

MODEL_PATH = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/runs/detect/tuna_v1/weights/best.pt"  # 실제 경로 확인
CAM_INDEX  = 4          # ABKO QHD (/dev/video4). 안 열리면 5 또는 0
CONF       = 0.6        # 신뢰도 임계값
SAVE_DIR   = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/yolo_infer_out"

# 클래스별 박스 색 (BGR)
COLORS = {"ok": (0, 200, 0), "ng": (0, 0, 255)}

os.makedirs(SAVE_DIR, exist_ok=True)

# 모델 로드
model = YOLO(MODEL_PATH)
names = model.names          # {0: 'ok', 1: 'ng'}
print(f"[INFO] model loaded: {MODEL_PATH}  classes={names}")

# 웹캠 열기 (QHD는 MJPG로 열어야 고해상도/프레임 확보되는 경우 많음)
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)

if not cap.isOpened():
    print(f"[ERR] cannot open camera {CAM_INDEX}. try 5 or 0, check /dev/video*")
    raise SystemExit

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] camera {CAM_INDEX} opened: {w}x{h}")
print("[INFO] s = save,  q = quit")

idx = 0
prev = time.time()
while True:
    ok, frame = cap.read()
    if not ok:
        print("[ERR] frame grab failed")
        break

    # 추론 (imgsz는 학습값에 맞춰 자동 리사이즈)
    results = model(frame, conf=CONF, verbose=False)

    ng_count = 0
    for box in results[0].boxes:
        cls_id = int(box.cls[0])
        conf   = float(box.conf[0])
        x1, y1, x2, y2 = map(int, box.xyxy[0])
        name = names[cls_id]
        color = COLORS.get(name, (0, 255, 255))
        if name == "ng":
            ng_count += 1
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)
        label = f"{name} {conf:.2f}"
        cv2.putText(frame, label, (x1, max(0, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

    # FPS + 불량 개수 표시
    now = time.time()
    fps = 1.0 / max(1e-6, now - prev)
    prev = now
    cv2.putText(frame, f"FPS {fps:.1f}  NG {ng_count}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

    cv2.imshow("YOLO infer (s=save, q=quit)", frame)
    k = cv2.waitKey(1) & 0xFF
    if k == ord("s"):
        path = os.path.join(SAVE_DIR, f"infer_{idx:03d}.png")
        cv2.imwrite(path, frame)
        print(f"[SAVE] {path}")
        idx += 1
    elif k == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
print("[DONE]")