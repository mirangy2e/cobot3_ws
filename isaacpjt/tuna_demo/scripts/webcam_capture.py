# webcam_capture.py
# USB 웹캠 실시간 보기 + 프레임 저장 (Ubuntu).
# 실행: python3 webcam_capture.py
# 조작: s = 현재 프레임 저장,  c = 크롭 on/off,  q = 종료
#
# 목적: 실제 웹캠에 캔(정상/찌그러짐)이 어떻게 보이는지 확인 -> sim 렌더와 비교해
#       카메라 각도/거리/배경/조명, 결함의 실제 가시성을 맞춤.
#
# 캡처 사이즈 조절:
#   - 웹캠은 정사각형 모드를 지원하지 않음(1280x1280 요청 -> 1280x960 으로 열림)
#   - 따라서 [중앙 정사각형 크롭] 후 [SAVE_SIZE 로 리사이즈]해 원하는 크기로 저장한다.
#   - sim 학습 데이터(512x512)와 맞추려면 SAVE_SIZE = 512

import cv2, os

CAM_INDEX = 4                 # ABKO QHD (/dev/video4). 안 열리면 5 또는 0
REQ_W, REQ_H = 2560, 1440     # 웹캠이 지원하는 모드 (MJPG로 30fps)
                              #   다른 옵션: 1920x1080, 1600x1200, 1280x960, 640x480

CROP_SQUARE  = True           # 중앙 정사각형 크롭 (c 키로 토글)
SAVE_SIZE    = 512            # 저장 크기 (정사각형). None 이면 크롭 원본 크기 그대로
DISPLAY_SIZE = 720            # 화면 표시 크기

SAVE_DIR = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/real_ref"

os.makedirs(SAVE_DIR, exist_ok=True)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_V4L2)   # Ubuntu는 V4L2 백엔드가 안정적
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))   # 고해상도 30fps 확보
cap.set(cv2.CAP_PROP_FRAME_WIDTH, REQ_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQ_H)

if not cap.isOpened():
    print(f"[ERR] cannot open camera index {CAM_INDEX}. try 5/0 or check /dev/video*")
    raise SystemExit

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] camera opened: {w}x{h}")
print(f"[INFO] crop={CROP_SQUARE}, save_size={SAVE_SIZE}")
print("[INFO] s = save,  c = crop on/off,  q = quit")


def crop_square(img):
    """중앙을 정사각형으로 크롭."""
    fh, fw = img.shape[:2]
    s = min(fh, fw)
    y0, x0 = (fh - s) // 2, (fw - s) // 2
    return img[y0:y0 + s, x0:x0 + s]


idx = 0
while True:
    ok, frame = cap.read()
    if not ok:
        print("[ERR] frame grab failed")
        break

    out = crop_square(frame) if CROP_SQUARE else frame

    # 저장용 크기로 리사이즈
    if SAVE_SIZE:
        if CROP_SQUARE:
            out = cv2.resize(out, (SAVE_SIZE, SAVE_SIZE))
        else:
            fh, fw = out.shape[:2]
            scale = SAVE_SIZE / max(fh, fw)
            out = cv2.resize(out, (int(fw * scale), int(fh * scale)))

    # 표시 (저장 이미지와 동일한 내용을 보기 좋은 크기로)
    dh, dw = out.shape[:2]
    scale = DISPLAY_SIZE / max(dh, dw)
    disp = cv2.resize(out, (int(dw * scale), int(dh * scale)))
    cv2.putText(disp, f"{out.shape[1]}x{out.shape[0]}  crop={CROP_SQUARE}",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("webcam (s=save, c=crop, q=quit)", disp)

    k = cv2.waitKey(1) & 0xFF
    if k == ord('s'):
        path = os.path.join(SAVE_DIR, f"defect_{idx:03d}.png")
        cv2.imwrite(path, out)
        print(f"[SAVE] {path}  ({out.shape[1]}x{out.shape[0]})")
        idx += 1
    elif k == ord('c'):
        CROP_SQUARE = not CROP_SQUARE
        print(f"[CROP] {CROP_SQUARE}")
    elif k == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"[DONE] saved to {SAVE_DIR}")