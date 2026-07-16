# check_bbox_tuna.py
# tuna SDG 결과의 bbox를 rgb 위에 그려 확인. ok=green, ng=red 로 구분.
# 실행: python check_bbox_tuna.py   (또는 isaac_python)

import os, json
import numpy as np
from PIL import Image, ImageDraw, ImageFont

BASE = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/sdg_raw/"
NUM_FRAMES = 1
COLORS = {"ok": "lime", "ng": "red"}   # 라벨별 색 (없으면 노랑)
FONT_SIZE = 40                          # 라벨 텍스트 크기


def load_font(size):
    """TrueType 폰트 로드. 없으면 기본 폰트로 폴백(작게 나옴)."""
    candidates = [
        "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Bold.ttf",
    ]
    for path in candidates:
        if os.path.exists(path):
            return ImageFont.truetype(path, size)
    print("[WARN] no TrueType font found, using small default font")
    return ImageFont.load_default()


def draw_one(idx, font):
    tag = f"{idx:04d}"
    rgb_path   = os.path.join(BASE, f"rgb_{tag}.png")
    npy_path   = os.path.join(BASE, f"bounding_box_2d_tight_{tag}.npy")
    label_path = os.path.join(BASE, f"bounding_box_2d_tight_labels_{tag}.json")

    if not os.path.exists(rgb_path):
        print(f"[SKIP] no rgb: {rgb_path}")
        return

    img = Image.open(rgb_path).convert("RGB")
    draw = ImageDraw.Draw(img)

    data = np.load(npy_path, allow_pickle=True)
    with open(label_path) as f:
        labels = json.load(f)

    for row in data:
        cls = labels[str(row["semanticId"])]["class"]
        x1, y1, x2, y2 = row["x_min"], row["y_min"], row["x_max"], row["y_max"]
        color = COLORS.get(cls, "yellow")
        draw.rectangle([x1, y1, x2, y2], outline=color, width=4)

        # 라벨 배경(가독성) + 큰 텍스트
        ty = max(0, y1 - FONT_SIZE - 6)
        tw = draw.textlength(cls, font=font)
        draw.rectangle([x1, ty, x1 + tw + 10, ty + FONT_SIZE + 4], fill=color)
        draw.text((x1 + 5, ty), cls, fill="black", font=font)

    out = os.path.join(BASE, f"bbox_result_{tag}.png")
    img.save(out)
    print(f"[SAVE] {out}  (boxes: {len(data)})")


if __name__ == "__main__":
    font = load_font(FONT_SIZE)
    for i in range(NUM_FRAMES):
        draw_one(i, font)
    print("[DONE]")