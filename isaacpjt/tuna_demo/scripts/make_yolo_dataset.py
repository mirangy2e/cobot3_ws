# make_yolo_dataset.py
# BasicWriter 출력(sdg_raw: rgb + bbox npy + labels json)
#   -> YOLO11 검출 데이터셋(images/labels + data.yaml)으로 변환.
# 실행: python make_yolo_dataset.py

import os, json, glob, shutil, random
import numpy as np
from PIL import Image

SRC = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/sdg_raw"
DST = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/yolo"

CLASS_MAP = {"ok": 0, "ng": 1}   # data.yaml names 순서와 일치해야 함
NAMES     = ["ok", "ng"]
VAL_RATIO = 0.2                   # 검증 비율
SEED      = 42


def convert_frame(tag):
    """한 프레임: npy(픽셀 bbox) + labels(클래스) -> YOLO 라벨 라인 리스트."""
    rgb   = os.path.join(SRC, f"rgb_{tag}.png")
    npy   = os.path.join(SRC, f"bounding_box_2d_tight_{tag}.npy")
    ljson = os.path.join(SRC, f"bounding_box_2d_tight_labels_{tag}.json")
    if not (os.path.exists(rgb) and os.path.exists(npy) and os.path.exists(ljson)):
        return None, None

    with Image.open(rgb) as im:
        W, H = im.size

    data = np.load(npy, allow_pickle=True)
    with open(ljson) as f:
        labels = json.load(f)

    lines = []
    for row in data:
        cls_name = labels[str(row["semanticId"])]["class"]
        if cls_name not in CLASS_MAP:
            continue
        cid = CLASS_MAP[cls_name]
        x1, y1 = float(row["x_min"]), float(row["y_min"])
        x2, y2 = float(row["x_max"]), float(row["y_max"])
        # 이미지 경계로 클램프
        x1 = max(0.0, min(x1, W)); x2 = max(0.0, min(x2, W))
        y1 = max(0.0, min(y1, H)); y2 = max(0.0, min(y2, H))
        bw, bh = x2 - x1, y2 - y1
        if bw <= 1 or bh <= 1:
            continue                       # degenerate box 제거
        # YOLO: class cx cy w h (정규화)
        cx = (x1 + x2) / 2.0 / W
        cy = (y1 + y2) / 2.0 / H
        nw = bw / W
        nh = bh / H
        lines.append(f"{cid} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}")
    return rgb, lines


def main():
    # 기존 데이터셋 폴더가 있으면 통째로 삭제 후 시작 (이전 결과 섞임 방지)
    if os.path.isdir(DST):
        shutil.rmtree(DST)
        print(f"[CLEAN] removed existing {DST}")
    os.makedirs(DST, exist_ok=True)

    tags = sorted(os.path.basename(p)[4:-4]           # "rgb_0007.png" -> "0007"
                  for p in glob.glob(os.path.join(SRC, "rgb_*.png")))
    if not tags:
        print(f"[ERR] no rgb_*.png in {SRC}")
        return

    random.seed(SEED)
    random.shuffle(tags)
    n_val = max(1, int(len(tags) * VAL_RATIO))
    val_set = set(tags[:n_val])

    for split in ("train", "val"):
        os.makedirs(os.path.join(DST, "images", split), exist_ok=True)
        os.makedirs(os.path.join(DST, "labels", split), exist_ok=True)

    n_ok, n_box = 0, 0
    for tag in tags:
        split = "val" if tag in val_set else "train"
        rgb, lines = convert_frame(tag)
        if rgb is None:
            continue
        shutil.copy(rgb, os.path.join(DST, "images", split, f"tuna_{tag}.png"))
        with open(os.path.join(DST, "labels", split, f"tuna_{tag}.txt"), "w") as f:
            f.write("\n".join(lines))       # 박스 없으면 빈 파일(=background)
        n_ok += 1
        n_box += len(lines)

    yaml_path = os.path.join(DST, "data.yaml")
    with open(yaml_path, "w") as f:
        f.write(f"path: {DST}\n")
        f.write("train: images/train\n")
        f.write("val: images/val\n")
        f.write(f"nc: {len(NAMES)}\n")
        f.write(f"names: {NAMES}\n")

    print(f"[DONE] frames={n_ok}, boxes={n_box}")
    print(f"  train={len(tags)-len(val_set)}, val={len(val_set)}")
    print(f"  yaml: {yaml_path}")


if __name__ == "__main__":
    main()