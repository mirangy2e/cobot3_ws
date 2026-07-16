# train_patchcore.py
# anomalib PatchCore로 참치캔 이상탐지 학습.
# 정상(train/good)만 학습하고, test에서 정상/결함 구분 성능을 평가한다.
#
# 설치: pip install anomalib
# 실행: python3 train_patchcore.py

import glob, os

from anomalib.data import Folder
from anomalib.models import Patchcore
from anomalib.engine import Engine

ROOT    = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/dataset/anomaly"
RESULTS = "/home/rokey/cobot3_ws/isaacpjt/tuna_demo/results"

datamodule = Folder(
    name="tuna",
    root=ROOT,
    normal_dir="train/good",        # 학습에 쓰는 정상 (PatchCore는 이것만 학습)
    abnormal_dir="test/defect",     # 평가용 결함
    normal_test_dir="test/good",    # 평가용 정상
    train_batch_size=8,
    eval_batch_size=8,
    num_workers=4,
)

model = Patchcore(
    backbone="wide_resnet50_2",
    layers=["layer2", "layer3"],
    coreset_sampling_ratio=0.1,
)

engine = Engine(max_epochs=1, default_root_dir=RESULTS)

if __name__ == "__main__":
    engine.fit(model=model, datamodule=datamodule)
    results = engine.test(model=model, datamodule=datamodule)
    print(results)

    # 추론에 쓸 체크포인트 경로 출력
    ckpts = glob.glob(os.path.join(RESULTS, "**", "*.ckpt"), recursive=True)
    print("\n[CKPT] 아래 경로를 webcam_anomaly.py 의 CKPT_PATH 에 넣으세요:")
    for c in sorted(ckpts):
        print("   ", c)