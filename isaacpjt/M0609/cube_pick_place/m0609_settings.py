"""m0609_settings.yaml 을 읽어 모든 값을 이 모듈의 대문자 변수로 제공하는 설정 로더.

사용법
    import m0609_settings as settings
    settings.TARGET_STEP         # m0609_settings.yaml 의 target_step
    changed = settings.reload()  # 매 프레임 호출. 파일이 저장됐을 때만 다시 읽고, 바뀐 값의 이름 목록을 돌려줌

동작
- 숫자 리스트는 numpy 배열로, 파일 경로는 yaml 기준 절대 경로(Path)로 바꾼다.
- STATIC_KEYS(yaml 의 수정 금지 영역)는 시작할 때 한 번만 읽고, 나머지는 실행 중에도 바뀐다.
- 저장한 yaml 에 문법 오류가 있으면 이전 값을 그대로 쓰고 경고만 출력한다.
- 모듈 이름은 다른 라이브러리와 겹치지 않도록 로봇 이름을 붙였다 (config 는 Isaac Sim 의 OpenCV 모듈과 실제로 겹침).
"""

from pathlib import Path

import numpy as np
import yaml

SETTINGS_PATH = Path(__file__).resolve().parent / "m0609_settings.yaml"

# 파일 경로로 바꿀 키
PATH_KEYS = {"robot_usd", "urdf_path", "robot_description_path", "rrt_config_path"}

# yaml 의 [수정 금지 영역] 키: 실행 중에 바뀌어도 적용하지 않는다
STATIC_KEYS = {
    "robot_usd",
    "urdf_path",
    "robot_description_path",
    "rrt_config_path",
    "robot_prim_path",
    "end_effector_prim_path",
    "arm_joints",
    "gripper_joint",
    "tcp_frame",
    "home_joints_deg",
    "branch_joints",
    "cube_size",
    "cube_initial_position",
    "obstacles",
    "obstacle_margin",
    "physics_dt",
    "rendering_dt",
}

_mtime = None  # 마지막으로 읽은 파일의 수정 시각
_values = {}  # 현재 적용 중인 값 (소문자 키)


def _convert(key, value):
    """yaml 값을 코드에서 쓰기 좋은 형태로 바꾼다 (경로 -> Path, 숫자 리스트 -> numpy 배열)."""
    if key in PATH_KEYS:
        return (SETTINGS_PATH.parent / value).resolve()
    if isinstance(value, list) and all(isinstance(v, (int, float)) for v in value):
        return np.array(value, dtype=float)
    return value


def _same(a, b):
    if isinstance(a, np.ndarray) or isinstance(b, np.ndarray):
        return np.array_equal(a, b)
    return a == b


def reload():
    """yaml 이 저장됐으면 다시 읽어 바뀐 값을 적용하고, 바뀐 값의 이름(대문자) 목록을 돌려준다."""
    global _mtime
    mtime = SETTINGS_PATH.stat().st_mtime
    if mtime == _mtime:  # 파일이 그대로면 아무것도 하지 않음 (매 프레임 호출해도 부담 없음)
        return []
    first = _mtime is None
    _mtime = mtime
    try:
        with open(SETTINGS_PATH) as f:
            values = {key: _convert(key, value) for key, value in yaml.safe_load(f).items()}
    except Exception as error:  # 편집 중 오타 등: 이전 값으로 계속 실행
        print(f"[m0609_settings] {SETTINGS_PATH.name} 을 읽지 못해 이전 값을 유지합니다: {error}")
        return []

    changed = []
    for key, value in values.items():
        if key in _values and _same(_values[key], value):
            continue
        if not first and key in STATIC_KEYS:
            print(f"[m0609_settings] {key} 는 수정 금지 영역의 값입니다. 적용하려면 스크립트를 다시 실행하세요")
            continue
        if not first:
            print(f"[m0609_settings] {key}: {_values.get(key)} -> {value}")
        _values[key] = value
        globals()[key.upper()] = value  # settings.<대문자 키> 로 읽을 수 있게 함
        changed.append(key.upper())
    return changed


reload()  # import 할 때 처음 한 번 읽음
