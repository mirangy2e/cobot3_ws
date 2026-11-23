"""이동 방식(motion)의 공통 인터페이스.

pick and place 의 이동 단계(큐브 위로, 하강, 상승 ...)에서 TCP 를 목표까지 옮기는 방법을 정의한다.
PickPlaceController 는 이동 방식이 무엇이든 아래 순서로만 호출한다.
    plan(목표 위치, 목표 자세)  단계를 시작할 때 한 번. 경로를 만들지 못하면 False
    step()                      매 호출마다. 로봇에 보낼 팔 동작을 돌려줌
    done()                      명령을 다 보냈으면 True (실제 도착 여부는 컨트롤러가 TCP 로 따로 확인)
    reset()                     처음 상태로

plan() 이 실패했을 때 컨트롤러의 처리는 on_fail 로 정한다.
    "ik"   : 이번 단계는 ik 방식으로 대신 이동
    "hold" : 제자리에서 기다렸다가 retry_frames 호출 뒤에 다시 plan()
             (장애물을 피하는 방식은 ik 로 대신 가면 장애물로 곧장 가므로 이쪽을 씀)

구현: ik (m0609_motion/ik.py), linear (m0609_motion/trajectory.py), rrt (m0609_motion/rrt.py)
Isaac Sim 에는 "한 번 계획하고 재생하는" 방식의 공통 인터페이스가 없어서 직접 정의했다.
Cortex 의 Commander(명령을 한 번 보내고 매 주기 처리) 와 같은 구조다.
"""

from abc import ABC, abstractmethod

import numpy as np
from isaacsim.core.utils.types import ArticulationAction


class MotionBackend(ABC):
    on_fail = "ik"  # plan() 실패 시 처리 방법: "ik" 또는 "hold"
    retry_frames = 60  # on_fail 이 "hold" 일 때 다시 계획하기까지 기다리는 호출 횟수

    @abstractmethod
    def plan(self, goal_position: np.ndarray, goal_orientation: np.ndarray) -> bool:
        raise NotImplementedError

    @abstractmethod
    def step(self) -> ArticulationAction:
        raise NotImplementedError

    @abstractmethod
    def done(self) -> bool:
        raise NotImplementedError

    def reset(self) -> None:
        return
