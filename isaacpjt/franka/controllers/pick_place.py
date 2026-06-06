# ~/cobot3_ws/isaacpjt/franka/controllers/pick_place.py
import numpy as np


class PickPlaceTask:
    # Pick & Place 높이 설정 (world 기준, 단위: 미터)
    APPROACH_Z = 0.20   # 바나나 위 접근 높이
    PICK_Z     = 0.05   # 바나나 집는 높이
    LIFT_Z     = 0.30   # 들어올리는 높이

    def __init__(self, controller, gripper, bridge):
        self.controller = controller
        self.gripper    = gripper
        self.bridge     = bridge
        self.state      = "IDLE"
        self.target     = None
        self._wait      = 0

    def update(self):
        # 새 목표 수신 시 IDLE → OPEN_GRIPPER 전환
        if self.bridge.new_target and self.state == "IDLE":
            self.target          = self.bridge.target_position.copy()
            self.bridge.new_target = False
            self.state           = "OPEN_GRIPPER"
            print(f"[PickPlace] New target: {self.target}")

        if self.state == "OPEN_GRIPPER":
            self.gripper.open()
            self.state = "APPROACH"
            print("[PickPlace] OPEN_GRIPPER → APPROACH")

        elif self.state == "APPROACH":
            approach = self.target.copy()
            approach[2] = self.APPROACH_Z
            self.controller.move_to(approach)
            if self.controller.is_reached(approach):
                self.state = "PICK"
                print("[PickPlace] APPROACH → PICK")

        elif self.state == "PICK":
            pick = self.target.copy()
            pick[2] = self.PICK_Z
            self.controller.move_to(pick)
            if self.controller.is_reached(pick):
                self.state  = "CLOSE_GRIPPER"
                self._wait  = 0
                print("[PickPlace] PICK → CLOSE_GRIPPER")

        elif self.state == "CLOSE_GRIPPER":
            self.gripper.close()
            self._wait += 1
            if self._wait > 60:   # 60 프레임 대기
                self.state = "LIFT"
                print("[PickPlace] CLOSE_GRIPPER → LIFT")

        elif self.state == "LIFT":
            lift = self.target.copy()
            lift[2] = self.LIFT_Z
            self.controller.move_to(lift)
            if self.controller.is_reached(lift):
                self.state = "IDLE"
                print("[PickPlace] LIFT → IDLE (완료)")

        # 매 프레임 RmpFlow 업데이트
        if self.state != "IDLE":
            self.controller.update()