# ~/cobot3_ws/isaacpjt/franka/controllers/gripper_controller.py
from isaacsim.robot.manipulators.grippers import ParallelGripper


class GripperController:
    def __init__(self, world):
        self.gripper = ParallelGripper(
            end_effector_prim_path="/World/franka/panda_hand",
            joint_prim_names=["panda_finger_joint1", "panda_finger_joint2"],
            joint_opened_positions=[0.04, 0.04],
            joint_closed_positions=[0.0,  0.0],
        )
        self.gripper.initialize()

    def open(self):
        self.gripper.apply_action(
            self.gripper.forward(action="open")
        )

    def close(self):
        self.gripper.apply_action(
            self.gripper.forward(action="close")
        )