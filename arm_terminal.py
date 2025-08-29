import time
import numpy as np
import sys
import select

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from inspire.modbus_data_handler import ModbusDataHandler
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

from controller import BodyController, ArmController, HandController
import g1_joints as Joints
import inspire.joint_mapping

class UserInterface:
    def __init__(self, option_menu):
        self.test_option_idx = None
        self.option_menu = option_menu

    def terminal_handle(self):
        input_str = input("Enter option index (or 'list'): \n")
        if input_str == "list":
            self.test_option_idx = None
            for idx, (name, _) in enumerate(self.option_menu):
                print(f"{idx:<3} {name}")
            return
        try:
            idx = int(input_str)
            if 0 <= idx < len(self.option_menu):
                self.test_option_idx = idx
                print(f"Test: {self.option_menu[idx][0]}, index: {idx}")
                return
        except ValueError:
            pass
        print("No matching test option found.")

class ControlSuite:
    def __init__(self):
        self.arms = ArmController()

        self.hand_r = HandController("r")
        self.hand_l = HandController("l")

        self.body = BodyController()
        self.body.init_msc()

    def wait_for_low_state(self):
        while self.arms.low_state is None:
            print(f"Waiting for low_state, {self.arms.low_state=}")
            time.sleep(1)
        time.sleep(2)

#region Hand Control
    def hand_control(self, angles):
        """
        Controls all joints of one or both hands.
        """
        
        l_r = input("(L)eft / (R)ight / (B)oth: ")

        if l_r.lower() == "r":
            self.hand_r.low_cmd_control(target_angles=angles, duration=1.0)
        elif l_r.lower() == "l":
            self.hand_l.low_cmd_control(target_angles=angles, duration=1.0)
        elif l_r.lower() == "b":
            self.hand_r.low_cmd_control(target_angles=angles, duration=1.0)
            self.hand_l.low_cmd_control(target_angles=angles, duration=1.0)
        else:
            print("Invalid selection.")

    def hand_control_single(self, angle, idx):
        """
        Controls a specified joint of one or both hands.
        """
        
        l_r = input("(L)eft / (R)ight / (B)oth: ")

        if l_r.lower() not in "lrb":
            print("Invalid selection.")
            return

        if l_r.lower() == "r" or l_r.lower() == "b":
            angles = self.hand_r.low_state.angle_act
            angles[idx] = angle
            self.hand_r.low_cmd_control(target_angles=angles, duration=1.0)
        if l_r.lower() == "l" or l_r.lower() == "b":
            angles = self.hand_l.low_state.angle_act
            angles[idx] = angle
            self.hand_l.low_cmd_control(target_angles=angles, duration=1.0)

    def hands_open(self):
        angles = [1000] * 6
        self.hand_control(angles)

    def hands_halfway(self):
        angles = [500, 500, 500, 500, 100, 700]
        self.hand_control(angles)

    def hands_close(self):
        angles = [0, 0, 0, 0, 100, 700]
        self.hand_control(angles)

    def hands_pinch(self):
        angles = [1000, 1000, 1000, 300, 300, 0]
        self.hand_control(angles)

    def hands_loop(self):
        while True:
            print("Press Enter to stop...")
            print()

            print("Closing right hand...")
            self.hand_r.low_cmd_control(target_angles=[0] * 6, duration=1.0)

            print("Closing left hand...")
            self.hand_l.low_cmd_control(target_angles=[0] * 6, duration=1.0)

            print("Opening right hand...")
            self.hand_r.low_cmd_control(target_angles=[1000] * 6, duration=1.0)

            print("Opening left hand...")
            self.hand_l.low_cmd_control(target_angles=[1000] * 6, duration=1.0)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break

    def hands_manual(self):
        #TODO: make joint map easier to use outside of mujoco
        #TODO: add conversion from radian to scaled in HandController outside of mujoco

        joint_map = inspire.joint_mapping.get_joint_mapping()
        inspire_joint_names = inspire.joint_mapping.NAMES

        for i, joint_pair in enumerate(joint_map):
            print(f"{i} {inspire_joint_names[i]} ({joint_pair[0].name}, {joint_pair[1].name})")

        idx = int(input("Joint to move (-1 for all): "))
        angle = int(input("Enter angle - 1000=open 0=closed: "))

        if idx == -1:
            self.hand_control([angle] * 6)
        else:
            self.hand_control_single(angle, idx)
#endregion

#region Arm Control
    def arm_zero(self):
        angles = [0] * 29

        self.arms.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

    def arm_default(self):
        angles = [0] * 29
        angles[Joints.Body.LeftElbow.idx] = np.deg2rad(50)
        angles[Joints.Body.RightElbow.idx] = np.deg2rad(50)
        angles[Joints.Body.LeftShoulderRoll.idx] = np.deg2rad(30)
        angles[Joints.Body.RightShoulderRoll.idx] = np.deg2rad(-30)

        self.arms.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

    def arm_bend_elbow(self):
        l_r = input("(L)eft / (R)ight / (B)oth: ")
        angle = float(input("Enter angle in degrees (relative to current): "))

        angles = [motor.q for motor in self.arms.low_state.motor_state]

        if l_r.lower() == "r":
            angles[Joints.Body.RightElbow.idx] =+ np.deg2rad(angle)
        elif l_r.lower() == "l":
            angles[Joints.Body.LeftElbow.idx] =+ np.deg2rad(angle)
        elif l_r.lower() == "b":
            angles[Joints.Body.RightElbow.idx] =+ np.deg2rad(angle)
            angles[Joints.Body.LeftElbow.idx] =+ np.deg2rad(angle)

        self.arms.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

    def arm_release_control(self):
        self.arms.release_control()

    def arm_manual_relative(self):
        for joint in self.arms.arm_joints:
            print(joint.idx, joint)

        idx = int(input("Joint to move (index): "))
        
        angles = [motor.q for motor in self.arms.low_state.motor_state]
        angle = float(input("Enter angle in degrees (relative to current): "))
        angles[idx] += np.deg2rad(angle)

        self.arms.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

    def arm_manual_absolute(self):
        for joint in self.arms.arm_joints:
            print(joint.idx, joint)

        idx = int(input("Joint to move (index): "))
        
        angles = [motor.q for motor in self.arms.low_state.motor_state]
        angle = float(input("Enter angle in degrees (absolute): "))
        angles[idx] = np.deg2rad(angle)

        self.arms.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

#endregion

#region Body Control
    def body_zero(self):
        angles = [0] * 29

        self.body.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

    def body_default(self):
        angles = Joints.Body.default_angles_list()

        self.body.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

    def body_lock_joints(self):
        #self.body.lock_joints_mode()
        self.body.lock_joints_write_current()
        #self.body.lock_joints_both()

    def body_manual_relative(self):
        for joint in Joints.Body:
            print(joint.idx, joint.name)

        idx = int(input("Joint to move (index): "))

        angle = float(input("Enter angle in degrees (relative to current): "))
        angles = [motor.q for motor in self.body.low_state.motor_state]
        angles[idx] += np.deg2rad(angle)

        self.body.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

    def body_manual_absolute(self):
        for joint in Joints.Body:
            print(joint.idx, joint.name)

        idx = int(input("Joint to move (index): "))

        angle = float(input("Enter angle in degrees (absolute): "))
        angles = [motor.q for motor in self.body.low_state.motor_state]
        angles[idx] = np.deg2rad(angle)

        self.body.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )
#endregion

#region Pose Set 
# (Arms + Hands)
    def pose_cart_push(self):
        angles = [0] * 29
        angles[Joints.Body.LeftElbow.idx] = np.deg2rad(-20)
        angles[Joints.Body.RightElbow.idx] = np.deg2rad(-20)
        angles[Joints.Body.LeftWristRoll.idx] = np.deg2rad(90)
        angles[Joints.Body.RightWristRoll.idx] = np.deg2rad(-90)

        self.arms.low_cmd_control(
            target_angles=angles,
            duration=2.0
        )

        angles_hand = [200] * 6
        self.hand_r.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0
        )
        self.hand_l.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0
        )

    def pose_push_button(self):
        l_r = input("(L)eft / (R)ight: ")

        angles_arm = [0] * 29
        if l_r.lower() == "r":
            angles_arm[Joints.Body.LeftElbow.idx] = np.deg2rad(40)
            angles_arm[Joints.Body.RightElbow.idx] = np.deg2rad(20)
            angles_arm[Joints.Body.RightShoulderPitch.idx] = np.deg2rad(-60)
        elif l_r.lower() == "l":
            angles_arm[Joints.Body.RightElbow.idx] = np.deg2rad(40)
            angles_arm[Joints.Body.LeftElbow.idx] = np.deg2rad(20)
            angles_arm[Joints.Body.LeftShoulderPitch.idx] = np.deg2rad(-60)
        else:
            print("Invalid selection.")
            return
        
        self.arms.low_cmd_control(
            target_angles=angles_arm,
            duration=2.0
        )

        angles_hand = [0, 0, 0, 1000, 300, 700]
        if l_r.lower() == "r":
            self.hand_r.low_cmd_control(
                target_angles=angles_hand,
                duration=1.0
            )
        elif l_r.lower() == "l":
            self.hand_l.low_cmd_control(
                target_angles=angles_hand,
                duration=1.0
            )

    def pose_palm_up(self):
        l_r = input("(L)eft / (R)ight: ")
        angles_arm = [0] * 29

        if l_r.lower() == "r":
            angles_arm[Joints.Body.LeftElbow.idx] = np.deg2rad(40)
            angles_arm[Joints.Body.RightElbow.idx] = np.deg2rad(20)
            angles_arm[Joints.Body.RightShoulderPitch.idx] = np.deg2rad(-40)
            angles_arm[Joints.Body.RightWristRoll.idx] = np.deg2rad(90)
        if l_r.lower() == "l":
            angles_arm[Joints.Body.RightElbow.idx] = np.deg2rad(40)
            angles_arm[Joints.Body.LeftElbow.idx] = np.deg2rad(20)
            angles_arm[Joints.Body.LeftShoulderPitch.idx] = np.deg2rad(-40)
            angles_arm[Joints.Body.LeftWristRoll.idx] = np.deg2rad(-90)

        self.arms.low_cmd_control(
            target_angles=angles_arm,
            duration=2.0
        )
        angles_hand = [900, 900, 900, 900, 900, 900]
        if l_r.lower() == "r":
            self.hand_r.low_cmd_control(
                target_angles=angles_hand,
                duration=1.0
            )
        elif l_r.lower() == "l":
            self.hand_l.low_cmd_control(
                target_angles=angles_hand,
                duration=1.0
            )

    def pose_palm_down(self):
        l_r = input("(L)eft / (R)ight: ")

        angles_arm = [0] * 29
        if l_r.lower() == "r":
            angles_arm[Joints.Body.LeftElbow.idx] = np.deg2rad(40)
            angles_arm[Joints.Body.RightElbow.idx] = np.deg2rad(20)
            angles_arm[Joints.Body.RightShoulderPitch.idx] = np.deg2rad(-40)
            angles_arm[Joints.Body.RightWristRoll.idx] = np.deg2rad(-90)
        if l_r.lower() == "l":
            angles_arm[Joints.Body.RightElbow.idx] = np.deg2rad(40)
            angles_arm[Joints.Body.LeftElbow.idx] = np.deg2rad(20)
            angles_arm[Joints.Body.LeftShoulderPitch.idx] = np.deg2rad(-40)
            angles_arm[Joints.Body.LeftWristRoll.idx] = np.deg2rad(90)

        self.arms.low_cmd_control(
            target_angles=angles_arm,
            duration=2.0
        )
        angles_hand = [900, 900, 900, 900, 900, 900]
        if l_r.lower() == "r":
            self.hand_r.low_cmd_control(
                target_angles=angles_hand,
                duration=1.0
            )
        elif l_r.lower() == "l":
            self.hand_l.low_cmd_control(
                target_angles=angles_hand,
                duration=1.0
            )

    def pose_thumbs_up(self):
        l_r = input("(L)eft / (R)ight: ")
        angles_arm = [0] * 29

        # TODO: make it easier to control specific hands
        if l_r.lower() == "r":
            angles_arm[Joints.Body.LeftElbow.idx] = np.deg2rad(40)
            angles_arm[Joints.Body.RightElbow.idx] = np.deg2rad(15)
            angles_arm[Joints.Body.RightShoulderPitch.idx] = np.deg2rad(-40)
            angles_arm[Joints.Body.RightWristRoll.idx] = np.deg2rad(10)
        if l_r.lower() == "l":
            angles_arm[Joints.Body.RightElbow.idx] = np.deg2rad(40)
            angles_arm[Joints.Body.LeftElbow.idx] = np.deg2rad(15)
            angles_arm[Joints.Body.LeftShoulderPitch.idx] = np.deg2rad(-40)
            angles_arm[Joints.Body.RightWristRoll.idx] = np.deg2rad(-10)

        self.arms.low_cmd_control(
            target_angles=angles_arm,
            duration=2.0
        )
        angles_hand = [0, 0, 0, 0, 1000, 1000]
        if l_r.lower() == "r":
            self.hand_r.low_cmd_control(
                target_angles=angles_hand,
                duration=1.0
            )
        elif l_r.lower() == "l":
            self.hand_l.low_cmd_control(
                target_angles=angles_hand,
                duration=1.0
            )
#endregion

#region Misc
    def test(self):
        sport_client = LocoClient()
        sport_client.SetTimeout(10.0)
        sport_client.Init()

        sport_client.StandUp2Squat()

        input("Press Enter to lock joints...")

        #body_controller.lock_joints_mode()
        self.body.lock_joints_write_current()
        #body_controller.lock_joints_both()

    def print_hand_errors(self):
        error_bits = {
            0: "Locked-rotor error",
            1: "Over temperature error",
            2: "Overcurrent error",
            3: "Abnormal operation of the motor",
            4: "Communication error"
        }

        inspire_joint_names = inspire.joint_mapping.NAMES

        l_r = input("(L)eft / (R)ight: ")
        if l_r.lower() == "r":
            hand = self.hand_r
        elif l_r.lower() == "l":
            hand = self.hand_l

        while True:
            for i, error_code in enumerate(hand.low_state.err):
                print(f"{inspire_joint_names[i]} - Raw Error Code: 0x{error_code:02X} ({error_code:08b})")
                for bit in range(5):
                    if error_code & (1 << bit):
                        print(f"  - {error_bits[bit]}")
            print()
            print("Press Enter to stop...")

            time.sleep(0.1)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break

    def print_high_state(self):
        while True:
            state = self.arms.high_state
            print("SportModeState_ Summary:")
            print(f"  Timestamp: {state.stamp}")
            print(f"  Error Code: {state.error_code}")
            print(f"  IMU State: {state.imu_state}")
            print(f"  Mode: {state.mode}")
            print(f"  Progress: {state.progress}")
            print(f"  Gait Type: {state.gait_type}")
            print(f"  Foot Raise Height: {state.foot_raise_height}")
            print(f"  Position: {state.position}")
            print(f"  Body Height: {state.body_height}")
            print(f"  Velocity: {state.velocity}")
            print(f"  Yaw Speed: {state.yaw_speed}")
            print(f"  Range Obstacle: {state.range_obstacle}")
            print(f"  Foot Force: {state.foot_force}")
            print(f"  Foot Position (Body): {state.foot_position_body}")
            print(f"  Foot Speed (Body): {state.foot_speed_body}")
            print(f"  Path Points:")

            for i, point in enumerate(state.path_point):
                print(f"    [{i}] {point}")
            print()
            print("Press Enter to stop...")

            time.sleep(0.1)

            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                input()
                break
#endregion

if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
        modbus_r = ModbusDataHandler(ip="192.168.123.211", device_id=1, l_r="r")
        modbus_l = ModbusDataHandler(ip="192.168.123.210", device_id=2, l_r="l")
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    suite = ControlSuite()
    suite.wait_for_low_state()

    option_menu = [
        ("Hands: open", suite.hands_open),
        ("Hands: halfway", suite.hands_halfway),
        ("Hands: close", suite.hands_close),
        ("Hands: pinch", suite.hands_pinch),
        ("Hands: loop", suite.hands_loop),
        ("Hands: manual", suite.hands_manual),

        ("Arm: zero", suite.arm_zero),
        ("Arm: default", suite.arm_default),
        ("Arm: bend elbow", suite.arm_bend_elbow),
        ("Arm: release control", suite.arm_release_control),
        ("Arm: manual (relative)", suite.arm_manual_relative),
        ("Arm: manual (absolute)", suite.arm_manual_absolute),

        ("Body: zero", suite.body_zero),
        ("Body: default", suite.body_default),
        ("Body: lock joints", suite.body_lock_joints),
        ("Body: manual (relative)", suite.body_manual_relative),
        ("Body: manual (absolute)", suite.body_manual_absolute),

        ("Pose set: cart push", suite.pose_cart_push),
        ("Pose set: button push", suite.pose_push_button),
        ("Pose set: palm up", suite.pose_palm_up),
        ("Pose set: palm down", suite.pose_palm_down),
        ("Pose set: thumbs up", suite.pose_thumbs_up),
        
        ("Test", suite.test),
        ("Print: hand error info", suite.print_hand_errors),
        ("Print: high level status", suite.print_high_state),
    ]
    user_interface = UserInterface(option_menu)
    while True:
        user_interface.terminal_handle()
        idx = user_interface.test_option_idx

        if idx is not None and 0 <= idx < len(option_menu):
            option_menu[idx][1]()

        time.sleep(1)