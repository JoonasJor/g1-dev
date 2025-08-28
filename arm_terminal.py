import time
import numpy as np
import sys
import select
from dataclasses import dataclass

from controller import ArmController, HandController
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from inspire.modbus_data_handler import ModbusDataHandler

import config as Cfg
import g1_joints as Joints
import inspire.joint_mapping

@dataclass
class TestOption:
    name: str
    id: int

option_list = [
    TestOption(name="Hands: close", id=0),         
    TestOption(name="Hands: open", id=1),   
    TestOption(name="Hands: pinch", id=2),    
    TestOption(name="Arm: bend elbows", id=3),   
    TestOption(name="Pose set: cart push", id=4),   
    TestOption(name="Pose set: zero", id=5),   
    TestOption(name="Pose set: default", id=6),  
    TestOption(name="Manual: arms", id=7),
    TestOption(name="Manual: hands", id=8),
    TestOption(name="Test", id=9),
    TestOption(name="Pose set: button push", id=10), 
    TestOption(name="Pose set: palm up", id=11), 
    TestOption(name="Pose set: palm down", id=12), 
    TestOption(name="Pose set: thumbs up", id=13), 
    TestOption(name="Hands: half", id=14), 
    TestOption(name="Print: hand error info", id=15), 
    TestOption(name="Arm: release control", id=16), 
    TestOption(name="Print: high level status", id=17), 
]

class UserInterface:
    def __init__(self):
        self.test_option_ = None

    def convert_to_int(self, input_str):
        try:
            return int(input_str)
        except ValueError:
            return None

    def terminal_handle(self):
        # TODO: remove name
        input_str = input("Enter id or name: \n")

        if input_str == "list":
            self.test_option_.name = None
            self.test_option_.id = None
            for option in option_list:
                print(f"{option.name}, id: {option.id}")
            return

        for option in option_list:
            if input_str == option.name or self.convert_to_int(input_str) == option.id:
                self.test_option_.name = option.name
                self.test_option_.id = option.id
                print(f"Test: {self.test_option_.name}, test_id: {self.test_option_.id}")
                return

        print("No matching test option found.")

def hand_control(angles, hand_r: HandController, hand_l: HandController):
    l_r = input("(L)eft / (R)ight / (B)oth: ")

    if l_r.lower() == "r":
        hand_r.low_cmd_control(
            target_angles=angles,
            duration=1.0,
            wait_for_user_input=False
    )
    elif l_r.lower() == "l":
        hand_l.low_cmd_control(
            target_angles=angles,
            duration=1.0,
            wait_for_user_input=False
    ) 
    elif l_r.lower() == "b":
        hand_r.low_cmd_control(
            target_angles=angles,
            duration=1.0,
            wait_for_user_input=False
        )
        hand_l.low_cmd_control(
            target_angles=angles,
            duration=1.0,
            wait_for_user_input=False
    )     
    else:
        print("Invalid selection.")   

def open_hand(hand_r: HandController, hand_l: HandController):
    angles = [1000] * 6
    hand_control(angles, hand_r, hand_l)

def close_hand_half(hand_r: HandController, hand_l: HandController):
    angles = [500, 500, 500, 500, 100, 700]
    hand_control(angles, hand_r, hand_l)

def close_hand(hand_r: HandController, hand_l: HandController):
    angles = [0, 0, 0, 0, 100, 700]
    hand_control(angles, hand_r, hand_l)

def pinch(hand_r: HandController, hand_l: HandController):
    angles = [1000, 1000, 1000, 300, 300, 0]
    hand_control(angles, hand_r, hand_l)

def bend_elbow(arms: ArmController):
    l_r = input("(L)eft / (R)ight / (B)oth: ")
    angle = float(input("Enter angle in degrees (relative to current): "))

    angles = [motor.q for motor in arms.low_state.motor_state]
    if l_r.lower() == "r":
        angles[Joints.Body.RightElbow.idx] =+ np.deg2rad(angle) 
    elif l_r.lower() == "l":
        angles[Joints.Body.LeftElbow.idx] =+ np.deg2rad(angle)
    elif l_r.lower() == "b":
        angles[Joints.Body.RightElbow.idx] =+ np.deg2rad(angle) 
        angles[Joints.Body.LeftElbow.idx] =+ np.deg2rad(angle)

    arms.low_cmd_control(
        target_angles=angles,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

def pose_cart_push(arms: ArmController):
    angles = [0] * 29
    angles[Joints.Body.LeftElbow.idx] = np.deg2rad(-20)
    angles[Joints.Body.RightElbow.idx] = np.deg2rad(-20)  

    #angles[Joints.Body.LeftShoulderPitch.idx] = np.deg2rad(45)
    #angles[Joints.Body.RightShoulderPitch.idx] = np.deg2rad(-45)

    angles[Joints.Body.LeftWristRoll.idx] = np.deg2rad(90)
    angles[Joints.Body.RightWristRoll.idx] = np.deg2rad(-90)

    arms.low_cmd_control(
        target_angles=angles,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

def pose_zero(arms: ArmController):
    angles = [0] * 29
    arms.low_cmd_control(
        target_angles=angles,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

def pose_default(arms: ArmController):
    angles = [0] * 29
    angles[Joints.Body.LeftElbow.idx] = np.deg2rad(40)
    angles[Joints.Body.RightElbow.idx] = np.deg2rad(40)  
    arms.low_cmd_control(
        target_angles=angles,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

def pose_push_button(arms: ArmController, hand_r: HandController, hand_l: HandController):
    l_r = input("(L)eft / (R)ight: ")
    
    # Negative is up for elbows
    # 0 makes arms point forwards, as opposed to down with BodyController
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

    arms.low_cmd_control(
        target_angles=angles_arm,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

    # Index =   0       1       2       3       4           5
    # Joint =   Pinky   Ring    Middle  Index   Thumb-bend  Thumb-rotation
    # Range =   0-1000

    angles_hand = [0, 0, 0, 1000, 300, 700]
    if l_r.lower() == "r":
        hand_r.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0,
            wait_for_user_input=False
        )
    elif l_r.lower() == "l":
        hand_l.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0,
            wait_for_user_input=False
        )        

def pose_palm_up(arms: ArmController, hand_r: HandController, hand_l: HandController):
    l_r = input("(L)eft / (R)ight: ")
    
    # Negative is up for elbows
    # 0 makes arms point forwards, as opposed to down with BodyController
    angles_arm = [0] * 29
    
    # TODO: make it easier to control specific hands
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

    arms.low_cmd_control(
        target_angles=angles_arm,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

    # Index =   0       1       2       3       4           5
    # Joint =   Pinky   Ring    Middle  Index   Thumb-bend  Thumb-rotation
    # Range =   0-1000

    angles_hand = [900, 900, 900, 900, 900, 900]
    if l_r.lower() == "r":
        hand_r.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0,
            wait_for_user_input=False
        )
    elif l_r.lower() == "l":
        hand_l.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0,
            wait_for_user_input=False
        )    

def pose_palm_down(arms: ArmController, hand_r: HandController, hand_l: HandController):
    l_r = input("(L)eft / (R)ight: ")
    
    # Negative is up for elbows
    # 0 makes arms point forwards, as opposed to down with BodyController
    angles_arm = [0] * 29
    
    # TODO: make it easier to control specific hands
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

    arms.low_cmd_control(
        target_angles=angles_arm,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

    # Index =   0       1       2       3       4           5
    # Joint =   Pinky   Ring    Middle  Index   Thumb-bend  Thumb-rotation
    # Range =   0-1000

    angles_hand = [900, 900, 900, 900, 900, 900]
    if l_r.lower() == "r":
        hand_r.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0,
            wait_for_user_input=False
        )
    elif l_r.lower() == "l":
        hand_l.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0,
            wait_for_user_input=False
        )    

def pose_thumbs_up(arms: ArmController, hand_r: HandController, hand_l: HandController):
    l_r = input("(L)eft / (R)ight: ")
    
    # Negative is up for elbows
    # 0 makes arms point forwards, as opposed to down with BodyController
    angles_arm = [0] * 29
    
    # TODO: make it easier to control specific hands
    # TODO: add pose functions for stuff like this (arm forward, arm up, ...)
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

    arms.low_cmd_control(
        target_angles=angles_arm,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

    # Index =   0       1       2       3       4           5
    # Joint =   Pinky   Ring    Middle  Index   Thumb-bend  Thumb-rotation
    # Range =   0-1000

    angles_hand = [0, 0, 0, 0, 1000, 1000]
    if l_r.lower() == "r":
        hand_r.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0,
            wait_for_user_input=False
        )
    elif l_r.lower() == "l":
        hand_l.low_cmd_control(
            target_angles=angles_hand,
            duration=1.0,
            wait_for_user_input=False
        )    

def manual_arm(arms: ArmController):
    for joint in arms.arm_joints:
        print(joint.idx, joint)

    idx = int(input("Joint to move (index): "))
    angle = float(input("Enter angle in degrees (relative to current): "))

    angles = [motor.q for motor in arms.low_state.motor_state]
    angles[idx] =+ np.deg2rad(angle) 

    arms.low_cmd_control(
        target_angles=angles,
        kp=arms.kp,
        kd=arms.kd,
        duration=2.0,
        wait_for_user_input=False
    )

def manual_hands(hand_r: HandController, hand_l: HandController):
    #TODO: make joint map easier to use outside of mujoco
    joint_map = inspire.joint_mapping.get_joint_mapping()
    inspire_joint_names = ["Pinky", "Ring", "Middle", "Index", "Thumb-bend", "Thumb-rotation"]
    for i, joint_pair in enumerate(joint_map):
        print(f"{i} {inspire_joint_names[i]} ({joint_pair[0].name}, {joint_pair[1].name})")

    idx = int(input("Joint to move (index): "))
    #TODO: add conversion from radian to scaled in HandController outside of mujoco
    angle = int(input("Enter angle - 1000=open 0=closed : "))

    # TODO: fix redundant code (hand_control())
    print(f"{hand_r.low_state.angle_act=}")
    l_r = input("(L)eft / (R)ight / (B)oth: ")
    if l_r.lower() == "r" or l_r.lower() == "b":
        angles = hand_r.low_state.angle_act
        angles[idx] = angle
        print(f"{angles=}")
        hand_r.low_cmd_control(
            target_angles=angles,
            duration=1.0,
            wait_for_user_input=False
    )
    if l_r.lower() == "l" or l_r.lower() == "b":
        angles = hand_l.low_state.angle_act
        angles[idx] = angle
        hand_l.low_cmd_control(
            target_angles=angles,
            duration=1.0,
            wait_for_user_input=False
    )

def print_hand_errors(hand_r: HandController, hand_l: HandController):
    error_bits = {
        0: "Locked-rotor error",
        1: "Over temperature error",
        2: "Overcurrent error",
        3: "Abnormal operation of the motor",
        4: "Communication error"
    }
    #TODO: move joint names to joint joint_mapping.py
    joint_names = ["Pinky", "Ring", "Middle", "Index", "Thumb-bend", "Thumb-rotation"]

    l_r = input("(L)eft / (R)ight: ")
    if l_r.lower() == "r":
        hand = hand_r
    elif l_r.lower() == "l":
        hand = hand_l
    
    while True:
        for i, error_code in enumerate(hand.low_state.err):
            print(f"{joint_names[i]} - Raw Error Code: 0x{error_code:02X} ({error_code:08b})")
            
            for bit in range(5):
                if error_code & (1 << bit):
                    print(f"  - {error_bits[bit]}")
        print()
        print("Press Enter to stop...")

        time.sleep(0.1)

        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            input()
            break 

def print_high_state(arms: ArmController):
    while True:
        state = arms.high_state

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

def sequence_transport(arms: ArmController):
    pass

def test(arms: ArmController):
    from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
    from controller import BodyController

    sport_client = LocoClient()  
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    body_controller = BodyController()
    body_controller.init_msc()

    sport_client.StandUp2Squat()

    input("Press Enter to lock joints...")

    body_controller.hold_angles()

def main():
    # arms freak out when trying to go over physical joint limits
    # TODO: fix^ clamp values in ArmController. grab joint ranges from mujoco model

    # arms sag temporarily when starting a movement
    # move joint x degrees and then -x degrees -> joint doesnt end up at same position
    # TODO: fix^ try increasing kp? + clamp kp if it already isnt being clamped

    arm_controller = ArmController()
    hand_r = HandController("r")
    hand_l = HandController("l")

    while arm_controller.low_state is None:
        print(f"Waiting for low_state, {arm_controller.low_state=}")
        time.sleep(1)
    time.sleep(2) 
    
    test_option = TestOption(name=None, id=None) 
    user_interface = UserInterface()
    user_interface.test_option_ = test_option

    print("Input \"list\" to list all test option ...")
    while True:
        user_interface.terminal_handle()

        print(f"Updated Test Option: Name = {test_option.name}, ID = {test_option.id}")

        if test_option.id == 0:
            close_hand(hand_r, hand_l)

        elif test_option.id == 1:
            open_hand(hand_r, hand_l)

        elif test_option.id == 2:
            pinch(hand_r, hand_l)

        elif test_option.id == 3:
            bend_elbow(arm_controller)

        elif test_option.id == 4:
            pose_cart_push(arm_controller)

        elif test_option.id == 5:
            pose_zero(arm_controller)

        elif test_option.id == 6:
            pose_default(arm_controller)

        elif test_option.id == 7:
            manual_arm(arm_controller)

        elif test_option.id == 8:
            manual_hands(hand_r, hand_l)

        elif test_option.id == 9:
            test(arm_controller, hand_r, hand_l)

        elif test_option.id == 10:
            pose_push_button(arm_controller, hand_r, hand_l)

        elif test_option.id == 11:
            pose_palm_up(arm_controller, hand_r, hand_l)

        elif test_option.id == 12:
            pose_palm_down(arm_controller, hand_r, hand_l)

        elif test_option.id == 13:
            pose_thumbs_up(arm_controller, hand_r, hand_l)

        elif test_option.id == 14:
            close_hand_half(hand_r, hand_l)

        elif test_option.id == 15:
            print_hand_errors(hand_r, hand_l)

        elif test_option.id == 16:
            arm_controller.release_control()

        elif test_option.id == 17:
            print_high_state(arm_controller)

        time.sleep(1)

if __name__ == '__main__':
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
        modbus_r = ModbusDataHandler(ip="192.168.123.211", device_id=1, l_r="r")
        modbus_l = ModbusDataHandler(ip="192.168.123.210", device_id=2, l_r="l")
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    main()