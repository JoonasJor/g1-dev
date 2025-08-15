import mujoco
import numpy as np
import pygame
import sys
import struct
import traceback
import os
import sys

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.utils.thread import RecurrentThread

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config as Cfg
import g1_joints as Joints

if Cfg.ROBOT=="g1":
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default
else:
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
    from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_ as LowState_default

TOPIC_LOWCMD = "rt/lowcmd"
TOPIC_LOWSTATE = "rt/lowstate"
TOPIC_HIGHSTATE = "rt/sportmodestate"
TOPIC_WIRELESS_CONTROLLER = "rt/wirelesscontroller"

MOTOR_SENSOR_NUM = 3
NUM_MOTOR_IDL_GO = 20
NUM_MOTOR_IDL_HG = 35

class UnitreeSdk2Bridge:

    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = Cfg.NUM_MOTOR_BODY + Cfg.NUM_MOTOR_FINGERS
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep
        self.idl_type = (self.num_motor > NUM_MOTOR_IDL_GO) # 0: unitree_go, 1: unitree_hg

        self.joystick = None

        joint_angle_range = []
        for idx in Joints.Body.mujoco_idx_list():
            # offset by 1 to accommodate for (joint_index: 0 , name: floating_base_joint)
            # this offset only applies to worldbody joints. not actuators or sensors.
            joint_angle_range.append(tuple(mj_model.jnt_range[idx + 1]))
        print(joint_angle_range)

        # Check sensor
        for i in range(self.dim_motor_sensor, self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name == "imu_quat":
                self.have_imu_ = True
            if name == "frame_pos":
                self.have_frame_sensor_ = True

        # Unitree sdk2 message
        self.low_state = LowState_default()
        self.low_state_puber = ChannelPublisher(TOPIC_LOWSTATE, LowState_)
        self.low_state_puber.Init()
        self.lowStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishLowState, name="sim_lowstate"
        )
        self.lowStateThread.Start()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_puber = ChannelPublisher(TOPIC_HIGHSTATE, SportModeState_)
        self.high_state_puber.Init()
        self.HighStateThread = RecurrentThread(
            interval=self.dt, target=self.PublishHighState, name="sim_highstate"
        )
        self.HighStateThread.Start()

        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_puber = ChannelPublisher(
            TOPIC_WIRELESS_CONTROLLER, WirelessController_
        )
        self.wireless_controller_puber.Init()
        self.WirelessControllerThread = RecurrentThread(
            interval=0.01,
            target=self.PublishWirelessController,
            name="sim_wireless_controller",
        )
        self.WirelessControllerThread.Start()

        self.low_cmd_suber = ChannelSubscriber(TOPIC_LOWCMD, LowCmd_)
        self.low_cmd_suber.Init(self.LowCmdHandler, 10)

        # joystick
        self.key_map = {
            "R1": 0,
            "L1": 1,
            "start": 2,
            "select": 3,
            "R2": 4,
            "L2": 5,
            "F1": 6,
            "F2": 7,
            "A": 8,
            "B": 9,
            "X": 10,
            "Y": 11,
            "up": 12,
            "right": 13,
            "down": 14,
            "left": 15,
        }

    def LowCmdHandler(self, msg: LowCmd_):
        if self.mj_data is None:
            print("[LowCmdHandler] mj_data is None")
            return
        
        try:
            for cmd_idx, joint_idx in enumerate(Joints.Body.mujoco_idx_list()):
                q_index = joint_idx
                dq_index = joint_idx + self.num_motor

                self.mj_data.ctrl[cmd_idx] = (
                    msg.motor_cmd[cmd_idx].tau
                    + msg.motor_cmd[cmd_idx].kp
                    * (msg.motor_cmd[cmd_idx].q - self.mj_data.sensordata[q_index])
                    + msg.motor_cmd[cmd_idx].kd
                    * (
                        msg.motor_cmd[cmd_idx].dq
                        - self.mj_data.sensordata[dq_index]
                    )
                )
        except Exception as e:
            print(f"[LowCmdHandler] error: {type(e).__name__}: {e}")
            print(f"{joint_idx=}, {q_index=}, {dq_index=}")
            print(f"{len(msg.motor_cmd)=}")
            print(f"{len(self.mj_data.ctrl)=}")
            print(f"{self.num_motor=}")

    def PublishLowState(self):
        try:
            if self.mj_data is None:
                print("[PublishLowState] mj_data is None")
                return

            sensor_len = len(self.mj_data.sensordata)
            expected_len = 3 * self.num_motor
            if sensor_len < expected_len:
                print(f"[PublishLowState] sensordata too short: expected {expected_len}, got {sensor_len}")
                return

            for state_idx, joint_idx in enumerate(Joints.Body.mujoco_idx_list()):
                q_index = joint_idx
                dq_index = joint_idx + self.num_motor
                tau_index = joint_idx + 2 * self.num_motor

                try:
                    self.low_state.motor_state[state_idx].q = self.mj_data.sensordata[q_index]
                    self.low_state.motor_state[state_idx].dq = self.mj_data.sensordata[dq_index]
                    self.low_state.motor_state[state_idx].tau_est = self.mj_data.sensordata[tau_index]
                except IndexError as e:
                    print(f"[PublishLowState] error: {type(e).__name__}: {e}")
                    print(f"{joint_idx=}, {q_index=}, {dq_index=}, {tau_index=}")
                    print(f"{len(self.low_state.motor_state)=}")
                    print(f"{len(self.mj_data.sensordata)=}")
                    print(f"{self.num_motor=}")
                    
                except Exception as e:
                    print(f"[PublishLowState] error: {type(e).__name__}: {e}")

            if self.have_frame_sensor_:
                imu_offset = self.dim_motor_sensor
                if sensor_len < imu_offset + 10:
                    print(f"[PublishLowState] sensordata too short for IMU: expected {imu_offset + 10}, got {sensor_len}")
                    return

                self.low_state.imu_state.quaternion = self.mj_data.sensordata[imu_offset : imu_offset + 4]
                self.low_state.imu_state.gyroscope = self.mj_data.sensordata[imu_offset + 4 : imu_offset + 7]
                self.low_state.imu_state.accelerometer = self.mj_data.sensordata[imu_offset + 7 : imu_offset + 10]

            if self.joystick is not None:
                try:
                    pygame.event.get()
                    self.low_state.wireless_remote[2] = int("".join([
                        f"{int(self.joystick.get_axis(self.axis_id.get('LT', 0)) > 0)}",
                        f"{int(self.joystick.get_axis(self.axis_id.get('RT', 0)) > 0)}",
                        f"{int(self.joystick.get_button(self.button_id.get('SELECT', 0)))}",
                        f"{int(self.joystick.get_button(self.button_id.get('START', 0)))}",
                        f"{int(self.joystick.get_button(self.button_id.get('LB', 0)))}",
                        f"{int(self.joystick.get_button(self.button_id.get('RB', 0)))}",
                    ]), 2)

                    self.low_state.wireless_remote[3] = int("".join([
                        f"{int(self.joystick.get_hat(0)[0] < 0)}",
                        f"{int(self.joystick.get_hat(0)[1] < 0)}",
                        f"{int(self.joystick.get_hat(0)[0] > 0)}",
                        f"{int(self.joystick.get_hat(0)[1] > 0)}",
                        f"{int(self.joystick.get_button(self.button_id.get('Y', 0)))}",
                        f"{int(self.joystick.get_button(self.button_id.get('X', 0)))}",
                        f"{int(self.joystick.get_button(self.button_id.get('B', 0)))}",
                        f"{int(self.joystick.get_button(self.button_id.get('A', 0)))}",
                    ]), 2)

                    sticks = [
                        self.joystick.get_axis(self.axis_id.get("LX", 0)),
                        self.joystick.get_axis(self.axis_id.get("RX", 0)),
                        -self.joystick.get_axis(self.axis_id.get("RY", 0)),
                        -self.joystick.get_axis(self.axis_id.get("LY", 0)),
                    ]
                    packs = list(map(lambda x: struct.pack("f", x), sticks))
                    self.low_state.wireless_remote[4:8] = packs[0]
                    self.low_state.wireless_remote[8:12] = packs[1]
                    self.low_state.wireless_remote[12:16] = packs[2]
                    self.low_state.wireless_remote[20:24] = packs[3]

                except Exception as e:
                    print(f"[PublishLowState] Joystick error: {type(e).__name__}: {e}")

            self.low_state_puber.Write(self.low_state)
        except Exception:
            traceback.print_exc()

    def PublishHighState(self):

        if self.mj_data != None:
            self.high_state.position[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 10
            ]
            self.high_state.position[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 11
            ]
            self.high_state.position[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 12
            ]

            self.high_state.velocity[0] = self.mj_data.sensordata[
                self.dim_motor_sensor + 13
            ]
            self.high_state.velocity[1] = self.mj_data.sensordata[
                self.dim_motor_sensor + 14
            ]
            self.high_state.velocity[2] = self.mj_data.sensordata[
                self.dim_motor_sensor + 15
            ]

        self.high_state_puber.Write(self.high_state)

    def PublishWirelessController(self):
        if self.joystick != None:
            pygame.event.get()
            key_state = [0] * 16
            key_state[self.key_map["R1"]] = self.joystick.get_button(
                self.button_id["RB"]
            )
            key_state[self.key_map["L1"]] = self.joystick.get_button(
                self.button_id["LB"]
            )
            key_state[self.key_map["start"]] = self.joystick.get_button(
                self.button_id["START"]
            )
            key_state[self.key_map["select"]] = self.joystick.get_button(
                self.button_id["SELECT"]
            )
            key_state[self.key_map["R2"]] = (
                self.joystick.get_axis(self.axis_id["RT"]) > 0
            )
            key_state[self.key_map["L2"]] = (
                self.joystick.get_axis(self.axis_id["LT"]) > 0
            )
            key_state[self.key_map["F1"]] = 0
            key_state[self.key_map["F2"]] = 0
            key_state[self.key_map["A"]] = self.joystick.get_button(self.button_id["A"])
            key_state[self.key_map["B"]] = self.joystick.get_button(self.button_id["B"])
            key_state[self.key_map["X"]] = self.joystick.get_button(self.button_id["X"])
            key_state[self.key_map["Y"]] = self.joystick.get_button(self.button_id["Y"])
            key_state[self.key_map["up"]] = self.joystick.get_hat(0)[1] > 0
            key_state[self.key_map["right"]] = self.joystick.get_hat(0)[0] > 0
            key_state[self.key_map["down"]] = self.joystick.get_hat(0)[1] < 0
            key_state[self.key_map["left"]] = self.joystick.get_hat(0)[0] < 0

            key_value = 0
            for i in range(16):
                key_value += key_state[i] << i

            self.wireless_controller.keys = key_value
            self.wireless_controller.lx = self.joystick.get_axis(self.axis_id["LX"])
            self.wireless_controller.ly = -self.joystick.get_axis(self.axis_id["LY"])
            self.wireless_controller.rx = self.joystick.get_axis(self.axis_id["RX"])
            self.wireless_controller.ry = -self.joystick.get_axis(self.axis_id["RY"])

            self.wireless_controller_puber.Write(self.wireless_controller)

    def SetupJoystick(self, device_id=0, js_type="xbox"):
        pygame.init()
        pygame.joystick.init()
        joystick_count = pygame.joystick.get_count()
        if joystick_count > 0:
            self.joystick = pygame.joystick.Joystick(device_id)
            self.joystick.init()
        else:
            print("No gamepad detected.")
            sys.exit()

        if js_type == "xbox":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 3,  # Right stick axis x
                "RY": 4,  # Right stick axis y
                "LT": 2,  # Left trigger
                "RT": 5,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 2,
                "Y": 3,
                "B": 1,
                "A": 0,
                "LB": 4,
                "RB": 5,
                "SELECT": 6,
                "START": 7,
            }

        elif js_type == "switch":
            self.axis_id = {
                "LX": 0,  # Left stick axis x
                "LY": 1,  # Left stick axis y
                "RX": 2,  # Right stick axis x
                "RY": 3,  # Right stick axis y
                "LT": 5,  # Left trigger
                "RT": 4,  # Right trigger
                "DX": 6,  # Directional pad x
                "DY": 7,  # Directional pad y
            }

            self.button_id = {
                "X": 3,
                "Y": 4,
                "B": 1,
                "A": 0,
                "LB": 6,
                "RB": 7,
                "SELECT": 10,
                "START": 11,
            }
        else:
            print("Unsupported gamepad. ")

    def PrintSceneInformation(self):
        print(" ")

        print("<<------------- Link ------------->> ")
        for i in range(self.mj_model.nbody):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_BODY, i)
            if name:
                print("link_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Joint ------------->> ")
        for i in range(self.mj_model.njnt):
            name = mujoco.mj_id2name(self.mj_model, mujoco._enums.mjtObj.mjOBJ_JOINT, i)
            if name:
                print("joint_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Actuator ------------->>")
        for i in range(self.mj_model.nu):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_ACTUATOR, i
            )
            if name:
                print("actuator_index:", i, ", name:", name)
        print(" ")

        print("<<------------- Sensor ------------->>")
        index = 0
        for i in range(self.mj_model.nsensor):
            name = mujoco.mj_id2name(
                self.mj_model, mujoco._enums.mjtObj.mjOBJ_SENSOR, i
            )
            if name:
                print(
                    "sensor_index:",
                    index,
                    ", name:",
                    name,
                    ", dim:",
                    self.mj_model.sensor_dim[i],
                )
            index = index + self.mj_model.sensor_dim[i]
        print(" ")


class ElasticBand:

    def __init__(self):
        self.stiffness = 200
        self.damping = 100
        self.point = np.array([0, 0, 2.5])
        self.length = 0
        self.enable = True

    def Advance(self, x, dx):
        """
        Args:
          δx: desired position - current position
          dx: current velocity
        """
        δx = self.point - x
        distance = np.linalg.norm(δx)
        direction = δx / distance
        v = np.dot(dx, direction)
        f = (self.stiffness * (distance - self.length) - self.damping * v) * direction
        return f

    def MujuocoKeyCallback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable
