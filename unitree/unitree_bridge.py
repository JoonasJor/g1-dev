
import mujoco
import numpy as np
import pygame
import sys
import struct
import traceback
import os
import sys
import threading

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelPublisher
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import WirelessController_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__WirelessController_
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowState_ as LowState_default

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config as Cfg
import g1_joints as Joints

MOTOR_SENSOR_NUM = 3
NUM_MOTOR_IDL_HG = 35

class UnitreeBridge:
    def __init__(self, mj_model, mj_data):
        self.mj_model = mj_model
        self.mj_data = mj_data

        self.num_motor = Cfg.NUM_MOTOR_BODY + Cfg.NUM_MOTOR_FINGERS
        self.dim_motor_sensor = MOTOR_SENSOR_NUM * self.num_motor
        self.have_imu = False
        self.have_frame_sensor = False
        self.dt = self.mj_model.opt.timestep

        # Joystick
        self.joystick = None
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

        # State handlers
        self.low_state = LowState_default()
        self.low_state_pub = ChannelPublisher(Cfg.TOPIC_BODY_LOW_STATE, LowState_)
        self.low_state_pub.Init()
        self.low_state_thread = RecurrentThread(
            interval=self.dt, 
            target=self.publish_low_state, 
            name="sim_lowstate"
        )
        self.low_state_thread.Start()

        self.high_state = unitree_go_msg_dds__SportModeState_()
        self.high_state_pub = ChannelPublisher(Cfg.TOPIC_BODY_HIGH_STATE, SportModeState_)
        self.high_state_pub.Init()
        self.high_state_thread = RecurrentThread(
            interval=self.dt, 
            target=self.publish_high_state, 
            name="sim_highstate"
        )
        self.high_state_thread.Start()

        self.wireless_controller = unitree_go_msg_dds__WirelessController_()
        self.wireless_controller_pub = ChannelPublisher(
            Cfg.TOPIC_WIRELESS_CONTROLLER, WirelessController_
        )
        self.wireless_controller_pub.Init()
        self.wireless_controller_thread = RecurrentThread(
            interval=0.01,
            target=self.publish_wireless_controller,
            name="sim_wireless_controller",
        )
        self.wireless_controller_thread.Start()

        # Control command handlers
        self.low_ctrl_sub = ChannelSubscriber(Cfg.TOPIC_BODY_CMD, LowCmd_)
        self.low_ctrl_sub.Init(self.low_cmd_handler, 10)

        self.arm_ctrl_sub = ChannelSubscriber(Cfg.TOPIC_ARM_SDK, LowCmd_)
        self.arm_ctrl_sub.Init(self.low_cmd_handler, 10)

        self.msg_lock = threading.Lock()
        self.last_ctrl_msg = None

        self.repeat_cmd_thread = RecurrentThread(
            interval=self.dt,
            target=self.repeat_last_low_cmd,
            name="sim_repeat_last_low_cmd",
        )
        self.repeat_cmd_thread.Start()

    def low_cmd_handler(self, msg: LowCmd_):
        """
        Save the last received control command. 
        Repeat this command in repeat_last_low_cmd thread.
        """

        try:
            with self.msg_lock:
                self.last_ctrl_msg = [cmd for cmd in msg.motor_cmd]
        except Exception as e:
            print(f"low_cmd_handler error: {type(e).__name__}: {e}")

    def repeat_last_low_cmd(self):
        """
        Repeat the last received control command to mimic real robot behaviour.
        """

        if self.mj_data is None:
            return
        with self.msg_lock:
            cmds = self.last_ctrl_msg
        if cmds is None:
            return
        
        try:
            for i, joint_idx in enumerate(Joints.Body.mujoco_idx_list()):
                q_index = joint_idx
                dq_index = joint_idx + self.num_motor
                cmd = cmds[i]
                control = (
                    cmd.tau + cmd.kp 
                    * (cmd.q - self.mj_data.sensordata[q_index])+ cmd.kd 
                    * (cmd.dq - self.mj_data.sensordata[dq_index])
                )
                self.mj_data.ctrl[joint_idx] = control
        except Exception as e:
            print(f"repeat_last_low_cmd error: {type(e).__name__}: {e}")
            print(f"{joint_idx=}, {q_index=}, {dq_index=}")
            print(f"{len(cmds)=}")
            print(f"{len(self.mj_data.ctrl)=}")
            print(f"{self.num_motor=}")

    def publish_low_state(self):
        try:
            if self.mj_data is None:
                print("[publish_low_state] mj_data is None")
                return

            sensor_len = len(self.mj_data.sensordata)
            expected_len = 3 * self.num_motor
            if sensor_len < expected_len:
                print(f"[publish_low_state] sensordata too short: expected {expected_len}, got {sensor_len}")
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
                    print(f"[publish_low_state] error: {type(e).__name__}: {e}")
                    print(f"{joint_idx=}, {q_index=}, {dq_index=}, {tau_index=}")
                    print(f"{len(self.low_state.motor_state)=}")
                    print(f"{len(self.mj_data.sensordata)=}")
                    print(f"{self.num_motor=}")
                    
                except Exception as e:
                    print(f"[publish_low_state] error: {type(e).__name__}: {e}")

            if self.have_frame_sensor_:
                imu_offset = self.dim_motor_sensor
                if sensor_len < imu_offset + 10:
                    print(f"[publish_low_state] sensordata too short for IMU: expected {imu_offset + 10}, got {sensor_len}")
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
                    print(f"[publish_low_state] Joystick error: {type(e).__name__}: {e}")

            self.low_state_pub.Write(self.low_state)
        except Exception:
            traceback.print_exc()

    def publish_high_state(self):

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

        self.high_state_pub.Write(self.high_state)

    def publish_wireless_controller(self):
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

            self.wireless_controller_pub.Write(self.wireless_controller)

    def setup_joystick(self, device_id=0, js_type="xbox"):
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

    def print_scene_info(self):
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

    def advance(self, x, dx):
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

    def mujoco_key_callback(self, key):
        glfw = mujoco.glfw.glfw
        if key == glfw.KEY_7:
            self.length -= 0.1
        if key == glfw.KEY_8:
            self.length += 0.1
        if key == glfw.KEY_9:
            self.enable = not self.enable
