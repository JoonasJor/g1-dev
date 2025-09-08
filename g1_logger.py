import os 
import sys
import time
import pickle

from controller import BodyController, HandController
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from inspire.modbus_data_handler import ModbusDataHandler
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
import inspire.inspire_dds as inspire_dds

class G1_Logger():
    def __init__(self, body: BodyController, hand_r: HandController, hand_l: HandController):
        self.body = body
        self.hand_r = hand_r
        self.hand_l = hand_l

        self.datetime_stamp = time.strftime(f"%Y%m%d_%H%M%S")

        self.logger_thread = RecurrentThread(
            interval=0.2, 
            target=self.save_data_to_file, 
            name=f"logger"
        )

    def save_data_to_file(self):
        data = {
            "timestamp": time.time(),
            "body": self.body.low_state, # unitree_hg.msg.dds_.LowState_
            "hand_r": {
                "state": self.hand_r.low_state, # inspire.inspire_hand_state
                "touch": self.hand_l.touch_state # inspire.inspire_hand_touch
            },
            "hand_l": {
                "state": self.hand_r.low_state,
                "touch": self.hand_l.touch_state
            }
        }

        with open(f"log/data_{self.datetime_stamp}.pkl", "ab") as file:
            pickle.dump(data, file)

    def load_data_from_file(self, filename):
        data_list = []

        with open(filename, "rb") as file:
            while True:
                try:
                    data = pickle.load(file)
                    data_list.append(data)
                except EOFError:
                    break

        return data_list

if __name__ == "__main__":
    # Example usage
    
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
        modbus_r = ModbusDataHandler(ip="192.168.123.211", device_id=1, l_r="r")
        modbus_l = ModbusDataHandler(ip="192.168.123.210", device_id=2, l_r="l")
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    body = BodyController()

    hand_r = HandController("r")
    hand_l = HandController("l")

    while body.low_state is None:
        print(f"Waiting for body low state")
        time.sleep(1)

    g1_logger = G1_Logger(body, hand_r, hand_l)
    write_logs = False

    # Write
    if write_logs: 
        print("Logging started.")
        g1_logger.logger_thread.Start()
        while True:
            time.sleep(1)
    # Read
    else:
        data_list = g1_logger.load_data_from_file("log/data_20250908_162826.pkl")
        for data in data_list:
            timestamp = data["timestamp"]

            body_low_state: LowState_ = data["body"]

            hand_r_state: inspire_dds.inspire_hand_state = data["hand_r"]["state"]
            hand_r_touch: inspire_dds.inspire_hand_touch = data["hand_l"]["touch"]

            hand_r_state: inspire_dds.inspire_hand_state = data["hand_r"]["state"]
            hand_r_touch: inspire_dds.inspire_hand_touch = data["hand_l"]["touch"]

            #angles = [motor.q for motor in body_low_state.motor_state]
            #print(angles)
            print(f"{timestamp}:\t{hand_r_state.angle_act}")

