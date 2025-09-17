import os 
import sys
import time
import pickle
import select
import cv2
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.utils.thread import RecurrentThread
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

from controller import BodyController, HandController

import inspire.inspire_dds as inspire_dds
from inspire.modbus_data_handler import ModbusDataHandler

from camera.camera_subscriber import CameraSubscriber
from camera.utils import decode_image
import camera.camera_dds as camera_dds

def create_symlink(target_path, latest_link):
    """
    Create or update a symlink latest_link -> target_path atomically.
    """
    os.makedirs(os.path.dirname(latest_link), exist_ok=True)
    tmp_link = latest_link + ".tmp"

    if os.path.lexists(tmp_link):
        os.unlink(tmp_link)
    os.symlink(os.path.abspath(target_path), tmp_link)
    os.replace(tmp_link, latest_link)

class G1_Logger():
    def __init__(self, body: BodyController, hand_r: HandController, hand_l: HandController, camera: CameraSubscriber):
        self.body = body
        self.hand_r = hand_r
        self.hand_l = hand_l
        self.camera = camera

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
            },
            "camera": self.camera.camera_msg # TODO: save only when new images are available
        }

        with open(f"log/data_{self.datetime_stamp}.pkl", "ab") as file:
            pickle.dump(data, file)

    @staticmethod
    def load_data_from_file(filename):
        data_list = []

        with open(filename, "rb") as file:
            print("Loading data from file...")
            while True:
                try:
                    data = pickle.load(file)
                    data_list.append(data)
                except EOFError:
                    break

        return data_list
    
if __name__ == "__main__":
    # Example usage
    write_logs = True

    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
        #modbus_r = ModbusDataHandler(ip="192.168.123.211", device_id=1, l_r="r")
        #modbus_l = ModbusDataHandler(ip="192.168.123.210", device_id=2, l_r="l")
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    body = BodyController()

    hand_r = HandController("r")
    hand_l = HandController("l")

    camera = CameraSubscriber()

    if write_logs:
        while body.low_state is None:
            print(f"Waiting for body low state")
            time.sleep(1)

    g1_logger = G1_Logger(body, hand_r, hand_l, camera)
    
    # Write
    if write_logs: 
        while True:
            input("Press Enter to start logging:")

            create_symlink(f"log/data_{g1_logger.datetime_stamp}.pkl", "log/data_latest.pkl")

            g1_logger.logger_thread.Start()
            print("Logging started. Press Enter again to exit...")

            while True:
                time.sleep(1)
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    input()
                    print("Exiting.")
                    g1_logger.logger_thread.Wait()
                    sys.exit()
    # Read
    else:
        data_list = g1_logger.load_data_from_file("log/data_latest.pkl")
        for data in data_list:
            # Unpack data
            timestamp = data["timestamp"]

            body_low_state: LowState_ = data["body"]
            body_joint_angles = [motor.q for motor in body_low_state.motor_state]

            hand_r_state: inspire_dds.inspire_hand_state = data["hand_r"]["state"]
            hand_r_touch: inspire_dds.inspire_hand_touch = data["hand_r"]["touch"]
            hand_r_joint_angles = hand_r_state.angle_act

            hand_l_state: inspire_dds.inspire_hand_state = data["hand_l"]["state"]
            hand_l_touch: inspire_dds.inspire_hand_touch = data["hand_l"]["touch"]
            hand_l_joint_angles = hand_l_state.angle_act

            camera_data: camera_dds.camera_image = data["camera"]
            
            try:
                # Decode and save images
                camera_rgb_decoded = decode_image(camera_data.rgb)
                cv2.imwrite(f"log/img/{camera_data.timestamp}_rgb.jpg", cv2.cvtColor(camera_rgb_decoded, cv2.COLOR_RGB2BGR))

                camera_depth_decoded = decode_image(camera_data.depth)
                np.save(f"log/img/{camera_data.timestamp}_depth.npy", camera_depth_decoded)
            except Exception as e:
                print(f"Error decoding/saving images: {e}")
                continue

            print(f"Timestamp (data saved):   {timestamp}")
            print(f"Timestamp (images taken): {camera_data.timestamp}")

