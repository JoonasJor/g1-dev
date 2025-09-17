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
                "state": self.hand_r.low_state, # inspire.inspire_dds.inspire_hand_state
                "touch": self.hand_l.touch_state # inspire.inspire_dds.inspire_hand_touch
            },
            "hand_l": {
                "state": self.hand_r.low_state,
                "touch": self.hand_l.touch_state
            },
            # TODO: save only when new images are available
            "camera": self.camera.camera_msg # camera.camera_dds.camera_image
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
    
    def stop(self):
        if not self.logger_thread._Thread__thread.is_alive():
            print("Logger is not running. Exiting.")
            return
        
        self.logger_thread.Wait()
        print("Logging stopped.")

        while True:
            user_input = input("Keep the log file? (y/n): ").strip().lower()
            if user_input in ["y", "n"]:
                break
            else:
                print("Invalid input.")

        if user_input == "y":
            print(f"Log file saved as log/data_{self.datetime_stamp}.pkl")
            create_symlink(f"log/data_{self.datetime_stamp}.pkl", "log/data_latest.pkl")
        if user_input == "n":
            os.remove(f"log/data_{self.datetime_stamp}.pkl")
    
if __name__ == "__main__":
    if len(sys.argv) > 1:
        # Run on real robot
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        # Run in Mujoco
        ChannelFactoryInitialize(1, "lo")

    body = BodyController()

    hand_r = HandController("r")
    hand_l = HandController("l")

    camera = CameraSubscriber()

    while body.low_state is None:
        print(f"Waiting for body low state")
        time.sleep(1)

    while True:
        try:
            g1_logger = G1_Logger(body, hand_r, hand_l, camera)

            input("Press Enter to start logging:")

            g1_logger.logger_thread.Start()
            print("Logging started. Press Enter again to stop...")

            while True:
                time.sleep(0.5)
                if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                    input()
                    g1_logger.stop()
                    break
        except KeyboardInterrupt:
            print("KeyboardInterrupt. Stopping logger...")
            g1_logger.stop()
            break


