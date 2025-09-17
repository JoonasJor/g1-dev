import os 
import sys
import cv2
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
import inspire.inspire_dds as inspire_dds
import camera.camera_dds as camera_dds
from camera.utils import decode_image

from logger.g1_logger import G1_Logger

if __name__ == "__main__":
    data_list = G1_Logger.load_data_from_file("log/data_latest.pkl")
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