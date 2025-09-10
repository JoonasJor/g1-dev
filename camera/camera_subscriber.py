import os
import sys
import time

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import camera.camera_dds as camera_dds
import config as Cfg

class CameraSubscriber:
    def __init__(self):
        self.camera_msg = None
        
        self.camera_image_sub = ChannelSubscriber(f"{Cfg.TOPIC_CAMERA_IMAGE}", camera_dds.camera_image)
        self.camera_image_sub.Init(self.camera_image_handler, 10)

    def camera_image_handler(self, msg: camera_dds.camera_image):
        self.camera_msg = msg

if __name__ == '__main__':
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    camera_subscriber = CameraSubscriber()

    while True:
        time.sleep(1)
