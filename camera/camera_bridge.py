import os
import sys
import time
import numpy as np
import mujoco as mj

from unitree_sdk2py.core.channel import ChannelPublisher

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config as Cfg
import camera.camera_dds as camera_dds
from camera.utils import encode_image

class CameraBridge():
    """
    Publishes camera images from the MuJoCo simulation using DDS communication.

    Args:
        mj_model: MuJoCo model
        mj_data: MuJoCo data
        resolution (tuple): (width, height) of the image
        camera_name (str): name of the camera in the MuJoCo model
    """

    def __init__(self, mj_model, mj_data, resolution, camera_name):
        self._mj_model = mj_model
        self._mj_data = mj_data
        self._width, self._height = resolution
        self._camera_name = camera_name

        self._renderer = mj.Renderer(
            model=self._mj_model, 
            height=self._height, 
            width=self._width
        )
        self._camera = mj.MjvCamera()
        self._scene = mj.MjvScene(self._mj_model, maxgeom=10_000)

        self._rgb_image = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        self._depth_image = np.zeros((self._height, self._width, 1), dtype=np.float32)
        self._timestamp = 0.0

        self.camera_image = camera_dds.camera_image(timestamp=self._timestamp, rgb=self._rgb_image, depth=self._depth_image)
        self.camera_image_pub = ChannelPublisher(f"{Cfg.TOPIC_CAMERA_IMAGE}", camera_dds.camera_image)
        self.camera_image_pub.Init()

    def _capture_rgb_image(self):
        rgb_image = self._renderer.render()

        return rgb_image

    def _capture_depth_image(self):
        self._renderer.enable_depth_rendering()
        depth_image = self._renderer.render()
        self._renderer.disable_depth_rendering()

        return depth_image

    def _publish_camera_image(self):
        compressed_rgb = encode_image(self._rgb_image)
        compressed_depth = encode_image(self._depth_image)

        self.camera_image.timestamp = self._timestamp
        self.camera_image.rgb = compressed_rgb
        self.camera_image.depth = compressed_depth

        self.camera_image_pub.Write(self.camera_image)

    def capture_and_publish(self):
        start_time = time.time()

        self._renderer.update_scene(self._mj_data, camera=self._camera_name)
        self._rgb_image = self._capture_rgb_image()
        self._depth_image = self._capture_depth_image()
        self._timestamp = time.time()

        self._publish_camera_image()

        #print(f"capture_and_publish time: {time.time() - start_time:.4f} seconds")