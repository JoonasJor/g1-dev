import pyrealsense2 as rs
import numpy as np
import os
import sys
import time

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.utils.thread import RecurrentThread

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
import config as Cfg
import camera.camera_dds as camera_dds
from camera.utils import encode_image

class G1_CameraPublisher():
    """
    Publishes images from the physical camera using DDS communication.  
    Deploy this on the G1 Jetson (PC2).

    Args:
        resolution (tuple): (width, height) of the image
        fps (int): frames per second
    """

    def __init__(self, resolution, fps):
        self._width, self._height = resolution
        self._fps = fps

        # Initialize RealSense camera
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.color, self._width, self._height, rs.format.bgr8, self._fps)
        config.enable_stream(rs.stream.depth, self._width, self._height, rs.format.z16, self._fps)

        profile = self.pipeline.start(config)

        self._device = profile.get_device()
        if self._device is None:
            print("[RealSenseCamera] pipe_profile.get_device() is None. Exiting.")
            sys.exit()

        depth_sensor = self._device.first_depth_sensor()
        self.g_depth_scale = depth_sensor.get_depth_scale()

        self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

        # Initialize DDS publisher
        self._rgb_image = np.zeros((self._height, self._width, 3), dtype=np.uint8)
        self._depth_image = np.zeros((self._height, self._width, 1), dtype=np.float32)
        self._timestamp = 0.0

        self.camera_image = camera_dds.camera_image(timestamp=self._timestamp, rgb=self._rgb_image, depth=self._depth_image)
        self.camera_image_pub = ChannelPublisher(f"{Cfg.TOPIC_CAMERA_IMAGE}", camera_dds.camera_image)
        self.camera_image_pub.Init()
        self.camera_image_thread = RecurrentThread(
            interval=1.0 / self._fps, 
            target=self.capture_and_publish, 
            name=f"g1_camera_publisher"
        )
        self.camera_image_thread.Start()

    def capture_images(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        if not color_frame or not depth_frame:
            print("[RealSenseCamera] No frame received.")
            return None

        self._rgb_image = np.asanyarray(color_frame.get_data())
        self._depth_image = np.asanyarray(depth_frame.get_data())
        self._timestamp = time.time()
    
    def capture_and_publish(self):
        self.capture_images()

        encoded_rgb = encode_image(self._rgb_image)
        encoded_depth = encode_image(self._depth_image)

        self.camera_image.timestamp = self._timestamp
        self.camera_image.rgb = encoded_rgb
        self.camera_image.depth = encoded_depth

        self.camera_image_pub.Write(self.camera_image)

    def release(self):
        self.pipeline.stop()

if __name__ == '__main__':
    camera_publisher = G1_CameraPublisher(resolution=(640, 480), fps=30)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("Stopping camera publisher...")
    finally:
        camera_publisher.release()