import cv2
import numpy as np

def encode_image(image_array, quality=50):
    success, encoded_image = cv2.imencode('.jpg', image_array, [cv2.IMWRITE_JPEG_QUALITY, quality])
    if not success:
        print("Image encoding failed")
        return b''
    return encoded_image.tobytes()

def decode_image(image):
    byte_array = bytes(image)
    np_bytes = np.frombuffer(byte_array, dtype=np.uint8)
    image = cv2.imdecode(np_bytes, cv2.IMREAD_COLOR)
    return image