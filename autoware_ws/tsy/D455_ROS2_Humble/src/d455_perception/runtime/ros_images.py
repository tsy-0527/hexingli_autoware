"""Read ROS Image rows including padding, byte order and RGB/BGR ordering."""
import numpy as np


def unpack_image(msg, depth=False):
    if depth and msg.encoding not in ('16UC1', 'mono16'):
        raise ValueError('Expected aligned RealSense 16UC1 depth in millimetres')
    if not depth and msg.encoding not in ('bgr8', 'rgb8'):
        raise ValueError('Expected rgb8 or bgr8 color')
    h, w, step = msg.height, msg.width, msg.step
    size = 2 if depth else 3
    if min(h, w) < 1 or step < w * size or len(msg.data) != h * step:
        raise ValueError('Invalid ROS Image dimensions, data length or row stride')
    if depth:
        dtype = '>u2' if msg.is_bigendian else '<u2'
        return np.ndarray((h, w), dtype, msg.data, strides=(step, 2)).astype(np.uint16, copy=False)
    a = np.ndarray((h, w, 3), np.uint8, msg.data, strides=(step, 3, 1))
    return np.ascontiguousarray(a[:, :, ::-1] if msg.encoding == 'rgb8' else a)
