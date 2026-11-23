"""Software alignment of raw RealSense depth into the color camera image.

The realsense-ros ``align_depth`` filter runs only on synchronized RGB-D
framesets.  If its asynchronous syncer stops pairing frames, the aligned depth
topic disappears even though raw color and raw depth keep streaming.  This
module performs the depth -> color projection directly from the raw
(rectified) depth image, the color/depth camera_info intrinsics, and the
``extrinsics/depth_to_color`` calibration.
"""

import numpy as np


class DepthProjector:
    """Project raw RealSense depth pixels into the color camera image."""

    def __init__(self, color_k, color_d, depth_k, rotation, translation,
                 color_size, depth_size):
        color_k = np.asarray(color_k, dtype=np.float64)
        color_d = np.asarray(color_d, dtype=np.float64)
        depth_k = np.asarray(depth_k, dtype=np.float64)
        rotation = np.asarray(rotation, dtype=np.float64)
        translation = np.asarray(translation, dtype=np.float64)
        if color_k.shape != (9,) or depth_k.shape != (9,):
            raise ValueError('camera_info K must contain 9 elements')
        if rotation.size != 9 or translation.size != 3:
            raise ValueError(
                'depth_to_color extrinsics must contain 9 rotation and 3 translation values')
        self.color_width, self.color_height = (int(v) for v in color_size)
        self.depth_width, self.depth_height = (int(v) for v in depth_size)
        if min(self.color_width, self.color_height,
               self.depth_width, self.depth_height) < 1:
            raise ValueError('camera_info dimensions must be positive')

        self._fx_d, self._cx_d = depth_k[0], depth_k[2]
        self._fy_d, self._cy_d = depth_k[4], depth_k[5]
        self._fx_c, self._cx_c = color_k[0], color_k[2]
        self._fy_c, self._cy_c = color_k[4], color_k[5]
        if min(self._fx_d, self._fy_d, self._fx_c, self._fy_c) <= 0.0:
            raise ValueError('camera focal lengths must be positive')

        # librealsense rs2_extrinsics.rotation is column-major.
        self._rotation = rotation.reshape(3, 3, order='F')
        self._translation = translation.reshape(3)
        self._color_d = color_d

        # Pixel grid and normalized rays of the rectified depth image.
        u = np.arange(self.depth_width, dtype=np.float64)[None, :]
        v = np.arange(self.depth_height, dtype=np.float64)[:, None]
        self._x = np.broadcast_to(
            (u - self._cx_d) / self._fx_d,
            (self.depth_height, self.depth_width)).copy()
        self._y = np.broadcast_to(
            (v - self._cy_d) / self._fy_d,
            (self.depth_height, self.depth_width)).copy()

    def _distort(self, x, y):
        """Apply the color camera's plumb_bob/rational distortion model."""
        d = self._color_d
        if d.size < 5 or not np.any(d):
            return x, y
        r2 = x * x + y * y
        if d.size >= 8:
            k1, k2, p1, p2, k3, k4, k5, k6 = d[:8]
            radial = ((1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) /
                      (1.0 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2))
        else:
            k1, k2, p1, p2, k3 = d[:5]
            radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        x_d = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        y_d = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return x_d, y_d

    def project(self, depth_mm):
        """Return image pixels, raw depth and XYZ in the color optical frame."""
        depth_mm = np.asarray(depth_mm)
        if depth_mm.shape != (self.depth_height, self.depth_width):
            raise ValueError(
                f'depth image shape {depth_mm.shape} does not match camera_info '
                f'{(self.depth_height, self.depth_width)}')
        z = depth_mm.astype(np.float64) * 0.001
        valid = z > 0.0
        if not np.any(valid):
            return (np.empty(0, np.int32), np.empty(0, np.int32),
                    np.empty(0, np.uint16), np.empty(0, np.float32),
                    np.empty(0, np.float32), np.empty(0, np.float32))

        z = z[valid]
        x = self._x[valid] * z
        y = self._y[valid] * z
        r = self._rotation
        t = self._translation
        xc = r[0, 0] * x + r[0, 1] * y + r[0, 2] * z + t[0]
        yc = r[1, 0] * x + r[1, 1] * y + r[1, 2] * z + t[1]
        zc = r[2, 0] * x + r[2, 1] * y + r[2, 2] * z + t[2]
        in_front = zc > 0.0
        if not np.any(in_front):
            return (np.empty(0, np.int32), np.empty(0, np.int32),
                    np.empty(0, np.uint16), np.empty(0, np.float32),
                    np.empty(0, np.float32), np.empty(0, np.float32))

        xc, yc, zc = xc[in_front], yc[in_front], zc[in_front]
        x_d, y_d = self._distort(xc / zc, yc / zc)
        u = np.rint(self._fx_c * x_d + self._cx_c).astype(np.int32)
        v = np.rint(self._fy_c * y_d + self._cy_c).astype(np.int32)
        inside = ((u >= 0) & (u < self.color_width) &
                  (v >= 0) & (v < self.color_height))
        return (u[inside], v[inside], depth_mm[valid][in_front][inside],
                xc[inside].astype(np.float32), yc[inside].astype(np.float32),
                zc[inside].astype(np.float32))
