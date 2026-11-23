"""Minimal, version-independent PointCloud2 <-> numpy helpers (no sensor_msgs_py needed)."""
import numpy as np

# sensor_msgs/PointField datatypes
_DT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}


def read_fields(msg, names):
    """Return dict name -> 1D numpy array for the requested fields (missing names are skipped)."""
    by_name = {f.name: f for f in msg.fields}
    endian = '>' if msg.is_bigendian else '<'
    formats, offsets, keep = [], [], []
    for n in names:
        f = by_name.get(n)
        if f is None:
            continue
        keep.append(n)
        formats.append(endian + _DT[f.datatype])
        offsets.append(f.offset)
    dtype = np.dtype({'names': keep, 'formats': formats, 'offsets': offsets, 'itemsize': msg.point_step})
    n_pts = msg.width * msg.height
    buf = msg.data if isinstance(msg.data, (bytes, bytearray, memoryview)) else bytes(msg.data)
    arr = np.frombuffer(buf, dtype=dtype, count=n_pts)
    return {n: arr[n].astype(np.float64) for n in keep}


def make_xyzi(header, xyz, intensity=None):
    """Build a PointCloud2 (x, y, z, intensity as float32) from numpy arrays."""
    from sensor_msgs.msg import PointCloud2, PointField
    n = len(xyz)
    data = np.zeros((n, 4), dtype='<f4')
    data[:, :3] = xyz
    if intensity is not None:
        data[:, 3] = intensity
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                  for i, k in enumerate(['x', 'y', 'z', 'intensity'])]
    msg.is_bigendian = False  # data is written explicitly as little-endian float32
    msg.point_step = 16
    msg.row_step = 16 * n
    msg.is_dense = True
    msg.data = data.tobytes()
    return msg
