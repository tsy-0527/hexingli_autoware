import math, sys, os
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
from indoor_localization.cloud_ops import deskew_planar, point_times_to_sec, voxel_downsample, crop
from indoor_localization.zones import Zone, inflate_covariance

def test_deskew_matches_ground_truth():
    rng = np.random.default_rng(0)
    world = rng.uniform(-10, 10, (2000, 3))            # static points in a fixed frame W
    vx, wz, T = 0.8, 0.6, 0.1                             # 0.8 m/s, 34 deg/s, 100 ms scan
    t = np.sort(rng.uniform(0, T, len(world)))
    t_ref = T
    def pose(tt):                                         # vehicle pose in W at time tt (start at origin)
        th = wz * tt
        return vx / wz * math.sin(th), vx / wz * (1 - math.cos(th)), th
    meas = np.zeros_like(world)
    for i, (p, ti) in enumerate(zip(world, t)):           # what the sensor sees at ti (in B(ti))
        x, y, th = pose(ti); c, s = math.cos(th), math.sin(th)
        dx, dy = p[0] - x, p[1] - y
        meas[i] = [c * dx + s * dy, -s * dx + c * dy, p[2]]
    out = deskew_planar(meas, t, t_ref, vx, wz)
    x, y, th = pose(t_ref); c, s = math.cos(th), math.sin(th)    # expected: world points in B(t_ref)
    dx, dy = world[:, 0] - x, world[:, 1] - y
    exp = np.column_stack([c * dx + s * dy, -s * dx + c * dy, world[:, 2]])
    err_raw = np.abs(meas - exp).max(); err = np.abs(out - exp).max()
    print(f'deskew: max error raw {err_raw:.3f} m -> corrected {err:.2e} m')
    assert err < 1e-9 and err_raw > 0.05

def test_time_parsing():
    hdr = 1_700_000_000.0
    abs_ns = (hdr + np.array([0.0, 0.05, 0.1])) * 1e9
    assert np.allclose(point_times_to_sec(abs_ns, hdr), hdr + np.array([0, .05, .1]), atol=1e-6)
    assert np.allclose(point_times_to_sec(np.array([0, 50, 100.]), hdr), hdr + np.array([0, .05, .1]))       # ms
    assert np.allclose(point_times_to_sec(np.array([0, 5e7, 1e8]), hdr), hdr + np.array([0, .05, .1]))      # ns
    assert np.allclose(point_times_to_sec(np.array([0, .05, .1]), hdr), hdr + np.array([0, .05, .1]))       # s
    print('time parsing ok')

def test_voxel_and_crop():
    p = np.array([[1.01, 1.01, 1.0], [1.04, 1.02, 1.0], [5, 5, 1.0], [0.1, 0, 0.5], [3, 0, 0.05]])
    q, _ = crop(p, r_min=0.3, r_max=20, z_min=0.1, z_max=4, self_box=[-0.3, 0.8, -0.4, 0.4, -1, 3])
    assert len(q) == 3
    v, _ = voxel_downsample(q, 0.1)
    assert len(v) == 2
    print('crop/voxel ok')

def test_zone_inflation():
    z = Zone('c', (16.25, -36.0), (16.65, -14.0), 1.6, 1.0, 2.0)
    w_mid, yaw = z.weight_and_axis(16.45, -25.0)
    w_edge, _ = z.weight_and_axis(16.27, -35.0)
    w_out, _ = z.weight_and_axis(20.0, -25.0)
    assert abs(w_mid - 1) < 1e-9 and 0.4 < w_edge < 0.6 and w_out == 0
    cov = np.diag([0.0225, 0.0225, 0.0225, 6.25e-4, 6.25e-4, 6.25e-4]).ravel()
    cov[5] = cov[30] = 1e-3                       # some x-yaw correlation
    new, sl = inflate_covariance(cov, yaw, 1.0, w_mid)
    C = np.array(new).reshape(6, 6)
    c, s = math.cos(yaw), math.sin(yaw)
    along = np.array([c, s]); lat = np.array([-s, c])
    var_long = along @ C[:2, :2] @ along; var_lat = lat @ C[:2, :2] @ lat
    assert abs(var_long - 1.0) < 1e-6 and abs(var_lat - 0.0225) < 1e-6
    assert np.linalg.eigvalsh(C).min() > 0
    print(f'zone: axis yaw {math.degrees(yaw):.2f} deg, sigma_long {math.sqrt(var_long):.2f} m, sigma_lat {math.sqrt(var_lat):.3f} m, PD ok')

if __name__ == '__main__':
    test_deskew_matches_ground_truth(); test_time_parsing(); test_voxel_and_crop(); test_zone_inflation(); print('ALL PASS')
