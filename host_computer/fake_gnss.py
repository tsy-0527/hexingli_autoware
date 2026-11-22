#!/usr/bin/env python3
"""
虚拟 GNSS 发布器（室内测试“实车模式·定位初始化”用）

向 /sensing/gnss/ublox/nav_sat_fix 持续发布一个固定的经纬度，模拟 GNSS 接收机。
既可以直接给经纬度，也可以给 map 坐标（x 东、y 北），脚本按地图投影反算成经纬度。

用法示例（在工控机上，source 过 ROS 2 / Autoware 环境后运行）：
  # 1) 直接给经纬度
  python3 fake_gnss.py --lat 31.9337764 --lon 118.6332689

  # 2) 给 map 坐标（推荐：在网页 3D 里找一个车道上的点），按 LocalCartesianUTM 反算
  python3 fake_gnss.py --x 12 --y 0 --projection LocalCartesianUTM \
      --origin-lat 31.933778 --origin-lon 118.633142

  # 3) 模拟“无解”，检查网页提示
  python3 fake_gnss.py --x 12 --y 0 --nofix

  # 只看换算结果、不发布（不需要 ROS 环境）
  python3 fake_gnss.py --x 12 --y 0 --dry-run

注意：
  - 投影类型与原点必须和 Autoware 地图一致：ros2 topic echo /map/map_projector_info --once
  - MGRS 投影请直接用 --lat/--lon（map 坐标反算需要知道所在 100 km 网格）。
  - 测试时不要同时运行真实的 GNSS 驱动，否则两路数据会互相覆盖。
"""
import argparse
import math
import sys

WGS84_A = 6378137.0
WGS84_F = 1 / 298.257223563
WGS84_E2 = WGS84_F * (2 - WGS84_F)


# ───────── UTM（Krüger 级数，毫米级精度） ─────────
def _utm_consts():
    n = WGS84_F / (2 - WGS84_F)
    A = WGS84_A / (1 + n) * (1 + n * n / 4 + n ** 4 / 64)
    alpha = [n / 2 - 2 * n * n / 3 + 5 * n ** 3 / 16, 13 * n * n / 48 - 3 * n ** 3 / 5, 61 * n ** 3 / 240]
    beta = [n / 2 - 2 * n * n / 3 + 37 * n ** 3 / 96, n * n / 48 + n ** 3 / 15, 17 * n ** 3 / 480]
    delta = [2 * n - 2 * n * n / 3 - 2 * n ** 3, 7 * n * n / 3 - 8 * n ** 3 / 5, 56 * n ** 3 / 15]
    return n, A, alpha, beta, delta


def latlon_to_utm(lat, lon, zone=None):
    zone = zone or int((lon + 180) // 6) + 1
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    k0 = 0.9996
    n, A, alpha, _, _ = _utm_consts()
    phi = math.radians(lat)
    lam = math.radians(lon) - lon0
    c = 2 * math.sqrt(n) / (1 + n)
    t = math.sinh(math.atanh(math.sin(phi)) - c * math.atanh(c * math.sin(phi)))
    xi = math.atan2(t, math.cos(lam))
    eta = math.atanh(math.sin(lam) / math.sqrt(1 + t * t))
    E, N = eta, xi
    for j in range(1, 4):
        E += alpha[j - 1] * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
        N += alpha[j - 1] * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
    return 500000 + k0 * A * E, (10000000 if lat < 0 else 0) + k0 * A * N, zone


def utm_to_latlon(E, N, zone, south=False):
    k0 = 0.9996
    n, A, _, beta, delta = _utm_consts()
    xi = (N - (10000000 if south else 0)) / (k0 * A)
    eta = (E - 500000) / (k0 * A)
    xi_p, eta_p = xi, eta
    for j in range(1, 4):
        xi_p -= beta[j - 1] * math.sin(2 * j * xi) * math.cosh(2 * j * eta)
        eta_p -= beta[j - 1] * math.cos(2 * j * xi) * math.sinh(2 * j * eta)
    chi = math.asin(math.sin(xi_p) / math.cosh(eta_p))
    phi = chi
    for j in range(1, 4):
        phi += delta[j - 1] * math.sin(2 * j * chi)
    lon0 = math.radians((zone - 1) * 6 - 180 + 3)
    lam = lon0 + math.atan2(math.sinh(eta_p), math.cos(xi_p))
    return math.degrees(phi), math.degrees(lam)


# ───────── LocalCartesian（以原点为中心的 ENU） ─────────
def _ecef(lat, lon, h):
    p, l = math.radians(lat), math.radians(lon)
    Nr = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(p) ** 2)
    return ((Nr + h) * math.cos(p) * math.cos(l), (Nr + h) * math.cos(p) * math.sin(l), (Nr * (1 - WGS84_E2) + h) * math.sin(p))


def enu_to_latlon(x, y, z, lat0, lon0, h0):
    x0, y0, z0 = _ecef(lat0, lon0, h0)
    p0, l0 = math.radians(lat0), math.radians(lon0)
    dx = -math.sin(l0) * x - math.sin(p0) * math.cos(l0) * y + math.cos(p0) * math.cos(l0) * z
    dy = math.cos(l0) * x - math.sin(p0) * math.sin(l0) * y + math.cos(p0) * math.sin(l0) * z
    dz = math.cos(p0) * y + math.sin(p0) * z
    X, Y, Z = x0 + dx, y0 + dy, z0 + dz
    lon = math.atan2(Y, X)
    p = math.hypot(X, Y)
    lat = math.atan2(Z, p * (1 - WGS84_E2))
    for _ in range(10):
        Nr = WGS84_A / math.sqrt(1 - WGS84_E2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - Nr
        lat = math.atan2(Z, p * (1 - WGS84_E2 * Nr / (Nr + h)))
    return math.degrees(lat), math.degrees(lon)


def map_to_latlon(x, y, projection, olat, olon, oalt=0.0):
    if projection == "LocalCartesianUTM":
        E0, N0, zone = latlon_to_utm(olat, olon)
        return utm_to_latlon(E0 + x, N0 + y, zone, south=olat < 0)
    if projection == "LocalCartesian":
        return enu_to_latlon(x, y, 0.0, olat, olon, oalt)
    raise SystemExit("MGRS 投影请直接用 --lat / --lon 指定经纬度")


def main():
    ap = argparse.ArgumentParser(description="虚拟 GNSS 发布器（NavSatFix）")
    ap.add_argument("--lat", type=float)
    ap.add_argument("--lon", type=float)
    ap.add_argument("--alt", type=float, default=0.0)
    ap.add_argument("--x", type=float, help="map 坐标 x（东）")
    ap.add_argument("--y", type=float, help="map 坐标 y（北）")
    ap.add_argument("--projection", default="LocalCartesianUTM", choices=["LocalCartesianUTM", "LocalCartesian", "MGRS"])
    ap.add_argument("--origin-lat", type=float, default=31.933778)
    ap.add_argument("--origin-lon", type=float, default=118.633142)
    ap.add_argument("--origin-alt", type=float, default=0.0)
    ap.add_argument("--topic", default="/sensing/gnss/ublox/nav_sat_fix")
    ap.add_argument("--frame", default="gnss_link")
    ap.add_argument("--rate", type=float, default=10.0, help="发布频率 Hz")
    ap.add_argument("--status", type=int, default=2, help="-1 无解 / 0 单点 / 1 差分 / 2 RTK")
    ap.add_argument("--nofix", action="store_true", help="发布无解状态（status=-1）")
    ap.add_argument("--dry-run", action="store_true", help="只打印换算结果，不发布")
    a = ap.parse_args()

    if a.lat is not None and a.lon is not None:
        lat, lon = a.lat, a.lon
    elif a.x is not None and a.y is not None:
        lat, lon = map_to_latlon(a.x, a.y, a.projection, a.origin_lat, a.origin_lon, a.origin_alt)
    else:
        ap.error("请指定 --lat/--lon，或 --x/--y")
    status = -1 if a.nofix else a.status
    print(f"虚拟 GNSS：lat={lat:.9f} lon={lon:.9f} alt={a.alt} status={status}  → {a.topic} @ {a.rate} Hz")
    if a.dry_run:
        return

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import NavSatFix, NavSatStatus

    class FakeGnss(Node):
        def __init__(self):
            super().__init__("fake_gnss")
            self.pub = self.create_publisher(NavSatFix, a.topic, 10)
            self.create_timer(1.0 / a.rate, self.tick)

        def tick(self):
            m = NavSatFix()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = a.frame
            m.status.status = status
            m.status.service = NavSatStatus.SERVICE_GPS
            m.latitude, m.longitude, m.altitude = lat, lon, a.alt
            m.position_covariance = [0.0004, 0.0, 0.0, 0.0, 0.0004, 0.0, 0.0, 0.0, 0.01]
            m.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            self.pub.publish(m)

    rclpy.init()
    node = FakeGnss()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
