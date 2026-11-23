#!/usr/bin/env python3
"""Smooth only the indoor lane pair (forward/reverse) of a lanelet2 map.

The centerline is fitted with a cubic smoothing spline, resampled with denser
vertices in curves, and the left/right boundaries are rebuilt from the original
width profile.  The start/junction nodes and the whole rest of the map stay
untouched.

Usage:
    python3 smooth_indoor_lane.py input.osm output.osm
"""
import argparse
import math

import numpy as np
import xml.etree.ElementTree as ET
from pyproj import Transformer
from scipy.interpolate import splprep, splev


def tags_of(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def role_ref(relation, role):
    for member in relation.findall("member"):
        if member.get("role") == role:
            return member.get("ref")
    raise ValueError("relation {} has no {} member".format(relation.get("id"), role))


def way_refs(way):
    return [nd.get("ref") for nd in way.findall("nd")]


def set_way_refs(way, refs):
    for nd in list(way.findall("nd")):
        way.remove(nd)
    for ref in refs:
        ET.SubElement(way, "nd", {"ref": str(ref)})


def smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def turn_angles(points):
    if len(points) < 3:
        return np.array([0.0])
    a = points[1:-1] - points[:-2]
    b = points[2:] - points[1:-1]
    cross = a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]
    dot = np.sum(a * b, axis=1)
    return np.abs(np.degrees(np.arctan2(cross, dot)))


def adaptive_indices(points, min_spacing, max_spacing, gain=6.0):
    ds = np.linalg.norm(np.diff(points, axis=0), axis=1)
    if len(ds) < 3:
        return np.arange(len(points))
    tangent = np.gradient(points, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    yaw = np.unwrap(np.arctan2(tangent[:, 1], tangent[:, 0]))
    yaw = np.convolve(np.pad(yaw, 4, mode="edge"), np.ones(9) / 9.0, mode="valid")
    kappa = np.abs(np.diff(yaw)) / np.maximum(ds, 1e-6)
    kappa = np.convolve(np.pad(kappa, 6, mode="edge"), np.ones(13) / 13.0, mode="valid")
    spacing = np.clip(max_spacing / (1.0 + gain * kappa), min_spacing, max_spacing)

    keep = [0]
    travelled = 0.0
    for i in range(1, len(points)):
        travelled += ds[i - 1]
        if travelled >= spacing[min(i - 1, len(spacing) - 1)]:
            keep.append(i)
            travelled = 0.0
    if keep[-1] != len(points) - 1:
        keep.append(len(points) - 1)
    return np.array(keep)


def fit_u_turn(positions, threshold_deg=15.0):
    """Fit the U-turn with a cubic Bezier.

    Returns (entry index, exit index, dense Bezier points).  The rest of the
    centerline is not modified here.
    """
    d = np.diff(positions, axis=0)
    yaws = np.arctan2(d[:, 1], d[:, 0])
    yaws = np.r_[yaws, yaws[-1]]
    threshold = np.deg2rad(threshold_deg)
    final = yaws[-1]
    opposite = final + np.pi

    exit_idx = len(positions) - 1
    while exit_idx > 0 and abs((yaws[exit_idx - 1] - final + np.pi) % (2 * np.pi) - np.pi) < threshold:
        exit_idx -= 1
    entry_idx = None
    for i in range(exit_idx - 1, -1, -1):
        if abs((yaws[i] - opposite + np.pi) % (2 * np.pi) - np.pi) < threshold:
            entry_idx = i
            break
    if entry_idx is None or exit_idx - entry_idx < 3:
        return None, None, None

    p0, p3 = positions[entry_idx], positions[exit_idx]
    t0 = positions[entry_idx] - positions[entry_idx - 1]
    t0 /= np.linalg.norm(t0)
    if exit_idx + 1 < len(positions):
        t3 = positions[exit_idx + 1] - positions[exit_idx]
    else:
        t3 = np.array([np.cos(final), np.sin(final)])
    t3 /= np.linalg.norm(t3)
    interior = positions[entry_idx + 1:exit_idx]
    pts = np.vstack([p0, interior, p3])
    chord = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    t_int = chord[1:-1] / chord[-1]
    matrix, rhs = [], []
    for t, point in zip(t_int, interior):
        b1 = (1 - t) ** 3 + 3 * (1 - t) ** 2 * t
        b2 = 3 * (1 - t) * t ** 2 + t ** 3
        base = b1 * p0 + b2 * p3
        ca = 3 * (1 - t) ** 2 * t
        cb = -3 * (1 - t) * t ** 2
        matrix.append(np.array([[ca * t0[0], cb * t3[0]], [ca * t0[1], cb * t3[1]]]))
        rhs.append(point - base)
    a, b = np.linalg.lstsq(np.vstack(matrix), np.hstack(rhs), rcond=None)[0]
    if not (np.isfinite(a) and np.isfinite(b)) or a <= 0 or b <= 0:
        a = b = 0.5 * np.linalg.norm(p3 - p0)
    p1, p2 = p0 + a * t0, p3 - b * t3
    steps = max(30, int(math.ceil(chord[-1] / 0.08)))
    ts = np.linspace(0.0, 1.0, steps)
    mt = 1.0 - ts
    dense = (
        (mt**3)[:, None] * p0
        + (3 * mt**2 * ts)[:, None] * p1
        + (3 * mt * ts**2)[:, None] * p2
        + (ts**3)[:, None] * p3
    )
    return entry_idx, exit_idx, dense


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input_osm")
    parser.add_argument("output_osm")
    parser.add_argument("--forward", default="981901")
    parser.add_argument("--reverse", default="981903")
    parser.add_argument("--smoothing", type=float, default=0.3,
                        help="splprep smoothing factor")
    parser.add_argument("--wide-width", type=float, default=2.5)
    parser.add_argument("--narrow-width", type=float, default=1.2)
    parser.add_argument("--narrow-start", type=float, default=15.0,
                        help="distance from the end where the narrow section starts [m]")
    parser.add_argument("--narrow-transition", type=float, default=3.0)
    parser.add_argument("--min-spacing", type=float, default=0.12)
    parser.add_argument("--max-spacing", type=float, default=0.45)
    parser.add_argument("--curvature-gain", type=float, default=6.0)
    args = parser.parse_args()

    root = ET.parse(args.input_osm).getroot()
    nodes = {n.get("id"): n for n in root.findall("node")}
    ways = {w.get("id"): w for w in root.findall("way")}
    relations = {r.get("id"): r for r in root.findall("relation")}

    forward = relations[args.forward]
    reverse = relations[args.reverse]
    f_left = ways[role_ref(forward, "left")]
    f_right = ways[role_ref(forward, "right")]
    f_center = ways[role_ref(forward, "centerline")]
    r_left = ways[role_ref(reverse, "left")]
    r_right = ways[role_ref(reverse, "right")]
    r_center = ways[role_ref(reverse, "centerline")]

    left_ids = way_refs(f_left)
    right_ids = way_refs(f_right)
    center_ids = way_refs(f_center)
    if not (len(left_ids) == len(right_ids) == len(center_ids)):
        raise ValueError("forward boundary/centerline lengths differ")
    if way_refs(r_left) != list(reversed(right_ids)):
        raise ValueError("reverse left way is not reversed forward right")
    if way_refs(r_right) != list(reversed(left_ids)):
        raise ValueError("reverse right way is not reversed forward left")
    if way_refs(r_center) != list(reversed(center_ids)):
        raise ValueError("reverse centerline way is not reversed forward centerline")

    def coords(ids):
        return np.array([
            [float(tags_of(nodes[i])["local_x"]), float(tags_of(nodes[i])["local_y"])]
            for i in ids
        ])

    L = coords(left_ids)
    R = coords(right_ids)
    C = coords(center_ids)
    print("input vertices:", len(L))
    print("old max boundary turn: %.1f deg" % max(turn_angles(L).max(), turn_angles(R).max()))

    entry_idx, exit_idx, bezier = fit_u_turn(C)
    if entry_idx is not None:
        print("U-turn Bezier: vertices {}..{}".format(entry_idx, exit_idx))
        pre = C[: entry_idx + 1]
        dpre = np.linalg.norm(np.diff(pre, axis=0), axis=1)
        if len(pre) >= 4 and dpre.sum() > 0:
            up = np.concatenate([[0.0], np.cumsum(dpre)])
            up /= up[-1]
            tck, _ = splprep([pre[:, 0], pre[:, 1]], u=up, s=args.smoothing, k=3)
            npre = max(20, int(math.ceil(dpre.sum() / 0.10)))
            upsam = np.linspace(0.0, 1.0, npre)
            xpre, ypre = splev(upsam, tck)
            pre_dense = np.c_[xpre, ypre]
            pre_dense[0] = pre[0]
            pre_dense[-1] = pre[-1]
        else:
            pre_dense = pre
        post = C[exit_idx:]
        dpost = np.linalg.norm(np.diff(post, axis=0), axis=1)
        if len(post) >= 2 and dpost.sum() > 0:
            spost = np.concatenate([[0.0], np.cumsum(dpost)])
            npost = max(2, int(math.ceil(spost[-1] / 0.10)))
            tpost = np.linspace(0.0, 1.0, npost)
            post_dense = np.c_[
                np.interp(tpost, spost / spost[-1], post[:, 0]),
                np.interp(tpost, spost / spost[-1], post[:, 1]),
            ]
        else:
            post_dense = post
        curve = np.vstack([pre_dense, bezier[1:], post_dense[1:]])
    else:
        # Fallback: smooth the whole centerline.
        ds = np.linalg.norm(np.diff(C, axis=0), axis=1)
        u = np.concatenate([[0.0], np.cumsum(ds)])
        u /= u[-1]
        tck, _ = splprep([C[:, 0], C[:, 1]], u=u, s=args.smoothing, k=3)
        uu = np.linspace(0.0, 1.0, 4000)
        xx, yy = splev(uu, tck)
        curve = np.c_[xx, yy]
        curve[0] = C[0]
        curve[-1] = C[-1]

    # Arc length and reconstructed width profile.
    sc = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(curve, axis=0), axis=1))])
    s_end = sc[-1] - sc
    widths = np.empty(len(curve))
    for i, distance in enumerate(s_end):
        if distance >= args.narrow_start + args.narrow_transition:
            widths[i] = args.wide_width
        elif distance <= args.narrow_start:
            widths[i] = args.narrow_width
        else:
            t = (distance - args.narrow_start) / args.narrow_transition
            widths[i] = args.narrow_width + (args.wide_width - args.narrow_width) * smoothstep(t)
    widths[-1] = np.linalg.norm(L[-1] - R[-1])
    widths[0] = np.linalg.norm(L[0] - R[0])

    # Normals and boundaries.
    tangent = np.gradient(curve, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)
    left = curve + normal * (widths / 2.0)[:, None]
    right = curve - normal * (widths / 2.0)[:, None]
    left[0], right[0] = L[0], R[0]
    left[-1], right[-1] = L[-1], R[-1]

    # Blend the first metres into the exact start cross-section.
    start_blend = 1.5
    start_j = 1
    while start_j < len(curve) - 1 and sc[start_j] < start_blend:
        start_j += 1
    if start_j > 1 and sc[start_j] > 1e-6:
        for i in range(1, start_j):
            t = smoothstep(sc[i] / sc[start_j])
            left[i] = (1.0 - t) * left[0] + t * left[start_j]
            right[i] = (1.0 - t) * right[0] + t * right[start_j]
            curve[i] = (left[i] + right[i]) / 2.0

    # Blend the last metres into the exact junction cross-section.  This avoids
    # a backwards segment when the smoothed normal at the penultimate vertex is
    # rotated relative to the existing lane.
    blend_length = 1.5
    junction = len(curve) - 1
    blend_start = junction
    while blend_start > 0 and s_end[blend_start] < blend_length:
        blend_start -= 1
    if blend_start < junction and s_end[blend_start] > 1e-6:
        for i in range(blend_start + 1, len(curve)):
            t = smoothstep((s_end[blend_start] - s_end[i]) / s_end[blend_start])
            left[i] = (1.0 - t) * left[blend_start] + t * left[-1]
            right[i] = (1.0 - t) * right[blend_start] + t * right[-1]
            curve[i] = (left[i] + right[i]) / 2.0

    keep = adaptive_indices(curve, args.min_spacing, args.max_spacing, args.curvature_gain)
    # Avoid segments shorter than the minimum spacing at the two ends.
    while len(keep) >= 2 and np.linalg.norm(curve[keep[-1]] - curve[keep[-2]]) < args.min_spacing:
        keep = np.delete(keep, -2)
    while len(keep) >= 2 and np.linalg.norm(curve[keep[1]] - curve[keep[0]]) < args.min_spacing:
        keep = np.delete(keep, 1)
    print("smoothed vertices:", len(keep), "spacing min/max:",
          round(float(np.min(np.linalg.norm(np.diff(curve[keep], axis=0), axis=1))), 3),
          round(float(np.max(np.linalg.norm(np.diff(curve[keep], axis=0), axis=1))), 3))


    # Fit the similarity transform used by this VMB-exported file.
    to_utm = Transformer.from_crs("EPSG:4326", "EPSG:32650", always_xy=True)
    to_wgs = Transformer.from_crs("EPSG:32650", "EPSG:4326", always_xy=True)
    local_pts, utm_pts = [], []
    for node in root.findall("node"):
        t = tags_of(node)
        if "local_x" in t and node.get("lat") is not None:
            local_pts.append(float(t["local_x"]) + 1j * float(t["local_y"]))
            east, north = to_utm.transform(float(node.get("lon")), float(node.get("lat")))
            utm_pts.append(east + 1j * north)
    local_pts = np.array(local_pts)
    utm_pts = np.array(utm_pts)
    mz, mw = local_pts.mean(), utm_pts.mean()
    a = np.vdot(local_pts - mz, utm_pts - mw) / np.vdot(local_pts - mz, local_pts - mz)
    offset = mw - a * mz
    residual = np.abs(a * local_pts + offset - utm_pts)
    print("lat/lon fit: scale %.9f rotation %.6f deg residual max %.4f m" % (
        abs(a), np.degrees(np.angle(a)), residual.max()))

    def latlon(x, y):
        utm = a * (x + 1j * y) + offset
        lon, lat = to_wgs.transform(utm.real, utm.imag)
        return lat, lon

    max_node_id = max(int(node.get("id")) for node in root.findall("node"))
    next_id = max(982000, max_node_id + 1)

    def make_node(x, y):
        nonlocal next_id
        uid = next_id
        next_id += 1
        lat, lon = latlon(float(x), float(y))
        node = ET.Element("node", {"id": str(uid), "lat": str(lat), "lon": str(lon)})
        for key, value in (("local_x", x), ("local_y", y), ("ele", 0)):
            ET.SubElement(node, "tag", {"k": key, "v": str(value)})
        root.append(node)
        return uid

    new_left_ids, new_right_ids, new_center_ids = [], [], []
    for n, index in enumerate(keep):
        is_first = n == 0
        is_last = n == len(keep) - 1
        if is_first:
            new_left_ids.append(int(left_ids[0]))
            new_right_ids.append(int(right_ids[0]))
            new_center_ids.append(int(center_ids[0]))
            continue
        if is_last:
            new_left_ids.append(int(left_ids[-1]))
            new_right_ids.append(int(right_ids[-1]))
            new_center_ids.append(int(center_ids[-1]))
            continue
        new_left_ids.append(make_node(left[index, 0], left[index, 1]))
        new_right_ids.append(make_node(right[index, 0], right[index, 1]))
        new_center_ids.append(make_node(curve[index, 0], curve[index, 1]))

    set_way_refs(f_left, new_left_ids)
    set_way_refs(f_right, new_right_ids)
    set_way_refs(f_center, new_center_ids)
    set_way_refs(r_left, list(reversed(new_right_ids)))
    set_way_refs(r_right, list(reversed(new_left_ids)))
    set_way_refs(r_center, list(reversed(new_center_ids)))

    # Remove old indoor vertices that are no longer referenced by any way.
    used = set()
    for way in root.findall("way"):
        used.update(way_refs(way))
    removed = 0
    for node in list(root.findall("node")):
        if node.get("id") not in used:
            root.remove(node)
            removed += 1

    # Rebuild the lookups because the new nodes were appended after `nodes`.
    all_nodes = {n.get("id"): n for n in root.findall("node")}

    def coords_of(ids):
        return np.array([
            [float(tags_of(all_nodes[str(i)])["local_x"]),
             float(tags_of(all_nodes[str(i)])["local_y"])]
            for i in ids
        ])

    new_left = coords_of(new_left_ids)
    new_right = coords_of(new_right_ids)
    print("new max boundary turn: %.1f deg" % max(
        turn_angles(new_left).max(), turn_angles(new_right).max()))
    print("new widths first/min/max/last: %.3f %.3f %.3f %.3f" % (
        np.linalg.norm(new_left[0] - new_right[0]),
        np.min(np.linalg.norm(new_left - new_right, axis=1)),
        np.max(np.linalg.norm(new_left - new_right, axis=1)),
        np.linalg.norm(new_left[-1] - new_right[-1]),
    ))

    # Keep OSM element order and pretty formatting.
    for element in root.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        if element.tail is not None and not element.tail.strip():
            element.tail = None
    children = list(root)
    grouped = (
        [el for el in children if el.tag in ("MetaInfo", "meta")]
        + [el for el in children if el.tag == "node"]
        + [el for el in children if el.tag == "way"]
        + [el for el in children if el.tag == "relation"]
    )
    for el in list(root):
        root.remove(el)
    for el in grouped:
        root.append(el)
    for relation in root.findall("relation"):
        members = relation.findall("member")
        tags = relation.findall("tag")
        for child in list(relation):
            relation.remove(child)
        for member in members:
            relation.append(member)
        for tag in tags:
            relation.append(tag)
    ET.indent(root, space="  ")
    xml = '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding="unicode")
    with open(args.output_osm, "w", encoding="UTF-8") as handle:
        handle.write(xml)

    print("removed old vertices:", removed)
    print("nodes %d ways %d relations %d" % (
        len(root.findall("node")), len(root.findall("way")), len(root.findall("relation"))))
    print("saved:", args.output_osm)


if __name__ == "__main__":
    main()

