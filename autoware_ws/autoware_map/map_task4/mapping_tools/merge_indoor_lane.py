#!/usr/bin/env python3
"""Append a recorded indoor route to an existing lanelet2 map.

The recorded trajectory becomes one lanelet.  Its last metres are tapered into
the cross-section of an existing lane and that existing lanelet is split at the
same cross-section, so the new lane is topologically connected through shared
left/right nodes instead of merely overlapping the old map.

The base map used in this workspace is loaded with ``projector_type: Local``
and takes its geometry from ``local_x`` / ``local_y``.  New nodes keep that
convention.  ``lat``/``lon`` are generated exactly like the existing nodes:
UTM zone 50N (EPSG:32650) coordinates minus (600000, 3500000).

Usage:
    python3 merge_indoor_lane.py <bag_dir> <base_osm> <output_osm>
"""
import argparse
import math
import os
import pathlib
import sys

import numpy as np
import xml.etree.ElementTree as ET
from pyproj import Transformer

BAG2LANELET_SCRIPTS = "/home/tang/tsy/autoware_tools/bag2lanelet/scripts"
if BAG2LANELET_SCRIPTS not in sys.path:
    sys.path.insert(0, BAG2LANELET_SCRIPTS)

from bag2way import bag2pose, pose2line  # noqa: E402

# The base map stores local (0, 0) at UTM 50N (600000, 3500000).
UTM_OFFSET_E = 600000.0
UTM_OFFSET_N = 3500000.0

# ID ranges far above the ids used by the current base map (max id ~970415).
NODE_ID_BASE = 981000
WAY_ID_BASE = 981500
REL_ID_BASE = 981900


def get_tags(element):
    return {tag.get("k"): tag.get("v") for tag in element.findall("tag")}


def yaw_from_quaternion(q):
    x, y, z, w = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def rotation(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]])


def smoothstep(t):
    t = min(max(t, 0.0), 1.0)
    return t * t * (3.0 - 2.0 * t)


def make_node(uid, x, y, to_wgs):
    lon, lat = to_wgs.transform(UTM_OFFSET_E + x, UTM_OFFSET_N + y)
    node = ET.Element("node", {"id": str(uid), "lat": str(lat), "lon": str(lon)})
    for key, value in (("local_x", x), ("local_y", y), ("ele", 0.0)):
        ET.SubElement(node, "tag", {"k": key, "v": str(value)})
    return node


def make_way(uid, node_ids, template=None):
    way = ET.Element("way", {"id": str(uid)})
    for node_id in node_ids:
        ET.SubElement(way, "nd", {"ref": str(node_id)})
    if template is not None:
        for tag in template.findall("tag"):
            ET.SubElement(way, "tag", {"k": tag.get("k"), "v": tag.get("v")})
    return way


def build_lane(poses, width, taper_length, junction_left, junction_right,
               narrow_width=None, narrow_start=15.0, narrow_transition=3.0,
               yaw_override=None):
    """Return tapered left/right/center points (Nx2) ending at the junction.

    ``width`` is used for the indoor part.  Within ``narrow_start`` metres of
    the route end the width smoothly changes to ``narrow_width`` (used for the
    fenced horizontal corridor), and the last ``taper_length`` metres blend
    into the cross-section of the existing lane.
    """
    positions = poses[:, 0:2]
    if yaw_override is None:
        yaws = np.array([yaw_from_quaternion(q) for q in poses[:, 3:7]])
    else:
        yaws = np.asarray(yaw_override, dtype=float)

    if narrow_width is None:
        narrow_width = width

    # Arc length from the end of the recorded route.
    seg = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    cumulative = np.concatenate([[0.0], np.cumsum(seg)])
    total = cumulative[-1]
    distance_to_end = total - cumulative

    half_widths = np.empty(len(positions))
    for i, distance in enumerate(distance_to_end):
        if distance <= narrow_start:
            half_widths[i] = narrow_width / 2.0
        elif distance >= narrow_start + narrow_transition or narrow_transition <= 0:
            half_widths[i] = width / 2.0
        else:
            t = (distance - narrow_start) / narrow_transition
            blended = narrow_width + (width - narrow_width) * smoothstep(t)
            half_widths[i] = blended / 2.0

    # Offset of the target cross-section expressed in the final vehicle frame.
    final_rot = rotation(-yaws[-1])
    offset_left = final_rot @ (np.asarray(junction_left) - positions[-1])
    offset_right = final_rot @ (np.asarray(junction_right) - positions[-1])

    left = np.empty_like(positions)
    right = np.empty_like(positions)
    center = positions.copy()
    for i, (p, yaw) in enumerate(zip(positions, yaws)):
        distance = distance_to_end[i]
        f = smoothstep(1.0 - distance / taper_length) if taper_length > 0 else 0.0
        if i == len(positions) - 1:
            f = 1.0
        rot = rotation(yaw)
        blended_left = (1.0 - f) * np.array([0.0, half_widths[i]]) + f * offset_left
        blended_right = (1.0 - f) * np.array([0.0, -half_widths[i]]) + f * offset_right
        left[i] = p + rot @ blended_left
        right[i] = p + rot @ blended_right
        center[i] = (left[i] + right[i]) / 2.0

    # Make the final cross-section exact (no floating point drift).
    left[-1] = junction_left
    right[-1] = junction_right
    center[-1] = (np.asarray(junction_left) + np.asarray(junction_right)) / 2.0
    return left, right, center


def split_lanelet(root, node_xy, ways, lanelet_id, junction_x, route_end_y,
                  node_ids, way_ids, rel_ids, to_wgs):
    """Split a lanelet at x=junction_x and return its two junction node ids."""
    rel = root.find("relation[@id='%s']" % lanelet_id)
    if rel is None:
        raise ValueError("lanelet {} not found".format(lanelet_id))
    members = {m.get("role"): m.get("ref") for m in rel.findall("member")}
    left_way = ways[members["left"]]
    right_way = ways[members["right"]]

    def find_crossing(way):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        points = np.array([node_xy[r] for r in refs])
        candidates = []
        for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
            if min(a[0], b[0]) - 1e-9 <= junction_x <= max(a[0], b[0]) + 1e-9:
                if abs(b[0] - a[0]) < 1e-12:
                    continue
                t = (junction_x - a[0]) / (b[0] - a[0])
                y = a[1] + t * (b[1] - a[1])
                candidates.append((abs(y - route_end_y), i, y))
        if not candidates:
            raise ValueError(
                "way {} does not cross x={:.3f}".format(way.get("id"), junction_x)
            )
        _, index, y = min(candidates)
        return refs, index, y

    left_refs, left_index, left_y = find_crossing(left_way)
    right_refs, right_index, right_y = find_crossing(right_way)
    if left_y <= right_y:
        raise ValueError("unexpected left/right order at the junction")

    left_uid = next(node_ids)
    right_uid = next(node_ids)
    root.append(make_node(left_uid, junction_x, left_y, to_wgs))
    root.append(make_node(right_uid, junction_x, right_y, to_wgs))

    left_west = left_refs[: left_index + 1] + [str(left_uid)]
    left_east = [str(left_uid)] + left_refs[left_index + 1:]
    right_west = right_refs[: right_index + 1] + [str(right_uid)]
    right_east = [str(right_uid)] + right_refs[right_index + 1:]

    # The right boundary of the target lane is only used by this lanelet, so
    # the original way can become the western half.  The left boundary is
    # shared with the neighbouring lane, therefore it is left untouched.
    right_way_id = int(right_way.get("id"))
    west_left_id = next(way_ids)
    east_left_id = next(way_ids)
    east_right_id = next(way_ids)
    east_relation_id = next(rel_ids)

    for nd in list(right_way.findall("nd")):
        right_way.remove(nd)
    for ref in right_west:
        ET.SubElement(right_way, "nd", {"ref": ref})

    root.append(make_way(west_left_id, left_west, template=left_way))
    root.append(make_way(east_left_id, left_east, template=left_way))
    root.append(make_way(east_right_id, right_east, template=right_way))

    for member in list(rel.findall("member")):
        rel.remove(member)
    ET.SubElement(rel, "member", {"type": "way", "ref": str(west_left_id), "role": "left"})
    ET.SubElement(rel, "member", {"type": "way", "ref": str(right_way_id), "role": "right"})

    east_rel = ET.Element("relation", {"id": str(east_relation_id)})
    ET.SubElement(east_rel, "member", {"type": "way", "ref": str(east_left_id), "role": "left"})
    ET.SubElement(east_rel, "member", {"type": "way", "ref": str(east_right_id), "role": "right"})
    for tag in rel.findall("tag"):
        ET.SubElement(east_rel, "tag", {"k": tag.get("k"), "v": tag.get("v")})
    root.append(east_rel)

    return left_uid, right_uid, east_relation_id


def split_opposite_lanelet(root, node_xy, ways, lanelet_id, junction_x, route_end_y,
                           left_node, right_node, way_ids, rel_ids):
    """Split the opposite-direction lanelet at the junction.

    The original westbound lanelet (960110) uses the same boundary nodes as the
    eastbound lane but in reverse order.  Its east part must end at the junction
    so that the reverse indoor lane can be entered from the road.
    """
    rel = root.find("relation[@id='%s']" % lanelet_id)
    if rel is None:
        raise ValueError("lanelet {} not found".format(lanelet_id))
    members = {m.get("role"): m.get("ref") for m in rel.findall("member")}
    left_way = ways[members["left"]]
    right_way = ways[members["right"]]

    def find_crossing(way):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        points = np.array([node_xy[r] for r in refs])
        candidates = []
        for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
            if min(a[0], b[0]) - 1e-9 <= junction_x <= max(a[0], b[0]) + 1e-9:
                if abs(b[0] - a[0]) < 1e-12:
                    continue
                t = (junction_x - a[0]) / (b[0] - a[0])
                y = a[1] + t * (b[1] - a[1])
                candidates.append((abs(y - route_end_y), i))
        if not candidates:
            raise ValueError(
                "way {} does not cross x={:.3f}".format(way.get("id"), junction_x)
            )
        return refs, min(candidates)[1]

    left_refs, left_index = find_crossing(left_way)
    right_refs, right_index = find_crossing(right_way)

    left_east = left_refs[: left_index + 1] + [str(left_node)]
    left_west = [str(left_node)] + left_refs[left_index + 1:]
    right_east = right_refs[: right_index + 1] + [str(right_node)]
    right_west = [str(right_node)] + right_refs[right_index + 1:]

    # The left boundary is used only by this lanelet, so the original way can
    # become its western half.  The right boundary is shared with the next
    # lanelet and is left untouched.
    left_way_id = int(left_way.get("id"))
    east_left_id = next(way_ids)
    west_right_id = next(way_ids)
    east_right_id = next(way_ids)
    east_relation_id = next(rel_ids)

    for nd in list(left_way.findall("nd")):
        left_way.remove(nd)
    for ref in left_west:
        ET.SubElement(left_way, "nd", {"ref": ref})

    root.append(make_way(east_left_id, left_east, template=left_way))
    root.append(make_way(west_right_id, right_west, template=right_way))
    root.append(make_way(east_right_id, right_east, template=right_way))

    for member in list(rel.findall("member")):
        rel.remove(member)
    ET.SubElement(rel, "member", {"type": "way", "ref": str(left_way_id), "role": "left"})
    ET.SubElement(rel, "member", {"type": "way", "ref": str(west_right_id), "role": "right"})

    east_rel = ET.Element("relation", {"id": str(east_relation_id)})
    ET.SubElement(east_rel, "member", {"type": "way", "ref": str(east_left_id), "role": "left"})
    ET.SubElement(east_rel, "member", {"type": "way", "ref": str(east_right_id), "role": "right"})
    for tag in rel.findall("tag"):
        ET.SubElement(east_rel, "tag", {"k": tag.get("k"), "v": tag.get("v")})
    root.append(east_rel)

    return east_relation_id


def add_reverse_lane(root, forward_relation_id, way_ids, rel_ids):
    """Create the opposite-direction lanelet over the same indoor geometry."""
    rel = root.find("relation[@id='%s']" % forward_relation_id)
    members = {m.get("role"): m.get("ref") for m in rel.findall("member")}
    all_ways = {w.get("id"): w for w in root.findall("way")}
    left_way = all_ways[members["left"]]
    right_way = all_ways[members["right"]]
    center_way = all_ways[members["centerline"]]

    left_refs = [nd.get("ref") for nd in left_way.findall("nd")]
    right_refs = [nd.get("ref") for nd in right_way.findall("nd")]
    center_refs = [nd.get("ref") for nd in center_way.findall("nd")]

    reverse_left_id = next(way_ids)
    reverse_right_id = next(way_ids)
    reverse_center_id = next(way_ids)
    reverse_relation_id = next(rel_ids)

    # Driving back towards the indoor start: left/right sides are exchanged and
    # the node order is reversed.  All nodes are shared with the forward lane.
    root.append(make_way(reverse_left_id, list(reversed(right_refs)), template=right_way))
    root.append(make_way(reverse_right_id, list(reversed(left_refs)), template=left_way))
    root.append(make_way(reverse_center_id, list(reversed(center_refs)), template=center_way))

    reverse = ET.Element("relation", {"id": str(reverse_relation_id)})
    ET.SubElement(reverse, "member", {"type": "way", "ref": str(reverse_left_id), "role": "left"})
    ET.SubElement(reverse, "member", {"type": "way", "ref": str(reverse_right_id), "role": "right"})
    ET.SubElement(
        reverse, "member", {"type": "way", "ref": str(reverse_center_id), "role": "centerline"}
    )
    for tag in rel.findall("tag"):
        ET.SubElement(reverse, "tag", {"k": tag.get("k"), "v": tag.get("v")})
    root.append(reverse)

    return reverse_relation_id


def angdiff(a, b):
    return abs((a - b + math.pi) % (2.0 * math.pi) - math.pi)


def smooth_u_turn(poses, threshold_deg=15.0):
    """Replace the recorded U-turn with a C1 cubic Bezier.

    The entry and exit are detected from the heading relative to the final
    heading and its opposite, so the straight sections and the junction
    approach stay unchanged.  The Bezier is fitted to the recorded interior
    points while matching the entry/exit tangents.
    """
    positions = poses[:, 0:2].astype(float).copy()
    yaws = np.array([yaw_from_quaternion(q) for q in poses[:, 3:7]])
    threshold = math.radians(threshold_deg)
    final = yaws[-1]
    opposite = final + math.pi

    # Start of the final straight.
    exit_idx = len(positions) - 1
    while exit_idx > 0 and angdiff(yaws[exit_idx - 1], final) < threshold:
        exit_idx -= 1
    # Last pose of the straight entering the U-turn.
    entry_idx = None
    for i in range(exit_idx - 1, -1, -1):
        if angdiff(yaws[i], opposite) < threshold:
            entry_idx = i
            break
    if entry_idx is None or exit_idx - entry_idx < 3:
        return positions, yaws, None

    p0 = positions[entry_idx]
    p3 = positions[exit_idx]
    t0 = positions[entry_idx] - positions[entry_idx - 1]
    t0 /= np.linalg.norm(t0)
    if exit_idx + 1 < len(positions):
        t3 = positions[exit_idx + 1] - positions[exit_idx]
    else:
        t3 = np.array([math.cos(yaws[exit_idx]), math.sin(yaws[exit_idx])])
    t3 /= np.linalg.norm(t3)

    interior = positions[entry_idx + 1:exit_idx]
    pts = np.vstack([p0, interior, p3])
    chord = np.concatenate(
        [[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
    )
    if chord[-1] <= 0:
        return positions, yaws, None
    t_int = chord[1:-1] / chord[-1]

    # Least-squares fit of a,b in P1=P0+a*t0, P2=P3-b*t3.
    matrix = []
    rhs = []
    for t, point in zip(t_int, interior):
        b1 = (1 - t) ** 3 + 3 * (1 - t) ** 2 * t
        b2 = 3 * (1 - t) * t ** 2 + t ** 3
        base = b1 * p0 + b2 * p3
        ca = 3 * (1 - t) ** 2 * t
        cb = -3 * (1 - t) * t ** 2
        matrix.append(np.array([[ca * t0[0], cb * t3[0]], [ca * t0[1], cb * t3[1]]]))
        rhs.append(point - base)
    if not matrix:
        return positions, yaws, None
    sol, *_ = np.linalg.lstsq(np.vstack(matrix), np.hstack(rhs), rcond=None)
    a, b = sol
    if not (np.isfinite(a) and np.isfinite(b)) or a <= 0 or b <= 0:
        a = b = 0.5 * np.linalg.norm(p3 - p0)
    p1 = p0 + a * t0
    p2 = p3 - b * t3

    for k, t in enumerate(t_int):
        mt = 1.0 - t
        point = mt**3 * p0 + 3 * mt**2 * t * p1 + 3 * mt * t**2 * p2 + t**3 * p3
        deriv = (
            3 * mt**2 * (p1 - p0)
            + 6 * mt * t * (p2 - p1)
            + 3 * t**2 * (p3 - p2)
        )
        positions[entry_idx + 1 + k] = point
        yaws[entry_idx + 1 + k] = math.atan2(deriv[1], deriv[0])

    return positions, yaws, (entry_idx, exit_idx)


def adaptive_indices(positions, yaws, min_spacing=0.12, max_spacing=0.45, gain=6.0):
    """Select vertices with a smaller spacing in curves than on straights."""
    ds = np.linalg.norm(np.diff(positions, axis=0), axis=1)
    if len(ds) < 3:
        return np.arange(len(positions))

    def moving_average(values, window):
        window = max(1, int(window))
        if window <= 1:
            return values
        pad = window // 2
        padded = np.pad(values, pad, mode="edge")
        return np.convolve(padded, np.ones(window) / window, mode="valid")

    kappa = np.abs(np.diff(moving_average(np.unwrap(yaws), 9))) / np.maximum(ds, 1e-6)
    kappa = moving_average(kappa, 13)
    spacing = np.clip(max_spacing / (1.0 + gain * kappa), min_spacing, max_spacing)

    keep = [0]
    travelled = 0.0
    for i in range(1, len(positions)):
        travelled += ds[i - 1]
        if travelled >= spacing[min(i - 1, len(spacing) - 1)]:
            keep.append(i)
            travelled = 0.0
    if keep[-1] != len(positions) - 1:
        keep.append(len(positions) - 1)
    return np.array(keep)


def main():
    global UTM_OFFSET_E, UTM_OFFSET_N

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="recorded rosbag2 directory")
    parser.add_argument("base_osm", help="existing lanelet2 map")
    parser.add_argument("output_osm", help="output lanelet2 map")
    parser.add_argument("-l", "--width", type=float, default=2.5, help="lane width [m]")
    parser.add_argument(
        "--taper-length",
        type=float,
        default=5.0,
        help="length over which the lane narrows into the existing lane [m]",
    )
    parser.add_argument(
        "--narrow-width",
        type=float,
        default=1.0,
        help="lane width in the fenced horizontal corridor [m]",
    )
    parser.add_argument(
        "--narrow-start",
        type=float,
        default=15.0,
        help="distance from the route end where the corridor starts [m]",
    )
    parser.add_argument(
        "--narrow-transition",
        type=float,
        default=3.0,
        help="transition length between the indoor width and corridor width [m]",
    )
    parser.add_argument(
        "--connect-lanelet",
        default="60110",
        help="existing eastbound lanelet that the new lane merges into",
    )
    parser.add_argument(
        "--bidirectional",
        action="store_true",
        help="also add the reverse lanelet and split the opposite road lane (960110)",
    )
    parser.add_argument(
        "--smooth-u-turn",
        action="store_true",
        help="fit the recorded U-turn with a smooth cubic Bezier",
    )
    parser.add_argument(
        "--pose-step",
        type=int,
        default=10,
        help="use every Nth pose from the bag (1 = all poses)",
    )
    parser.add_argument(
        "--curve-adaptive",
        action="store_true",
        help="keep more vertices in curves and fewer on straights",
    )
    parser.add_argument("--min-spacing", type=float, default=0.12,
                        help="minimum vertex spacing in curves [m]")
    parser.add_argument("--max-spacing", type=float, default=0.45,
                        help="maximum vertex spacing on straights [m]")
    parser.add_argument("--curvature-gain", type=float, default=6.0,
                        help="how strongly the vertex spacing follows curvature")
    parser.add_argument(
        "--utm-epsg", type=int, default=32650, help="UTM CRS of the base map lat/lon"
    )
    parser.add_argument(
        "--utm-offset",
        type=float,
        nargs=2,
        default=[UTM_OFFSET_E, UTM_OFFSET_N],
        help="UTM easting/northing of local (0, 0)",
    )
    args = parser.parse_args()

    if not os.path.isdir(args.bag):
        raise FileNotFoundError("bag directory not found: {}".format(args.bag))

    UTM_OFFSET_E, UTM_OFFSET_N = args.utm_offset
    to_wgs = Transformer.from_crs(
        "EPSG:{}".format(args.utm_epsg), "EPSG:4326", always_xy=True
    )

    root = ET.parse(args.base_osm).getroot()
    node_xy = {}
    for node in root.findall("node"):
        tags = get_tags(node)
        if "local_x" in tags and "local_y" in tags:
            node_xy[node.get("id")] = (float(tags["local_x"]), float(tags["local_y"]))
    ways = {way.get("id"): way for way in root.findall("way")}

    def id_gen(start):
        value = start
        while True:
            yield value
            value += 1

    max_node_id = max(int(node.get("id")) for node in root.findall("node"))
    max_way_id = max(int(way.get("id")) for way in root.findall("way"))
    max_rel_id = max(int(rel.get("id")) for rel in root.findall("relation"))
    node_ids = id_gen(max(NODE_ID_BASE, max_node_id + 1000))
    way_ids = id_gen(max(WAY_ID_BASE, max_way_id + 1000))
    rel_ids = id_gen(max(REL_ID_BASE, max_rel_id + 1000))

    poses = bag2pose(str(pathlib.Path(args.bag)), [0.1, 2.0])
    if len(poses) == 0:
        raise RuntimeError("no map -> base_link pose found in the bag")
    poses = poses[:: args.pose_step] if args.pose_step > 1 else poses
    raw_count = len(poses)
    positions = poses[:, 0:2].astype(float).copy()
    yaws = np.array([yaw_from_quaternion(q) for q in poses[:, 3:7]])

    if args.smooth_u_turn:
        positions, yaws, smooth_info = smooth_u_turn(poses)
        if smooth_info:
            print("smoothed U-turn between poses {}..{}".format(*smooth_info))

    if args.curve_adaptive:
        keep = adaptive_indices(
            positions,
            yaws,
            min_spacing=args.min_spacing,
            max_spacing=args.max_spacing,
            gain=args.curvature_gain,
        )
        positions = positions[keep]
        yaws = yaws[keep]
        print(
            "adaptive vertices: {} -> {} (min/max spacing {:.2f}/{:.2f} m)".format(
                raw_count, len(keep), args.min_spacing, args.max_spacing
            )
        )

    lane_poses = np.zeros((len(positions), 7))
    lane_poses[:, 0:2] = positions
    yaw_override = yaws

    _, _, center_raw = pose2line(poses, width=args.width, offset=[0.0, 0.0, 0.0])
    route_end_x = float(center_raw[-1][0])
    route_end_y = float(center_raw[-1][1])

    rel = root.find("relation[@id='%s']" % args.connect_lanelet)
    if rel is None:
        raise ValueError("connect lanelet {} not found".format(args.connect_lanelet))
    members = {m.get("role"): m.get("ref") for m in rel.findall("member")}
    left_way = ways[members["left"]]
    right_way = ways[members["right"]]

    def crossing_y(way):
        refs = [nd.get("ref") for nd in way.findall("nd")]
        points = np.array([node_xy[ref] for ref in refs])
        candidates = []
        for i, (a, b) in enumerate(zip(points[:-1], points[1:])):
            if min(a[0], b[0]) - 1e-9 <= route_end_x <= max(a[0], b[0]) + 1e-9:
                t = (route_end_x - a[0]) / (b[0] - a[0])
                y = a[1] + t * (b[1] - a[1])
                candidates.append((abs(y - route_end_y), y))
        if not candidates:
            raise ValueError(
                "way {} does not cross x={:.3f}".format(way.get("id"), route_end_x)
            )
        return min(candidates)[1]

    junction_left = (route_end_x, crossing_y(left_way))
    junction_right = (route_end_x, crossing_y(right_way))
    if junction_left[1] <= junction_right[1]:
        raise ValueError("unexpected left/right order at the junction")

    left, right, center = build_lane(
        lane_poses,
        args.width,
        args.taper_length,
        junction_left,
        junction_right,
        narrow_width=args.narrow_width,
        narrow_start=args.narrow_start,
        narrow_transition=args.narrow_transition,
        yaw_override=yaw_override,
    )

    left_jid, right_jid, east_relation_id = split_lanelet(
        root,
        node_xy,
        ways,
        args.connect_lanelet,
        route_end_x,
        route_end_y,
        node_ids,
        way_ids,
        rel_ids,
        to_wgs,
    )

    left_node_ids, right_node_ids, center_node_ids = [], [], []
    for i in range(len(left)):
        if i == len(left) - 1:
            left_node_ids.append(left_jid)
            right_node_ids.append(right_jid)
        else:
            lid = next(node_ids)
            rid = next(node_ids)
            root.append(make_node(lid, float(left[i, 0]), float(left[i, 1]), to_wgs))
            root.append(make_node(rid, float(right[i, 0]), float(right[i, 1]), to_wgs))
            left_node_ids.append(lid)
            right_node_ids.append(rid)
        cid = next(node_ids)
        root.append(make_node(cid, float(center[i, 0]), float(center[i, 1]), to_wgs))
        center_node_ids.append(cid)

    left_way_id = next(way_ids)
    right_way_id = next(way_ids)
    center_way_id = next(way_ids)
    lanelet_id = next(rel_ids)

    def boundary_template(subtype, color=None):
        element = ET.Element("way")
        ET.SubElement(element, "tag", {"k": "type", "v": "line_thin"})
        ET.SubElement(element, "tag", {"k": "subtype", "v": subtype})
        if color is not None:
            ET.SubElement(element, "tag", {"k": "color", "v": color})
        return element

    root.append(make_way(left_way_id, left_node_ids, boundary_template("solid", "white")))
    root.append(make_way(right_way_id, right_node_ids, boundary_template("solid", "white")))
    root.append(make_way(center_way_id, center_node_ids, boundary_template("solid")))

    lanelet = ET.Element("relation", {"id": str(lanelet_id)})
    ET.SubElement(lanelet, "member", {"type": "way", "ref": str(left_way_id), "role": "left"})
    ET.SubElement(lanelet, "member", {"type": "way", "ref": str(right_way_id), "role": "right"})
    ET.SubElement(
        lanelet, "member", {"type": "way", "ref": str(center_way_id), "role": "centerline"}
    )
    for key, value in (
        ("type", "lanelet"),
        ("subtype", "road"),
        ("speed_limit", "10"),
        ("location", "urban"),
        ("one_way", "yes"),
    ):
        ET.SubElement(lanelet, "tag", {"k": key, "v": value})
    root.append(lanelet)

    reverse_lanelet_id = None
    opposite_east_relation_id = None
    if args.bidirectional:
        # Make the new indoor lane two-way: split the westbound road lane at
        # the same junction and add a reverse lanelet over the same geometry.
        opposite_east_relation_id = split_opposite_lanelet(
            root,
            node_xy,
            ways,
            "960110",
            route_end_x,
            route_end_y,
            # Driving west, the left side is the southern junction node.
            right_jid,
            left_jid,
            way_ids,
            rel_ids,
        )
        reverse_lanelet_id = add_reverse_lane(
            root, lanelet_id, way_ids, rel_ids
        )

    # Drop whitespace-only text/tail left by parsing the pretty-printed input,
    # then indent the complete document once.
    for element in root.iter():
        if element.text is not None and not element.text.strip():
            element.text = None
        if element.tail is not None and not element.tail.strip():
            element.tail = None
    ET.indent(root, space="  ")
    # Keep the conventional OSM order: MetaInfo, nodes, ways, relations.
    children = list(root)
    head = [el for el in children if el.tag in ("MetaInfo", "meta")]
    grouped = (
        head
        + [el for el in children if el.tag == "node"]
        + [el for el in children if el.tag == "way"]
        + [el for el in children if el.tag == "relation"]
    )
    for el in children:
        root.remove(el)
    for el in grouped:
        root.append(el)
    ET.indent(root, space="  ")

    # OSM convention: relation members before tags.
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
    xml = b'<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding="UTF-8")
    output_path = os.path.abspath(args.output_osm)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "wb") as handle:
        handle.write(xml)

    print("route end: ({:.4f}, {:.4f})".format(route_end_x, route_end_y))
    print(
        "junction left/right: ({:.4f}, {:.4f}) / ({:.4f}, {:.4f})".format(
            junction_left[0],
            junction_left[1],
            junction_right[0],
            junction_right[1],
        )
    )
    print("new lanelet id: {}".format(lanelet_id))
    print(
        "split lanelet {} -> west {} / east {}".format(
            args.connect_lanelet, args.connect_lanelet, east_relation_id
        )
    )
    print("new nodes: {}".format(3 * len(left)))
    if args.bidirectional:
        print(
            "bidirectional: reverse lanelet {} ; split 960110 -> west 960110 / east {}".format(
                reverse_lanelet_id, opposite_east_relation_id
            )
        )
    print("saved: {}".format(output_path))


if __name__ == "__main__":
    main()

