#!/usr/bin/env python3
"""Build a 2.5 m bidirectional lanelet map from indoor_route_11201511.

- one parking lap only (duplicate loop removed)
- the main road loop becomes two adjacent 2.5 m lanes with a shared dashed
  divider, so lane changes are possible
- the parking/corridor part is a single bidirectional 2.5 m lane
- curves keep dense vertices
"""
import argparse
import math
import sys

import numpy as np
import xml.etree.ElementTree as ET
from pyproj import Transformer

sys.path.insert(0, "/home/tang/tsy/autoware_tools/bag2lanelet/scripts")
sys.path.insert(0, "/home/tang/autoware_map/map_task4/mapping_tools")
from bag2way import bag2pose  # noqa: E402
from smooth_indoor_lane import adaptive_indices  # noqa: E402

UTM_OFFSET_E = 600000.0
UTM_OFFSET_N = 3500000.0
WIDTH = 2.5
HALF = WIDTH / 2.0


class NodeFactory:
    def __init__(self, root, start_id, to_wgs):
        self.root = root
        self.next_id = start_id
        self.to_wgs = to_wgs

    def make(self, x, y):
        uid = self.next_id
        self.next_id += 1
        lon, lat = self.to_wgs.transform(UTM_OFFSET_E + float(x), UTM_OFFSET_N + float(y))
        node = ET.Element("node", {"id": str(uid), "lat": str(lat), "lon": str(lon)})
        for key, value in (("local_x", x), ("local_y", y), ("ele", 0)):
            ET.SubElement(node, "tag", {"k": key, "v": str(value)})
        self.root.append(node)
        return str(uid)


def normals(points):
    tangent = np.gradient(points, axis=0)
    tangent /= np.maximum(np.linalg.norm(tangent, axis=1, keepdims=True), 1e-12)
    return np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)


def offset_lines(points, normal, offset):
    return points + normal * offset


def make_way(uid, refs, subtype, color=None):
    way = ET.Element("way", {"id": str(uid)})
    for ref in refs:
        ET.SubElement(way, "nd", {"ref": str(ref)})
    ET.SubElement(way, "tag", {"k": "type", "v": "line_thin"})
    ET.SubElement(way, "tag", {"k": "subtype", "v": subtype})
    if color is not None:
        ET.SubElement(way, "tag", {"k": "color", "v": color})
    return way


def make_lanelet(uid, left_ref, right_ref, speed=10):
    rel = ET.Element("relation", {"id": str(uid)})
    ET.SubElement(rel, "member", {"type": "way", "ref": str(left_ref), "role": "left"})
    ET.SubElement(rel, "member", {"type": "way", "ref": str(right_ref), "role": "right"})
    for key, value in (("type", "lanelet"), ("subtype", "road"),
                       ("speed_limit", str(speed)), ("location", "urban"),
                       ("one_way", "yes")):
        ET.SubElement(rel, "tag", {"k": key, "v": value})
    return rel



def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", help="rosbag2 directory")
    parser.add_argument("output", help="output lanelet2 osm")
    parser.add_argument("--min-spacing", type=float, default=0.30)
    parser.add_argument("--max-spacing", type=float, default=1.00)
    args = parser.parse_args()

    poses = bag2pose(args.bag, [0.1, 2.0])
    pos = poses[:, 0:2]
    print("bag poses:", len(pos))

    # Tail: first parking lap (up/top/right side), skip the duplicate bottom
    # side and the whole second lap, then continue from the exit connector.
    part1 = pos[0:1081]
    part2 = pos[2280:3141]
    bridge = np.linspace(part1[-1], part2[0], 21)[1:-1]
    tail_raw = np.vstack([part1, bridge, part2])
    tail_idx = adaptive_indices(tail_raw, args.min_spacing, args.max_spacing, 6.0)
    T = tail_raw[tail_idx].copy()
    print("tail vertices:", len(T))

    # Road loop: from the corridor junction around west U-turn, the main road,
    # east U-turn and back to the corridor junction.
    ring0 = pos[3140:6401]
    ring_raw = ring0
    ring_idx = adaptive_indices(ring_raw, args.min_spacing, args.max_spacing, 6.0)
    R = ring_raw[ring_idx].copy()
    nR = normals(R)
    print("ring vertices:", len(R))

    # Split at the east U-turn bottom so the ring becomes two open lanelets.
    split = int(np.argmin(R[:, 1]))
    if split < 5 or split > len(R) - 5:
        split = len(R) // 2
    print("ring split index:", split, "at", np.round(R[split], 2))

    # Lane offsets: divider = recorded road loop; P = south side, M = north.
    D = R
    P_outer = D + nR * WIDTH
    M_outer = D - nR * WIDTH
    P_center = D + nR * HALF
    M_center = D - nR * HALF

    # Tail ends on the north lane (M) at the ring junction.
    target_end = M_center[0]
    tail_normals = normals(T)
    tail_left = T + tail_normals * HALF
    tail_right = T - tail_normals * HALF
    blend_n = min(12, len(T) - 1)
    for k in range(blend_n):
        i = len(T) - blend_n + k
        f = (k + 1.0) / blend_n
        f = f * f * (3.0 - 2.0 * f)
        T[i] = (1.0 - f) * T[i] + f * target_end
        tail_left[i] = (1.0 - f) * tail_left[i] + f * D[0]
        tail_right[i] = (1.0 - f) * tail_right[i] + f * M_outer[0]
    T[-1] = target_end
    tail_left[-1] = D[0]
    tail_right[-1] = M_outer[0]


    to_wgs = Transformer.from_crs("EPSG:32650", "EPSG:4326", always_xy=True)
    root = ET.Element("osm", {"generator": "route11201511"})
    ET.SubElement(root, "MetaInfo", {"format_version": "1", "map_version": "1"})
    factory = NodeFactory(root, 1000, to_wgs)

    n = len(R)
    d_ids = [factory.make(R[i, 0], R[i, 1]) for i in range(n)]
    p_ids = [factory.make(P_outer[i, 0], P_outer[i, 1]) for i in range(n)]
    m_ids = [factory.make(M_outer[i, 0], M_outer[i, 1]) for i in range(n)]

    tail_l_ids = [factory.make(x, y) for x, y in tail_left]
    tail_r_ids = [factory.make(x, y) for x, y in tail_right]
    tail_l_ids[-1], tail_r_ids[-1] = d_ids[0], m_ids[0]

    way_id = [5000]
    rel_id = [7000]

    def next_way():
        way_id[0] += 1
        return way_id[0] - 1

    def next_rel():
        rel_id[0] += 1
        return rel_id[0] - 1

    def add_way(refs, subtype, color=None):
        uid = next_way()
        root.append(make_way(uid, refs, subtype, color))
        return uid

    def add_lanelet(left, right):
        uid = next_rel()
        root.append(make_lanelet(uid, left, right))
        return uid

    # Ring boundary ways (A = junction -> east U-turn, B = split -> junction).
    dA, dB = d_ids[: split + 1], d_ids[split:]
    pA, pB = p_ids[: split + 1], p_ids[split:]
    mA, mB = m_ids[: split + 1], m_ids[split:]
    ways = {}
    for tag, refs in (("dA", dA), ("dB", dB), ("pA", pA), ("pB", pB), ("mA", mA), ("mB", mB)):
        ways[tag] = add_way(refs, "dashed" if tag.startswith("d") else "solid", "white")
        ways[tag + "_rev"] = add_way(list(reversed(refs)),
                                     "dashed" if tag.startswith("d") else "solid", "white")

    # Ring lanelets: P south lane, M north lane, both directions.
    for seg in ("A", "B"):
        d, dr = ways["d" + seg], ways["d" + seg + "_rev"]
        p, pr = ways["p" + seg], ways["p" + seg + "_rev"]
        m, mr = ways["m" + seg], ways["m" + seg + "_rev"]
        add_lanelet(p, d)    # P forward
        add_lanelet(d, m)    # M forward
        add_lanelet(dr, pr)  # P reverse
        add_lanelet(mr, dr)  # M reverse

    # Tail lane: single bidirectional 2.5 m lane.
    tl = add_way(tail_l_ids, "solid", "white")
    tr = add_way(tail_r_ids, "solid", "white")
    tlr = add_way(list(reversed(tail_l_ids)), "solid", "white")
    trr = add_way(list(reversed(tail_r_ids)), "solid", "white")
    add_lanelet(tl, tr)
    add_lanelet(trr, tlr)

    used = set()
    for way in root.findall("way"):
        used.update(nd.get("ref") for nd in way.findall("nd"))
    for node in list(root.findall("node")):
        if node.get("id") not in used:
            root.remove(node)

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
    with open(args.output, "w", encoding="UTF-8") as handle:
        handle.write(xml)
    print("nodes %d ways %d relations %d" % (
        len(root.findall("node")), len(root.findall("way")), len(root.findall("relation"))))
    print("saved:", args.output)


if __name__ == "__main__":
    main()
