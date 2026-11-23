#!/usr/bin/env python3
"""Rebuild lanelet2_map(2).osm with a drivable merge area and full topology.

Problems in the original map
----------------------------
* Every lanelet was topologically isolated: adjacent lanelets only had
  *coincident coordinates*, never shared nodes/ways, so the lanelet2
  routing graph had zero connections.
* The ramp ``conn`` ended where ``east_in`` began, but ``east_in`` was a
  lane floating ~1.7 m north of the main road (``east_out``) with no
  connection at all - the "merge area" could not be driven through.

What this script builds
-----------------------
* Junction nodes are shared, so lanelet2 creates successor relations:
      start -> start_n -> park -> conn -> east_in -> uturn -> east_out -> main_w
      park_loop -> start / start_n        (closed loop)
* ``start`` and ``park`` are split at their T-junctions (lanelet2 requires
  lanelets to start/end at junctions; a lanelet cannot branch mid-way).
* ``main_w`` is extended to the merge cross-section (x ~= 70 m).
* ``east_in`` is rebuilt as the left lane *adjacent* to ``east_out``: both
  share the same dashed boundary way, so vehicles can merge / change lanes
  and both lanes are drivable through the whole merge area.
* The last part of ``conn`` is resampled as a smooth Bezier that joins the
  new merge cross-section tangentially, so the ramp flows into ``east_in``.
* The U-turn mouth is adapted to the new lane end edges.

Output keeps the local_x/local_y design coordinates and the original
lat/lon frame:  lat = 35 + y * 8.983111665353559e-06
                lon = 139 + x * 1.0966354197886395e-05
"""
import math
import sys
import xml.etree.ElementTree as ET

ORIGINAL = '/home/tang/下载/lanelet2_map(2).osm'
OUTPUT = '/home/tang/autoware_map/map_task4/merge_fix/lanelet2_map_merge_fixed.osm'

LAT0 = 35.0
LON0 = 139.0
K_LAT = 8.983111665353559e-06
K_LON = 1.0966354197886395e-05

LANE_WIDTH = 2.5


# --------------------------------------------------------------------------
# loading
# --------------------------------------------------------------------------
def load_map(path):
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.findall('node'):
        tags = {t.get('k'): t.get('v') for t in n.findall('tag')}
        nodes[n.get('id')] = {
            'lat': float(n.get('lat')),
            'lon': float(n.get('lon')),
            'local': (float(tags['local_x']), float(tags['local_y'])),
        }
    ways = {}
    for w in root.findall('way'):
        ways[w.get('id')] = {
            'refs': [nd.get('ref') for nd in w.findall('nd')],
            'tags': {t.get('k'): t.get('v') for t in w.findall('tag')},
        }
    relations = {}
    for rel in root.findall('relation'):
        relations[rel.get('id')] = {
            'members': [(m.get('type'), m.get('ref'), m.get('role'))
                        for m in rel.findall('member')],
            'tags': {t.get('k'): t.get('v') for t in rel.findall('tag')},
        }
    return nodes, ways, relations


def way_points(nodes, ways, wid):
    return [nodes[r]['local'] for r in ways[wid]['refs']]


# --------------------------------------------------------------------------
# 2D geometry helpers (local cartesian frame)
# --------------------------------------------------------------------------
def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def mul(a, s):
    return (a[0] * s, a[1] * s)


def lerp(a, b, t):
    return (a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t)


def length(v):
    return math.hypot(v[0], v[1])


def unit(v):
    n = length(v)
    if n < 1e-9:
        raise ValueError('zero length vector')
    return (v[0] / n, v[1] / n)


def distance(a, b):
    return length(sub(b, a))


def smoothstep(t):
    t = max(0.0, min(1.0, t))
    return t * t * (3.0 - 2.0 * t)


def polyline_length(pts):
    return sum(distance(pts[i], pts[i + 1]) for i in range(len(pts) - 1))


def resample(pts, n):
    """Resample a polyline into n points equally spaced by arc length."""
    if len(pts) < 2:
        raise ValueError('polyline needs at least 2 points')
    total = polyline_length(pts)
    if total < 1e-6:
        raise ValueError('degenerate polyline')
    step = total / (n - 1)
    out = [pts[0]]
    seg = 0
    seg_start = pts[0]
    seg_len = distance(pts[0], pts[1])
    travelled = 0.0
    for k in range(1, n - 1):
        target = k * step
        while travelled + seg_len < target and seg + 1 < len(pts) - 1:
            travelled += seg_len
            seg += 1
            seg_start = pts[seg]
            seg_len = distance(pts[seg], pts[seg + 1])
        t = 0.0 if seg_len < 1e-9 else (target - travelled) / seg_len
        out.append(lerp(seg_start, pts[seg + 1], t))
    out.append(pts[-1])
    return out


def vertex_left_normals(pts):
    """Unit left normals at every vertex (average of adjacent segments)."""
    normals = []
    for i in range(len(pts)):
        seg_normals = []
        if i > 0:
            t = unit(sub(pts[i], pts[i - 1]))
            seg_normals.append((-t[1], t[0]))
        if i < len(pts) - 1:
            t = unit(sub(pts[i + 1], pts[i]))
            seg_normals.append((-t[1], t[0]))
        n = (sum(v[0] for v in seg_normals), sum(v[1] for v in seg_normals))
        normals.append(unit(n))
    return normals


def offset_left(pts, d):
    """Offset a west->east polyline to its left (north) side by d metres."""
    return [add(p, mul(n, d)) for p, n in zip(pts, vertex_left_normals(pts))]


def bezier(p0, p1, p2, p3, n):
    out = []
    for k in range(n):
        t = k / (n - 1)
        mt = 1.0 - t
        p = add(add(mul(p0, mt ** 3), mul(p1, 3 * mt * mt * t)),
                add(mul(p2, 3 * mt * t * t), mul(p3, t ** 3)))
        out.append(p)
    return out


def centerline(left, right, n=60):
    """Centerline as the average of two resampled boundaries."""
    l = resample(left, n)
    r = resample(right, n)
    return [lerp(a, b, 0.5) for a, b in zip(l, r)]


# --------------------------------------------------------------------------
# rebuild geometry
# --------------------------------------------------------------------------
def build_geometry(nodes, all_ways):
    def w(wid):
        return way_points(nodes, all_ways, wid)

    start_l = w('1000')
    start_r = w('1002')
    park_l = w('1003')
    park_r = w('1005')
    conn_l = w('1006')
    conn_c = w('1007')
    conn_r = w('1008')
    main_l = w('1009')
    main_r = w('1011')
    ref = w('1012')       # east_out left bound -> shared middle line
    east_r = w('1014')    # east_out right bound
    ut_l = w('1018')
    ut_r = w('1020')

    assert len(start_l) == len(start_r) == 16
    assert len(park_l) == len(park_r) == 46
    assert len(conn_l) == len(conn_r) == len(conn_c) == 44
    assert len(main_l) == len(main_r) == 3
    assert len(ref) == len(east_r) == 37

    # ---- 1. split start at the park-return cross-section ------------
    J1_l = start_l[4]      # west side of the start lane
    J1_r = start_r[5]      # east side of the start lane
    start_south_l = list(start_l[:5])
    start_south_r = list(start_r[:6])
    start_north_l = list(start_l[4:])
    start_north_r = list(start_r[5:])
    # snap the north part onto park's start edge (shared junction nodes)
    start_north_l[-1] = park_l[0]
    start_north_r[-1] = park_r[0]

    # ---- 2. split park at the ramp branch ---------------------------
    branch_l = conn_r[0]   # on park's west edge
    branch_r = conn_l[0]   # on park's east edge
    park_access_l = list(park_l[:14]) + [branch_l]
    park_access_r = list(park_r[:14]) + [branch_r]
    park_loop_l = [branch_l] + list(park_l[14:])
    park_loop_r = [branch_r] + list(park_r[14:])
    # park returns onto the west/east side of the start lane
    park_loop_l[-1] = J1_r
    park_loop_r[-1] = J1_l

    # ---- 3. two adjacent eastbound lanes ----------------------------
    MERGE_I = 3            # ref[3] ~= (70.051, -79.894)
    END_T = 0.4            # shared boundary stops ~2 m before old end
    M = lerp(ref[35], ref[36], END_T)
    R_end = lerp(east_r[35], east_r[36], END_T)
    shared_mid = list(ref[MERGE_I:36]) + [M]          # ONE way, both lanes
    east_out_r = list(east_r[MERGE_I:36]) + [R_end]
    east_in_l = offset_left(shared_mid, LANE_WIDTH)
    P_S = shared_mid[0]
    C_S = east_in_l[0]
    Pc_S = lerp(P_S, C_S, 0.5)

    # ---- 4. reshape the ramp tail into the merge cross-section ------
    conn_l = list(conn_l)
    conn_r = list(conn_r)
    conn_l[0] = branch_r
    conn_r[0] = branch_l
    TAIL_I = 42            # ramp already heads east here
    p0 = conn_c[TAIL_I]
    p3 = Pc_S
    p1 = (p0[0] + 2.2, p0[1])
    p2 = (p3[0] - 2.6, p3[1] - 0.05)
    tail = bezier(p0, p1, p2, p3, 24)[1:]
    conn_c_new = list(conn_c[:TAIL_I + 1]) + tail
    normals = vertex_left_normals(conn_c_new)
    conn_l_new = list(conn_l[:TAIL_I + 1]) + [
        add(conn_c_new[i], mul(normals[i], LANE_WIDTH / 2))
        for i in range(TAIL_I + 1, len(conn_c_new))]
    conn_r_new = list(conn_r[:TAIL_I + 1]) + [
        add(conn_c_new[i], mul(normals[i], -LANE_WIDTH / 2))
        for i in range(TAIL_I + 1, len(conn_c_new))]
    conn_l_new[-1] = C_S
    conn_r_new[-1] = P_S

    # ---- 5. main_w extended to the merge cross-section --------------
    main_l_new = list(main_l) + [ref[1], ref[2], ref[3]]
    main_r_new = list(main_r) + [east_r[1], east_r[2], east_r[3]]

    # ---- 6. U-turn mouth --------------------------------------------
    L_end = east_in_l[-1]
    ut_l_new = list(ut_l)
    ut_r_new = list(ut_r)
    ut_l_new[0] = M
    ut_l_new[-1] = M
    ut_r_new[0] = R_end
    ut_r_new[-1] = L_end


    polylines = {
        'start.left': (start_south_l, 'solid'),
        'start.right': (start_south_r, 'solid'),
        'start_north.left': (start_north_l, 'solid'),
        'start_north.right': (start_north_r, 'solid'),
        'park.left': (park_access_l, 'solid'),
        'park.right': (park_access_r, 'solid'),
        'park_loop.left': (park_loop_l, 'solid'),
        'park_loop.right': (park_loop_r, 'solid'),
        'conn.left': (conn_l_new, 'solid'),
        'conn.right': (conn_r_new, 'solid'),
        'main_w.left': (main_l_new, 'solid'),
        'main_w.right': (main_r_new, 'solid'),
        'mid': (shared_mid, 'dashed'),
        'east_out.right': (east_out_r, 'solid'),
        'east_in.left': (east_in_l, 'solid'),
        'uturn.left': (ut_l_new, 'solid'),
        'uturn.right': (ut_r_new, 'solid'),
    }

    lanelets = [
        {'id': '2000', 'lane_id': 'start', 'left': 'start.left',
         'right': 'start.right', 'bounds': (start_south_l, start_south_r)},
        {'id': '2007', 'lane_id': 'start_north', 'left': 'start_north.left',
         'right': 'start_north.right',
         'bounds': (start_north_l, start_north_r)},
        {'id': '2001', 'lane_id': 'park', 'left': 'park.left',
         'right': 'park.right', 'bounds': (park_access_l, park_access_r)},
        {'id': '2008', 'lane_id': 'park_loop', 'left': 'park_loop.left',
         'right': 'park_loop.right', 'bounds': (park_loop_l, park_loop_r)},
        {'id': '2002', 'lane_id': 'conn', 'left': 'conn.left',
         'right': 'conn.right', 'bounds': (conn_l_new, conn_r_new)},
        {'id': '2003', 'lane_id': 'main_w', 'left': 'main_w.left',
         'right': 'main_w.right', 'bounds': (main_l_new, main_r_new)},
        {'id': '2004', 'lane_id': 'east_out', 'left': 'mid',
         'right': 'east_out.right', 'bounds': (shared_mid, east_out_r)},
        {'id': '2005', 'lane_id': 'east_in', 'left': 'east_in.left',
         'right': 'mid', 'bounds': (east_in_l, shared_mid)},
        {'id': '2006', 'lane_id': 'uturn', 'left': 'uturn.left',
         'right': 'uturn.right', 'bounds': (ut_l_new, ut_r_new)},
    ]
    return polylines, lanelets


# --------------------------------------------------------------------------
# writer
# --------------------------------------------------------------------------
def coord_key(p):
    return (round(p[0], 4), round(p[1], 4))


def write_map(path, polylines, lanelets, orig_nodes):
    orig_by_coord = {}
    for nid, n in orig_nodes.items():
        orig_by_coord.setdefault(coord_key(n['local']), nid)
    max_orig = max(int(k) for k in orig_nodes)

    nodes_out = {}          # id -> (x, y)
    coord_to_id = {}
    next_id = [max_orig + 1]

    def node_id(p):
        k = coord_key(p)
        if k in coord_to_id:
            return coord_to_id[k]
        nid = orig_by_coord.get(k)
        if nid is None:
            nid = str(next_id[0])
            next_id[0] += 1
        coord_to_id[k] = nid
        nodes_out[nid] = (round(p[0], 4), round(p[1], 4))
        return nid

    # --- boundary ways ---------------------------------------------------
    way_defs = []           # (way_id, [node ids], subtype)
    way_id_by_key = {}
    next_way = [1000]
    for key, (points, subtype) in polylines.items():
        ids = [node_id(p) for p in points]
        if len(ids) < 2:
            raise ValueError(f'way {key} has < 2 points')
        if any(ids[i] == ids[i + 1] for i in range(len(ids) - 1)):
            raise ValueError(f'way {key} contains duplicate consecutive nodes')
        wid = str(next_way[0])
        next_way[0] += 1
        way_id_by_key[key] = wid
        way_defs.append((wid, ids, subtype))

    # --- centerlines -----------------------------------------------------
    center_wid = {}
    for ll in lanelets:
        left, right = ll['bounds']
        cl = centerline(left, right, n=80)
        ids = [node_id(p) for p in cl]
        if any(ids[i] == ids[i + 1] for i in range(len(ids) - 1)):
            ids = [nid for i, nid in enumerate(ids)
                   if i == 0 or nid != ids[i - 1]]
        wid = str(next_way[0])
        next_way[0] += 1
        center_wid[ll['id']] = wid
        way_defs.append((wid, ids, 'dashed'))

    # --- XML -------------------------------------------------------------
    lines = ['<?xml version="1.0" encoding="UTF-8"?>',
             '<osm version="0.6" generator="lanelet2_merge_fixer">',
             '  <meta_info>',
             '    <tag k="format_version" v="0.1"/>',
             '    <tag k="map_version" v="1.0"/>',
             '  </meta_info>']
    for nid in sorted(nodes_out, key=int):
        x, y = nodes_out[nid]
        lat = LAT0 + y * K_LAT
        lon = LON0 + x * K_LON
        lines.append(f'  <node id="{nid}" lat="{lat:.9f}" lon="{lon:.9f}" '
                     f'visible="true">')
        lines.append(f'    <tag k="local_x" v="{x:.4f}"/>')
        lines.append(f'    <tag k="local_y" v="{y:.4f}"/>')
        lines.append('    <tag k="ele" v="0.0"/>')
        lines.append('  </node>')
    for wid, ids, subtype in way_defs:
        lines.append(f'  <way id="{wid}" visible="true">')
        for nid in ids:
            lines.append(f'    <nd ref="{nid}"/>')
        lines.append('    <tag k="type" v="line_thin"/>')
        lines.append(f'    <tag k="subtype" v="{subtype}"/>')
        lines.append('  </way>')
    for ll in lanelets:
        left_wid = way_id_by_key[ll['left']]
        right_wid = way_id_by_key[ll['right']]
        lines.append(f'  <relation id="{ll["id"]}" visible="true">')
        lines.append(f'    <member type="way" ref="{left_wid}" role="left"/>')
        lines.append(f'    <member type="way" ref="{right_wid}" role="right"/>')
        lines.append(f'    <member type="way" ref="{center_wid[ll["id"]]}" '
                     f'role="centerline"/>')
        lines.append('    <tag k="type" v="lanelet"/>')
        lines.append('    <tag k="subtype" v="road"/>')
        lines.append('    <tag k="one_way" v="no"/>')
        lines.append('    <tag k="location" v="urban"/>')
        lines.append('    <tag k="participant:vehicle" v="yes"/>')
        lines.append(f'    <tag k="lane_id" v="{ll["lane_id"]}"/>')
        lines.append('  </relation>')
    lines.append('</osm>')
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    return way_id_by_key, nodes_out


# --------------------------------------------------------------------------
# self checks
# --------------------------------------------------------------------------
def point_segment_distance(p, a, b):
    ab = sub(b, a)
    denom = ab[0] * ab[0] + ab[1] * ab[1]
    if denom < 1e-12:
        return distance(p, a)
    t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / denom
    t = max(0.0, min(1.0, t))
    return distance(p, add(a, mul(ab, t)))


def point_polyline_distance(p, pts):
    return min(point_segment_distance(p, pts[i], pts[i + 1])
               for i in range(len(pts) - 1))


def check_geometry(polylines, lanelets):
    problems = []
    print('\n=== lane width check ===')
    for ll in lanelets:
        left, right = ll['bounds']
        widths = ([point_polyline_distance(p, right) for p in left] +
                  [point_polyline_distance(p, left) for p in right])
        lo = min(widths)
        hi = max(widths)
        print(f'  {ll["lane_id"]:12s} width min={lo:6.3f} max={hi:6.3f} m')
        if lo < 1.8 or hi > 3.6:
            problems.append(f'lane {ll["lane_id"]} width out of range: '
                            f'{lo:.3f}..{hi:.3f}')
    print('\n=== polyline check ===')
    for key, (pts, subtype) in polylines.items():
        if len(pts) < 2:
            problems.append(f'{key}: <2 points')
        for i in range(len(pts) - 1):
            if distance(pts[i], pts[i + 1]) < 1e-4:
                problems.append(f'{key}: duplicate point at {i}')
        if subtype not in ('solid', 'dashed'):
            problems.append(f'{key}: bad subtype {subtype}')
    print(f'  {len(polylines)} boundary polylines checked')
    return problems


# --------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------
def main():
    out_path = sys.argv[1] if len(sys.argv) > 1 else OUTPUT
    nodes, ways, relations = load_map(ORIGINAL)
    polylines, lanelets = build_geometry(nodes, ways)

    problems = check_geometry(polylines, lanelets)
    if problems:
        print('\nGEOMETRY PROBLEMS:')
        for p in problems:
            print(' -', p)
    else:
        print('\ngeometry checks: OK')

    way_id_by_key, nodes_out = write_map(out_path, polylines, lanelets, nodes)

    print(f'\nwritten: {out_path}')
    print(f'nodes={len(nodes_out)} ways={len(way_id_by_key) + len(lanelets)} '
          f'lanelets={len(lanelets)}')
    print('\n=== lanelet map ===')
    for ll in lanelets:
        print(f"  {ll['id']} {ll['lane_id']:12s} left={way_id_by_key[ll['left']]:5s}"
              f" right={way_id_by_key[ll['right']]:5s}")
    print('\n=== key merge coordinates ===')
    for ll in lanelets:
        if ll['lane_id'] in ('conn', 'east_in', 'east_out', 'main_w'):
            left, right = ll['bounds']
            print(f"  {ll['lane_id']:9s} start L={left[0][0]:8.3f},{left[0][1]:8.3f}"
                  f"  R={right[0][0]:8.3f},{right[0][1]:8.3f}")
            print(f"  {'':9s} end   L={left[-1][0]:8.3f},{left[-1][1]:8.3f}"
                  f"  R={right[-1][0]:8.3f},{right[-1][1]:8.3f}")
    if problems:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())





