#!/usr/bin/env python3
"""File-level validation of the rebuilt lanelet2 OSM.

Checks:
  * XML structure, unique ids, all references resolve
  * node lat/lon matches the documented local frame
  * ways: >=2 nodes, no duplicate consecutive nodes, valid tags
  * lanelets: exactly one left/right/centerline, end widths ~2.5 m
  * centerline stays inside the lanelet polygon (sampled)
"""
import math
import sys
import xml.etree.ElementTree as ET

LAT0 = 35.0
LON0 = 139.0
K_LAT = 8.983111665353559e-06
K_LON = 1.0966354197886395e-05


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_segment_distance(p, a, b):
    ab = (b[0] - a[0], b[1] - a[1])
    denom = ab[0] ** 2 + ab[1] ** 2
    if denom < 1e-12:
        return dist(p, a)
    t = ((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / denom
    t = max(0.0, min(1.0, t))
    return dist(p, (a[0] + ab[0] * t, a[1] + ab[1] * t))


def point_in_polygon(p, poly):
    inside = False
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        if (y1 > p[1]) != (y2 > p[1]):
            x = (x2 - x1) * (p[1] - y1) / (y2 - y1) + x1
            if p[0] < x:
                inside = not inside
    return inside


def resample(pts, n):
    total = sum(dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    step = total / (n - 1)
    out = [pts[0]]
    seg, travelled = 0, 0.0
    seg_start = pts[0]
    seg_len = dist(pts[0], pts[1])
    for k in range(1, n - 1):
        target = k * step
        while travelled + seg_len < target and seg + 1 < len(pts) - 1:
            travelled += seg_len
            seg += 1
            seg_start = pts[seg]
            seg_len = dist(pts[seg], pts[seg + 1])
        t = (target - travelled) / seg_len if seg_len > 1e-9 else 0.0
        out.append((seg_start[0] + (pts[seg + 1][0] - seg_start[0]) * t,
                    seg_start[1] + (pts[seg + 1][1] - seg_start[1]) * t))
    out.append(pts[-1])
    return out


def main():
    path = sys.argv[1]
    root = ET.parse(path).getroot()
    problems = []

    nodes = {}
    for n in root.findall('node'):
        nid = n.get('id')
        if nid in nodes:
            problems.append(f'duplicate node id {nid}')
        tags = {t.get('k'): t.get('v') for t in n.findall('tag')}
        nodes[nid] = (float(tags['local_x']), float(tags['local_y']),
                      float(n.get('lat')), float(n.get('lon')))

    ways = {}
    for w in root.findall('way'):
        wid = w.get('id')
        if wid in ways:
            problems.append(f'duplicate way id {wid}')
        refs = [nd.get('ref') for nd in w.findall('nd')]
        tags = {t.get('k'): t.get('v') for t in w.findall('tag')}
        ways[wid] = (refs, tags)

    relations = {}
    for r in root.findall('relation'):
        rid = r.get('id')
        if rid in relations:
            problems.append(f'duplicate relation id {rid}')
        relations[rid] = (
            [(m.get('type'), m.get('ref'), m.get('role'))
             for m in r.findall('member')],
            {t.get('k'): t.get('v') for t in r.findall('tag')})

    for nid, (x, y, lat, lon) in nodes.items():
        elat = LAT0 + y * K_LAT
        elon = LON0 + x * K_LON
        if abs(elat - lat) > 5e-9 or abs(elon - lon) > 5e-9:
            problems.append(f'node {nid}: lat/lon does not match local frame')
    for wid, (refs, tags) in ways.items():
        if len(refs) < 2:
            problems.append(f'way {wid}: fewer than 2 nodes')
        for ref in refs:
            if ref not in nodes:
                problems.append(f'way {wid}: missing node {ref}')
        for i in range(len(refs) - 1):
            if refs[i] == refs[i + 1]:
                problems.append(f'way {wid}: duplicate consecutive node')
        if tags.get('type') != 'line_thin':
            problems.append(f'way {wid}: type != line_thin')
        if tags.get('subtype') not in ('solid', 'dashed'):
            problems.append(f'way {wid}: bad subtype {tags.get("subtype")}')

    print('=== lanelets ===')
    for rid, (members, tags) in relations.items():
        if tags.get('type') != 'lanelet':
            continue
        roles = {}
        for mtype, ref, role in members:
            if mtype != 'way':
                problems.append(f'lanelet {rid}: non-way member')
                continue
            roles.setdefault(role, []).append(ref)
        for role in ('left', 'right', 'centerline'):
            if len(roles.get(role, [])) != 1:
                problems.append(f'lanelet {rid}: role {role} count != 1')
        if any(len(roles.get(r, [])) != 1 for r in ('left', 'right')):
            continue
        left_way = roles['left'][0]
        right_way = roles['right'][0]
        if left_way not in ways or right_way not in ways:
            problems.append(f'lanelet {rid}: boundary way missing')
            continue
        lp = [nodes[r][:2] for r in ways[left_way][0]]
        rp = [nodes[r][:2] for r in ways[right_way][0]]
        w_start = dist(lp[0], rp[0])
        w_end = dist(lp[-1], rp[-1])
        lengths = [dist(lp[i], lp[i + 1]) for i in range(len(lp) - 1)]
        poly = lp + rp[::-1]
        cl_way = roles.get('centerline', [None])[0]
        cl = [nodes[r][:2] for r in ways[cl_way][0]] if cl_way else []

        def cl_ok(p):
            if point_in_polygon(p, poly):
                return True
            return min(point_segment_distance(p, poly[i],
                                              poly[(i + 1) % len(poly)])
                       for i in range(len(poly))) < 0.05

        inside = all(cl_ok(p) for p in cl) if cl else False
        print(f'  {rid} {tags.get("lane_id"):12s} {tags.get("one_way")} '
              f'w_start={w_start:.3f} w_end={w_end:.3f} '
              f'cl_inside={inside}')
        if not (2.0 <= w_start <= 3.2):
            problems.append(f'lanelet {rid}: start width {w_start:.3f}')
        if not (2.0 <= w_end <= 3.2):
            problems.append(f'lanelet {rid}: end width {w_end:.3f}')
        if min(lengths) < 0.02:
            problems.append(f'lanelet {rid}: degenerate left boundary')
        if cl and not inside:
            problems.append(f'lanelet {rid}: centerline leaves polygon')

    print(f'\nnodes={len(nodes)} ways={len(ways)} relations={len(relations)}')
    if problems:
        print(f'\n{len(problems)} PROBLEM(S):')
        for p in problems:
            print(' -', p)
        return 1
    print('\nfile validation: OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())

