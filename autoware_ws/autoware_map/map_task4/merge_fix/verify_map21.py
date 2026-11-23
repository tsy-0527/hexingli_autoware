#!/usr/bin/env python3
"""Validate lanelet2_map(21) rebuilt map: topology, Y merge/exit, overlaps.

Usage: python3 verify_map21.py <map.osm>
"""
import sys

from lanelet2.core import BasicPoint2d
from lanelet2.geometry import inside, overlaps2d
from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants, create


def amap(attrs):
    return {k: attrs[k] for k in attrs.keys()}


def lname(ll):
    return amap(ll.attributes).get('lane_id') or f'lane_{ll.id}'


def bbox(ll):
    pts = [(p.x, p.y) for p in ll.leftBound] + \
          [(p.x, p.y) for p in ll.rightBound]
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return min(xs), max(xs), min(ys), max(ys)


def overlap_area(a, b, step=0.1):
    x0, x1, y0, y1 = bbox(a)
    x0b, x1b, y0b, y1b = bbox(b)
    x0, x1 = max(x0, x0b), min(x1, x1b)
    y0, y1 = max(y0, y0b), min(y1, y1b)
    if x1 <= x0 or y1 <= y0:
        return 0.0
    hits = 0
    y = y0 + step / 2
    while y < y1:
        x = x0 + step / 2
        while x < x1:
            p = BasicPoint2d(x, y)
            if inside(a, p) and inside(b, p):
                hits += 1
            x += step
        y += step
    return hits * step * step


def endpoint_ids(ll):
    return {ll.leftBound[0].id, ll.leftBound[-1].id,
            ll.rightBound[0].id, ll.rightBound[-1].id}


def main():
    path = sys.argv[1]
    llmap = load(path, LocalCartesianProjector(Origin(35.0, 139.0)))
    lanelets = sorted(llmap.laneletLayer, key=lambda x: x.id)
    print(f'loaded {path}: {len(lanelets)} lanelets')
    print('\n=== lanelets ===')
    for ll in lanelets:
        print(f'  {ll.id} {lname(ll):12s} '
              f'one_way={amap(ll.attributes).get("one_way")} '
              f'left_pts={len(ll.leftBound)} right_pts={len(ll.rightBound)}')

    rules = create(Locations.Germany, Participants.Vehicle)
    graph = RoutingGraph(llmap, rules)
    errors = []

    print('\n=== routing graph ===')
    for ll in lanelets:
        nxt = sorted(graph.following(ll), key=lambda x: x.id)
        prv = sorted(graph.previous(ll), key=lambda x: x.id)
        lft = sorted(graph.lefts(ll), key=lambda x: x.id)
        rgt = sorted(graph.rights(ll), key=lambda x: x.id)
        print(f'  {ll.id} {lname(ll):12s} '
              f'next={[lname(x) for x in nxt]} prev={[lname(x) for x in prv]}')
        print(f'  {"":17s} lefts={[lname(x) for x in lft]} '
              f'rights={[lname(x) for x in rgt]}')

    print('\n=== reachability (following + previous) ===')
    for src in lanelets:
        reached = set()
        stack = [src]
        while stack:
            cur = stack.pop()
            for nxt in list(graph.following(cur)) + list(graph.previous(cur)):
                if nxt.id not in reached:
                    reached.add(nxt.id)
                    stack.append(nxt)
        names = sorted(lname(x) for x in lanelets if x.id in reached)
        print(f'  from {lname(src):12s}: {names}')
        if len(reached | {src.id}) != len(lanelets):
            errors.append(f'{lname(src)} cannot reach all lanelets')

    print('\n=== route checks (Y merge / exit, both directions) ===')
    by_name = {lname(ll): ll for ll in lanelets}
    routes = [
        ('main_w', 'east_out', False, False),     # through at x~50
        ('main_w', 'main_link', False, False),    # branch to Y
        ('main_link', 'east_in', False, False),   # merge from west
        ('conn', 'east_in', False, False),        # merge from north
        ('east_in', 'main_link', True, True),     # exit to west link
        ('east_in', 'conn', True, True),          # exit to north ramp
        ('east_out', 'main_w', True, True),       # reverse through
        ('east_in', 'uturn', False, True),        # to U-turn (westbound)
        ('uturn', 'east_out', True, True),         # U-turn back to east_out
        ('start', 'east_in', False, False),
    ]
    for src_name, dst_name, inv_src, inv_dst in routes:
        src, dst = by_name[src_name], by_name[dst_name]
        if inv_src:
            src = src.invert()
        if inv_dst:
            dst = dst.invert()
        route = graph.getRoute(src, dst)
        label = f'{src_name} -> {dst_name}'
        if route is None:
            print(f'  {label}: NO ROUTE')
            errors.append(f'no route {label}')
        else:
            names = [lname(ll) for ll in route.shortestPath()]
            print(f'  {label}: ' + ' -> '.join(names))

    print('\n=== overlap check ===')
    bad = []
    for i in range(len(lanelets)):
        for j in range(i + 1, len(lanelets)):
            if overlaps2d(lanelets[i], lanelets[j]):
                bad.append((lanelets[i], lanelets[j]))
    fork_overlaps = []
    for a, b in bad:
        area = overlap_area(a, b)
        fork = bool(endpoint_ids(a) & endpoint_ids(b))
        kind = 'fork/junction overlap' if fork else 'UNEXPECTED overlap'
        print(f'  {kind}: {lname(a)} <-> {lname(b)} ~= {area:.2f} m^2')
        if fork:
            fork_overlaps.append((lname(a), lname(b), area))
        else:
            errors.append(f'overlap {lname(a)}/{lname(b)}')
    if not bad:
        print('  none')

    print('\n=== routing graph validity ===')
    try:
        issues = graph.checkValidity(False)
    except Exception as exc:  # noqa: BLE001
        print(f'  checkValidity failed: {exc}')
        issues = []
    if issues:
        for issue in issues:
            print('  ISSUE:', issue)
        errors.extend(issues)
    else:
        print('  OK')

    print('\n=== summary ===')
    if fork_overlaps:
        print('  inherent lanelet2 fork overlaps (info only):')
        for a, b, area in fork_overlaps:
            print(f'    {a} <-> {b}: {area:.2f} m^2')
    if errors:
        print(f'  {len(errors)} problem(s) found')
        return 1
    print('  all checks passed')
    return 0


if __name__ == '__main__':
    sys.exit(main())

