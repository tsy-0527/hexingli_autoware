#!/usr/bin/env python3
"""Validate the rebuilt lanelet2 map: topology, reachability, overlaps.

Usage: python3 verify_fixed_map.py <map.osm>
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
    by_id = {}
    for ll in lanelets:
        attrs = amap(ll.attributes)
        by_id[ll.id] = ll
        print(f'  {ll.id} lane_id={attrs.get("lane_id")!r} '
              f'one_way={attrs.get("one_way")} '
              f'vehicle={attrs.get("participant:vehicle")} '
              f'left_pts={len(ll.leftBound)} right_pts={len(ll.rightBound)}')

    rules = create(Locations.Germany, Participants.Vehicle)
    graph = RoutingGraph(llmap, rules)

    print('\n=== routing graph ===')
    errors = []
    for ll in lanelets:
        name = amap(ll.attributes).get('lane_id')
        following = sorted(graph.following(ll), key=lambda x: x.id)
        previous = sorted(graph.previous(ll), key=lambda x: x.id)
        lefts = sorted(graph.lefts(ll), key=lambda x: x.id)
        rights = sorted(graph.rights(ll), key=lambda x: x.id)
        print(f'  {ll.id} {name:12s} '
              f'next={[x.attributes["lane_id"] for x in following]} '
              f'prev={[x.attributes["lane_id"] for x in previous]}')
        print(f'  {"":17s} lefts={[x.attributes["lane_id"] for x in lefts]} '
              f'rights={[x.attributes["lane_id"] for x in rights]}')

    print('\n=== lane change permission (dashed shared boundary) ===')
    eo = [ll for ll in lanelets
          if amap(ll.attributes).get('lane_id') == 'east_out'][0]
    ei = [ll for ll in lanelets
          if amap(ll.attributes).get('lane_id') == 'east_in'][0]
    for a, b in ((eo, ei), (ei, eo)):
        try:
            ok = rules.canChangeLane(a, b)
        except Exception as exc:  # noqa: BLE001
            ok = f'ERROR {exc}'
        print(f'  {a.attributes["lane_id"]} -> {b.attributes["lane_id"]}: {ok}')

    print('\n=== reachability (following, both directions) ===')
    for src in lanelets:
        reached = set()
        stack = [src]
        while stack:
            cur = stack.pop()
            for nxt in list(graph.following(cur)) + list(graph.previous(cur)):
                if nxt.id not in reached:
                    reached.add(nxt.id)
                    stack.append(nxt)
        names = sorted(amap(by_id[i].attributes).get('lane_id')
                       for i in reached)
        print(f'  from {amap(src.attributes)["lane_id"]:12s}: {names}')

    print('\n=== overlap check ===')
    bad = []
    for i in range(len(lanelets)):
        for j in range(i + 1, len(lanelets)):
            if overlaps2d(lanelets[i], lanelets[j]):
                bad.append((lanelets[i], lanelets[j]))
    fork_overlaps = []
    if bad:
        for a, b in bad:
            area = overlap_area(a, b)
            fork = bool(endpoint_ids(a) & endpoint_ids(b))
            kind = 'fork/junction overlap' if fork else 'UNEXPECTED overlap'
            print(f'  {kind}: {a.attributes["lane_id"]} <-> '
                  f'{b.attributes["lane_id"]} ~= {area:.2f} m^2')
            if fork:
                fork_overlaps.append((a.attributes['lane_id'],
                                      b.attributes['lane_id'], area))
            else:
                errors.append(f'overlap {a.attributes["lane_id"]}/'
                              f'{b.attributes["lane_id"]}')
    else:
        print('  none')

    print('\n=== route checks ===')
    routes = [
        ('main_w', 'east_out', False),
        ('conn', 'east_in', False),
        ('start', 'uturn', False),
        ('east_out', 'main_w', True),
        ('uturn', 'main_w', True),
    ]
    by_name = {amap(ll.attributes).get('lane_id'): ll for ll in lanelets}
    for src_name, dst_name, inverted in routes:
        src = by_name[src_name]
        dst = by_name[dst_name]
        if inverted:
            src, dst = src.invert(), dst.invert()
        route = graph.getRoute(src, dst)
        if route is None:
            print(f'  {src_name} -> {dst_name}: NO ROUTE')
            errors.append(f'no route {src_name}->{dst_name}')
        else:
            names = [amap(ll.attributes).get('lane_id')
                     for ll in route.shortestPath()]
            print(f'  {src_name} -> {dst_name}: ' + ' -> '.join(names))

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
