#!/usr/bin/env python3
"""Load the OSM with the lanelet2 python API, validate and inspect routing.

Usage: python3 check_with_lanelet2.py <map.osm>
"""
import sys

import lanelet2
from lanelet2.core import AttributeMap, LaneletMap, getId
from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants, create


def amap(attrs):
    """Convert a lanelet2 AttributeMap to a plain dict."""
    return {k: attrs[k] for k in attrs.keys()}


def main():
    path = sys.argv[1]
    # The nodes carry local_x/local_y tags; project with a local Cartesian
    # projector referenced at the first node so the geometry stays metric.
    origin = Origin(35.0, 139.0)
    projector = LocalCartesianProjector(origin)
    try:
        llmap = load(path, projector)
        print('lanelet2 load: OK')
    except Exception as exc:  # noqa: BLE001
        print(f'lanelet2 load: FAILED -> {exc}')
        raise

    lanelets = list(llmap.laneletLayer)
    print(f'lanelets: {len(lanelets)}')
    for ll in sorted(lanelets, key=lambda x: x.id):
        attrs = amap(ll.attributes)
        print(f'  id={ll.id} lane_id={attrs.get("lane_id")!r} '
              f'one_way={attrs.get("one_way")} '
              f'centerline_pts={len(ll.centerline)} '
             )

    print('\n=== routing graph ===')
    for rules in (Locations.Germany,):
        graph = RoutingGraph(llmap, create(rules, Participants.Vehicle))
        for ll in sorted(lanelets, key=lambda x: x.id):
            nxt = sorted(graph.following(ll), key=lambda x: x.id)
            prev = sorted(graph.previous(ll), key=lambda x: x.id)
            side_l = [x.id for x in graph.lefts(ll)]
            side_r = [x.id for x in graph.rights(ll)]
            print(f'  {ll.id} ({amap(ll.attributes).get("lane_id")}): '
                  f'following={[x.id for x in nxt]} '
                  f'previous={[x.id for x in prev]} '
                  f'lefts={side_l} rights={side_r}')

        print('\n=== reachability (all pairs) ===')
        for src in sorted(lanelets, key=lambda x: x.id):
            reached = set()
            stack = [src]
            while stack:
                cur = stack.pop()
                for nxt in graph.following(cur):
                    if nxt.id not in reached:
                        reached.add(nxt.id)
                        stack.append(nxt)
            print(f'  from {src.id} ({amap(src.attributes).get("lane_id")}): '
                  f'{sorted(reached)}')

    print('\n=== validity ===')
    try:
        from lanelet2.validation import autoware  # noqa: F401
        print('autoware validation module available')
    except Exception:  # noqa: BLE001
        pass


if __name__ == '__main__':
    main()
