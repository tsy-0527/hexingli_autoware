#!/usr/bin/env python3
"""Check lanelet2 adjacency requirements for parallel lanelets.

Cases:
  1. two parallel lanelets share the middle way id
  2. two parallel lanelets use different middle ways with identical nodes
Also checks whether a lanelet with two 'left' member ways loads and works.
"""
import os
import tempfile

from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants, create

LAT = 35.0
LON = 139.0
DLAT = 8.983111665353559e-06
DLON = 1.0966354197886395e-05


def node_xml(nid, x, y):
    return (f'  <node id="{nid}" lat="{LAT + y * DLAT:.9f}" '
            f'lon="{LON + x * DLON:.9f}" visible="true"/>\n')


HEADER = '<?xml version="1.0" encoding="UTF-8"?>\n<osm version="0.6">\n'

TAIL = '''  <relation id="100" visible="true">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="{mid_a}" role="right"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="one_way" v="no"/>
    <tag k="location" v="urban"/>
  </relation>
  <relation id="101" visible="true">
    <member type="way" ref="{mid_b}" role="left"/>
    <member type="way" ref="13" role="right"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="one_way" v="no"/>
    <tag k="location" v="urban"/>
  </relation>
</osm>
'''


def build(case, path):
    """case: 'same_way' or 'same_nodes'."""
    nodes = []
    # two parallel lanes along +x, lane A y in [-1.5,-0.0], lane B y in [0,1.5]
    coords = {
        1: (0.0, -1.5), 2: (10.0, -1.5), 3: (20.0, -1.5),
        4: (0.0, 0.0), 5: (10.0, 0.0), 6: (20.0, 0.0),
        7: (0.0, 1.5), 8: (10.0, 1.5), 9: (20.0, 1.5),
    }
    for nid, (x, y) in coords.items():
        nodes.append(node_xml(nid, x, y))
    ways = []
    ways.append('  <way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/>'
                '<tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>\n')
    ways.append('  <way id="11"><nd ref="4"/><nd ref="5"/><nd ref="6"/>'
                '<tag k="type" v="line_thin"/><tag k="subtype" v="dashed"/></way>\n')
    ways.append('  <way id="13"><nd ref="7"/><nd ref="8"/><nd ref="9"/>'
                '<tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>\n')
    if case == 'same_way':
        mid_a = mid_b = '11'
    else:
        ways.append('  <way id="12"><nd ref="4"/><nd ref="5"/><nd ref="6"/>'
                    '<tag k="type" v="line_thin"/>'
                    '<tag k="subtype" v="dashed"/></way>\n')
        mid_a, mid_b = '11', '12'
    with open(path, 'w') as f:
        f.write(HEADER + ''.join(nodes) + ''.join(ways) + TAIL.format(
            mid_a=mid_a, mid_b=mid_b))


def check(path):
    llmap = load(path, LocalCartesianProjector(Origin(LAT, LON)))
    graph = RoutingGraph(llmap, create(Locations.Germany, Participants.Vehicle))
    lls = {ll.id: ll for ll in llmap.laneletLayer}
    a, b = lls[100], lls[101]
    return {
        'lefts(A)': [x.id for x in graph.lefts(a)],
        'rights(A)': [x.id for x in graph.rights(a)],
        'adjacentLefts(A)': [x.id for x in graph.adjacentLefts(a)],
        'lefts(B)': [x.id for x in graph.lefts(b)],
        'rights(B)': [x.id for x in graph.rights(b)],
        'adjacentRights(B)': [x.id for x in graph.adjacentRights(b)],
        'bound_types': (type(a.leftBound).__name__,
                        type(a.rightBound).__name__),
    }


def multi_way_left(path):
    """Lanelet whose left boundary consists of two ways in one relation."""
    nodes = ''.join([
        node_xml(1, 0.0, -1.5), node_xml(2, 10.0, -1.5),
        node_xml(3, 20.0, -1.5), node_xml(4, 0.0, 0.0),
        node_xml(5, 10.0, 0.0), node_xml(6, 20.0, 0.0),
    ])
    ways = (
        '  <way id="10"><nd ref="1"/><nd ref="2"/>'
        '<tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>\n'
        '  <way id="14"><nd ref="2"/><nd ref="3"/>'
        '<tag k="type" v="line_thin"/><tag k="subtype" v="dashed"/></way>\n'
        '  <way id="11"><nd ref="4"/><nd ref="5"/><nd ref="6"/>'
        '<tag k="type" v="line_thin"/><tag k="subtype" v="solid"/></way>\n')
    rel = '''  <relation id="100" visible="true">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="14" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="one_way" v="no"/>
    <tag k="location" v="urban"/>
  </relation>
</osm>
'''
    with open(path, 'w') as f:
        f.write(HEADER + nodes + ways + rel)
    llmap = load(path, LocalCartesianProjector(Origin(LAT, LON)))
    ll = list(llmap.laneletLayer)[0]
    return (type(ll.leftBound).__name__, len(ll.leftBound))


def main():
    with tempfile.TemporaryDirectory() as tmp:
        for case in ('same_way', 'same_nodes'):
            path = os.path.join(tmp, case + '.osm')
            build(case, path)
            try:
                print(case, check(path))
            except Exception as exc:  # noqa: BLE001
                print(case, 'ERROR', exc)
        path = os.path.join(tmp, 'multi.osm')
        try:
            print('multi_left', multi_way_left(path))
        except Exception as exc:  # noqa: BLE001
            print('multi_left ERROR', exc)


if __name__ == '__main__':
    main()
