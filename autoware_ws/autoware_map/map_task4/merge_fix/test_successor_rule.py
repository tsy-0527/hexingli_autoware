#!/usr/bin/env python3
"""Test what lanelet2 requires for successor detection.

Builds tiny OSM files with two consecutive lanelets:
  A) boundary ways share the junction node ids
  B) boundary ways use duplicated coordinates (distinct node ids)
Checks RoutingGraph.following in both cases.
"""
import os
import tempfile

import lanelet2
from lanelet2.io import Origin, load
from lanelet2.projection import LocalCartesianProjector
from lanelet2.routing import RoutingGraph
from lanelet2.traffic_rules import Locations, Participants, create


TEMPLATE = """<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="topology-test">
  <node id="1" lat="{lat0}" lon="{lon}" visible="true"/>
  <node id="2" lat="{lat1}" lon="{lon}" visible="true"/>
  <node id="3" lat="{lat2}" lon="{lon}" visible="true"/>
  <node id="4" lat="{lat0}" lon="{lon4}" visible="true"/>
  <node id="5" lat="{lat1}" lon="{lon5}" visible="true"/>
  <node id="6" lat="{lat2}" lon="{lon2}" visible="true"/>
{extra_nodes}
  <way id="10" visible="true">
{left_a}
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="solid"/>
  </way>
  <way id="11" visible="true">
{right_a}
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="solid"/>
  </way>
  <way id="12" visible="true">
{left_b}
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="dashed"/>
  </way>
  <way id="13" visible="true">
{right_b}
    <tag k="type" v="line_thin"/>
    <tag k="subtype" v="dashed"/>
  </way>
  <relation id="100" visible="true">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="one_way" v="yes"/>
    <tag k="location" v="urban"/>
  </relation>
  <relation id="101" visible="true">
    <member type="way" ref="12" role="left"/>
    <member type="way" ref="13" role="right"/>
    <tag k="type" v="lanelet"/>
    <tag k="subtype" v="road"/>
    <tag k="one_way" v="yes"/>
    <tag k="location" v="urban"/>
  </relation>
</osm>
"""


def make_case(shared_nodes, path):
    lat0, lat1, lat2 = 35.0, 35.00001, 35.00002
    lon = 139.0
    lon4 = 139.0000125  # 1.25 m at this latitude approx
    lon5 = 139.0000125
    lon2 = 139.0000125
    if shared_nodes:
        left_a = '    <nd ref="1"/>\n    <nd ref="2"/>'
        left_b = '    <nd ref="2"/>\n    <nd ref="3"/>'
        right_a = '    <nd ref="4"/>\n    <nd ref="5"/>'
        right_b = '    <nd ref="5"/>\n    <nd ref="6"/>'
        extra = ''
    else:
        extra = (
            '  <node id="7" lat="%f" lon="%f" visible="true"/>\n'
            '  <node id="8" lat="%f" lon="%f" visible="true"/>\n'
            % (lat1, lon4 * 1.0000001, lat1, lon5 * 1.0000001))
        left_a = '    <nd ref="1"/>\n    <nd ref="2"/>'
        left_b = '    <nd ref="7"/>\n    <nd ref="3"/>'
        right_a = '    <nd ref="4"/>\n    <nd ref="5"/>'
        right_b = '    <nd ref="8"/>\n    <nd ref="6"/>'
    xml = TEMPLATE.format(
        lat0=lat0, lat1=lat1, lat2=lat2, lon=lon, lon4=lon4, lon5=lon5,
        lon2=lon2, extra_nodes=extra, left_a=left_a, right_a=right_a,
        left_b=left_b, right_b=right_b)
    with open(path, 'w') as f:
        f.write(xml)


def check(path):
    llmap = load(path, LocalCartesianProjector(Origin(35.0, 139.0)))
    graph = RoutingGraph(llmap, create(Locations.Germany, Participants.Vehicle))
    lls = {ll.id: ll for ll in llmap.laneletLayer}
    a, b = lls[100], lls[101]
    return ([x.id for x in graph.following(a)],
            [x.id for x in graph.previous(b)])


def main():
    with tempfile.TemporaryDirectory() as tmp:
        for shared in (True, False):
            path = os.path.join(tmp, f'case_{shared}.osm')
            make_case(shared, path)
            try:
                fwd, back = check(path)
                print(f'shared_nodes={shared}: following(A)={fwd} '
                      f'previous(B)={back}')
            except Exception as exc:  # noqa: BLE001
                print(f'shared_nodes={shared}: error {exc}')


if __name__ == '__main__':
    main()
