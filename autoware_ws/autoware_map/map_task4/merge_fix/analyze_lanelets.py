#!/usr/bin/env python3
"""Dump lanelet2 map geometry / topology from an OSM file (offline analysis).

Usage: python3 analyze_lanelets.py <map.osm>
"""
import sys
import xml.etree.ElementTree as ET


def load(path):
    root = ET.parse(path).getroot()
    nodes = {}
    for n in root.findall('node'):
        tags = {t.get('k'): t.get('v') for t in n.findall('tag')}
        nodes[n.get('id')] = {
            'lat': float(n.get('lat')),
            'lon': float(n.get('lon')),
            'local': (float(tags['local_x']), float(tags['local_y']))
            if 'local_x' in tags and 'local_y' in tags else None,
        }
    ways = {}
    for w in root.findall('way'):
        ways[w.get('id')] = {
            'refs': [nd.get('ref') for nd in w.findall('nd')],
            'tags': {t.get('k'): t.get('v') for t in w.findall('tag')},
        }
    relations = []
    for rel in root.findall('relation'):
        relations.append({
            'id': rel.get('id'),
            'members': [(m.get('type'), m.get('ref'), m.get('role'))
                        for m in rel.findall('member')],
            'tags': {t.get('k'): t.get('v') for t in rel.findall('tag')},
        })
    return nodes, ways, relations


def way_pts(way, nodes):
    pts = []
    for r in way['refs']:
        p = nodes[r]['local']
        pts.append(p if p is not None else (nodes[r]['lat'], nodes[r]['lon']))
    return pts


def main():
    path = sys.argv[1]
    nodes, ways, relations = load(path)
    print(f'file: {path}')
    print(f'nodes={len(nodes)} ways={len(ways)} relations={len(relations)}')

    lanelets = {}
    for rel in relations:
        if rel['tags'].get('type') != 'lanelet':
            continue
        lanes = {}
        for mtype, ref, role in rel['members']:
            if mtype == 'way':
                lanes.setdefault(role, []).append(ref)
        lanelets[rel['id']] = {'lanes': lanes, 'tags': rel['tags']}

    print('\n=== lanelets ===')
    for lid in sorted(lanelets, key=lambda x: int(x)):
        info = lanelets[lid]
        lanes = info['lanes']
        tags = info['tags']
        print(f"\nlanelet {lid} lane_id={tags.get('lane_id')!r} "
              f"one_way={tags.get('one_way')} subtype={tags.get('subtype')}")
        for role in ('left', 'right', 'centerline'):
            if role not in lanes:
                print(f'  missing {role}')
                continue
            wid = lanes[role][0]
            pts = way_pts(ways[wid], nodes)
            first, last = pts[0], pts[-1]
            print(f'  {role:10s} way={wid} npts={len(pts)} '
                  f'start=({first[0]:.3f},{first[1]:.3f}) '
                  f'end=({last[0]:.3f},{last[1]:.3f})')

    print('\n=== way endpoints (connectivity check) ===')
    # map endpoint coordinates -> ways, rounded 1e-3
    ep = {}
    for wid, way in ways.items():
        if way['tags'].get('type') != 'line_thin':
            continue
        pts = way_pts(way, nodes)
        for label, p in (('A', pts[0]), ('B', pts[-1])):
            key = (round(p[0], 3), round(p[1], 3))
            ep.setdefault(key, []).append(f'{wid}:{label}')
    for key in sorted(ep):
        if len(ep[key]) > 1:
            print(f'  shared endpoint {key}: {ep[key]}')


if __name__ == '__main__':
    main()
