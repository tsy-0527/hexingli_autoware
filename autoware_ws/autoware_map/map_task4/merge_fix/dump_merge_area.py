#!/usr/bin/env python3
"""Print the geometry of specific ways/lanelets around the merge area."""
import sys
sys.path.insert(0, '/home/tang/autoware_map/map_task4/merge_fix')
from analyze_lanelets import load, way_pts


def main():
    path = sys.argv[1]
    nodes, ways, relations = load(path)

    def show(wid, xmin=-10, xmax=200):
        pts = way_pts(ways[wid], nodes)
        print(f'\nway {wid} tags={ways[wid]["tags"]} refs={ways[wid]["refs"]}')
        for i, (x, y) in enumerate(pts):
            if xmin <= x <= xmax:
                print(f'  [{i:02d}] ({x:9.4f}, {y:9.4f})')

    # conn, main_w, east_in boundaries + centerlines
    for wid in ('1006', '1007', '1008', '1009', '1010', '1011',
                '1015', '1016', '1017', '1012', '1013', '1014'):
        show(wid)


if __name__ == '__main__':
    main()
