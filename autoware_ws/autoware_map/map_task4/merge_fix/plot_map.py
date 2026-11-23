#!/usr/bin/env python3
"""Render lanelets to a PNG for visual inspection of the merge area."""
import sys
import xml.etree.ElementTree as ET

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, '/home/tang/autoware_map/map_task4/merge_fix')
from analyze_lanelets import load, way_pts  # noqa: E402


COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
          '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']


def main():
    path = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else 'map.png'
    nodes, ways, relations = load(path)

    lanelets = []
    for rel in relations:
        if rel['tags'].get('type') != 'lanelet':
            continue
        lanes = {}
        for mtype, ref, role in rel['members']:
            if mtype == 'way':
                lanes.setdefault(role, []).append(ref)
        lanelets.append((rel, lanes))

    fig, ax = plt.subplots(figsize=(18, 12))
    for idx, (rel, lanes) in enumerate(sorted(
            lanelets, key=lambda x: int(x[0]['id']))):
        color = COLORS[idx % len(COLORS)]
        lid = rel['id']
        name = rel['tags'].get('lane_id', lid)
        if 'left' in lanes and 'right' in lanes:
            left = way_pts(ways[lanes['left'][0]], nodes)
            right = way_pts(ways[lanes['right'][0]], nodes)
            poly = left + right[::-1]
            xs = [p[0] for p in poly]
            ys = [p[1] for p in poly]
            ax.fill(xs, ys, alpha=0.28, color=color, label=f'{lid} {name}')
            ax.plot([p[0] for p in left], [p[1] for p in left],
                    color=color, lw=1.2)
            ax.plot([p[0] for p in right], [p[1] for p in right],
                    color=color, lw=1.2)
        if 'centerline' in lanes:
            cl = way_pts(ways[lanes['centerline'][0]], nodes)
            cx = [p[0] for p in cl]
            cy = [p[1] for p in cl]
            ax.plot(cx, cy, '--', color=color, lw=1.0)
            ax.annotate(f'{lid} {name}', (cx[len(cx) // 2], cy[len(cy) // 2]),
                        fontsize=8, color=color)
        # mark connection endpoints of boundaries
        for role in ('left', 'right'):
            if role in lanes:
                pts = way_pts(ways[lanes[role][0]], nodes)
                ax.plot(pts[0][0], pts[0][1], 'o', ms=4, color=color)
                ax.plot(pts[-1][0], pts[-1][1], 's', ms=4, color=color)

    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend(loc='best', fontsize=8)
    ax.set_xlabel('local_x [m]')
    ax.set_ylabel('local_y [m]')
    ax.set_title(path)
    fig.tight_layout()
    fig.savefig(out, dpi=140)
    print(f'saved {out}')


if __name__ == '__main__':
    main()
