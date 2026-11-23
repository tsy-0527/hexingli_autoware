#!/usr/bin/env python3
"""Before/after view of the Y merge/exit area of lanelet2_map(21)."""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, '/home/tang/autoware_map/map_task4/merge_fix')
from analyze_lanelets import load, way_pts  # noqa: E402

COLORS = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd',
          '#8c564b', '#e377c2', '#7f7f7f', '#bcbd22', '#17becf']


def draw(ax, path, title):
    nodes, ways, rels = load(path)
    for rel in rels:
        if rel['tags'].get('type') != 'lanelet':
            continue
        lanes = {}
        for mtype, ref, role in rel['members']:
            if mtype == 'way':
                lanes.setdefault(role, []).append(ref)
        name = (rel['tags'].get('lane_id')
                or f"{rel['id']}")
        color = COLORS[int(rel['id']) % len(COLORS)]
        if 'left' not in lanes or 'right' not in lanes:
            continue
        left = way_pts(ways[lanes['left'][0]], nodes)
        right = way_pts(ways[lanes['right'][0]], nodes)
        poly = left + right[::-1]
        ax.fill([p[0] for p in poly], [p[1] for p in poly],
                alpha=0.33, color=color, label=name)
        ax.plot([p[0] for p in left], [p[1] for p in left],
                color=color, lw=1.3)
        ax.plot([p[0] for p in right], [p[1] for p in right],
                color=color, lw=1.3)
    ax.set_xlim(45, 80)
    ax.set_ylim(-82, -70)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.4)
    ax.set_title(title)
    ax.legend(fontsize=8, loc='lower left')


def main():
    before = '/home/tang/下载/lanelet2_map(21).osm'
    after = '/home/tang/autoware_map/map_task4/merge_fix/lanelet2_map21_merge_fixed.osm'
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    draw(axes[0], before, 'before: Y edges only touch at one point')
    draw(axes[1], after, 'after: shared merge/exit cross-section (2-way)')
    fig.tight_layout()
    out = '/home/tang/autoware_map/map_task4/merge_fix/map21_y_before_after.png'
    fig.savefig(out, dpi=150)
    print('saved', out)


if __name__ == '__main__':
    main()
