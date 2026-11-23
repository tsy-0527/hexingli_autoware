#!/usr/bin/env python3
"""Side-by-side before/after view of the merge area."""
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, '/home/tang/autoware_map/map_task4/merge_fix')
from analyze_lanelets import load, way_pts  # noqa: E402

COLORS = {
    'start': '#1f77b4', 'start_north': '#aec7e8',
    'park': '#2ca02c', 'park_loop': '#98df8a',
    'conn': '#9467bd', 'main_w': '#8c564b',
    'east_out': '#e377c2', 'east_in': '#7f7f7f', 'uturn': '#bcbd22',
}


def draw(ax, path, title):
    nodes, ways, rels = load(path)
    for rel in rels:
        if rel['tags'].get('type') != 'lanelet':
            continue
        lanes = {}
        for mtype, ref, role in rel['members']:
            if mtype == 'way':
                lanes.setdefault(role, []).append(ref)
        name = rel['tags'].get('lane_id', rel['id'])
        color = COLORS.get(name, '#333333')
        if 'left' not in lanes or 'right' not in lanes:
            continue
        left = way_pts(ways[lanes['left'][0]], nodes)
        right = way_pts(ways[lanes['right'][0]], nodes)
        poly = left + right[::-1]
        ax.fill([p[0] for p in poly], [p[1] for p in poly],
                alpha=0.35, color=color, label=name)
        ax.plot([p[0] for p in left], [p[1] for p in left],
                color=color, lw=1.4)
        ax.plot([p[0] for p in right], [p[1] for p in right],
                color=color, lw=1.4)
    ax.set_xlim(45, 100)
    ax.set_ylim(-86, -72)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.4)
    ax.set_title(title)
    ax.legend(fontsize=8, loc='lower right')


def main():
    before = '/home/tang/下载/lanelet2_map(2).osm'
    after = '/home/tang/autoware_map/map_task4/merge_fix/lanelet2_map_merge_fixed.osm'
    fig, axes = plt.subplots(1, 2, figsize=(18, 8))
    draw(axes[0], before, 'before: isolated lanes / 1.7 m gap')
    draw(axes[1], after, 'after: ramp merges into adjacent lane (dashed)')
    fig.tight_layout()
    out = '/home/tang/autoware_map/map_task4/merge_fix/merge_area_before_after.png'
    fig.savefig(out, dpi=150)
    print('saved', out)


if __name__ == '__main__':
    main()
