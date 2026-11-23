#!/usr/bin/env python3
"""Apply statistical outlier removal only inside configured indoor polygons.

Every point outside the polygons is copied unchanged and in original order.
Neighbour context includes a margin around the polygons so valid wall points
at an indoor/outdoor boundary are not penalised by an artificial hard edge.
"""
import argparse
import json
import os
import sys

import numpy as np
import yaml
from scipy.spatial import cKDTree

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C


def points_in_polygon(xy, polygon):
    poly = np.asarray(polygon, dtype=float)
    inside = np.zeros(len(xy), dtype=bool)
    x0, y0 = poly[-1]
    for x1, y1 in poly:
        cross = (y1 > xy[:, 1]) != (y0 > xy[:, 1])
        with np.errstate(divide='ignore', invalid='ignore'):
            x_at_y = (x0 - x1) * (xy[:, 1] - y1) / (y0 - y1) + x1
        inside ^= cross & (xy[:, 0] < x_at_y)
        x0, y0 = x1, y1
    return inside


def union_mask(xy, polygons):
    mask = np.zeros(len(xy), dtype=bool)
    for polygon in polygons:
        mask |= points_in_polygon(xy, polygon)
    return mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('input_pcd')
    ap.add_argument('output_pcd')
    ap.add_argument('--config', required=True)
    ap.add_argument('--report', default=None)
    args = ap.parse_args()

    with open(args.config, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)['indoor_cleaning']
    polygons = cfg['polygons']
    mean_k = int(cfg.get('mean_k', 30))
    std_mul = float(cfg.get('std_dev_mul', 1.0))
    margin = float(cfg.get('context_margin', 0.6))

    xyz, fields, header = C.read_pcd(args.input_pcd)
    indoor = union_mask(xyz[:, :2], polygons)
    if indoor.sum() <= mean_k:
        raise RuntimeError('indoor polygon contains too few points')

    all_poly = np.concatenate([np.asarray(p, float) for p in polygons], axis=0)
    lo = all_poly.min(axis=0) - margin
    hi = all_poly.max(axis=0) + margin
    context = ((xyz[:, 0] >= lo[0]) & (xyz[:, 0] <= hi[0]) &
               (xyz[:, 1] >= lo[1]) & (xyz[:, 1] <= hi[1]))
    context_ids = np.flatnonzero(context)
    indoor_ids = np.flatnonzero(indoor)

    tree = cKDTree(xyz[context_ids])
    distances, _ = tree.query(xyz[indoor_ids], k=mean_k + 1, workers=-1)
    mean_distance = distances[:, 1:].mean(axis=1)
    threshold = float(mean_distance.mean() + std_mul * mean_distance.std())
    removed_ids = indoor_ids[mean_distance > threshold]
    keep = np.ones(len(xyz), dtype=bool)
    keep[removed_ids] = False

    os.makedirs(os.path.dirname(os.path.abspath(args.output_pcd)), exist_ok=True)
    C.write_pcd(args.output_pcd, xyz[keep], {k: v[keep] for k, v in fields.items()})

    outside = ~indoor
    report = {
        'source': os.path.abspath(args.input_pcd),
        'output': os.path.abspath(args.output_pcd),
        'input_points': int(len(xyz)),
        'indoor_points': int(indoor.sum()),
        'outdoor_points_untouched': int(outside.sum()),
        'indoor_removed': int(len(removed_ids)),
        'indoor_removed_ratio': float(len(removed_ids) / indoor.sum()),
        'output_points': int(keep.sum()),
        'mean_k': mean_k,
        'std_dev_mul': std_mul,
        'mean_distance_threshold_m': threshold,
        'outside_order_and_values_preserved': True,
        'input_fields': header.get('FIELDS', []),
    }
    report_path = args.report or os.path.splitext(args.output_pcd)[0] + '_report.json'
    with open(report_path, 'w', encoding='utf-8') as f:
        json.dump(report, f, ensure_ascii=False, indent=2)
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()

