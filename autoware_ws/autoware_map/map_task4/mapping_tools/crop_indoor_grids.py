#!/usr/bin/env python3
"""Crop existing Nav2 maps to the configured transition + indoor bounds."""
import argparse
import os
import shutil
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import common as C


MAP_NAMES = ('nav_map', 'localization_map', 'speed_mask')


def crop_one(src_dir, dst_dir, name, bounds):
    yaml_path = os.path.join(src_dir, name + '.yaml')
    with open(yaml_path, encoding='utf-8') as f:
        meta = yaml.safe_load(f)
    image_path = os.path.join(src_dir, meta['image'])
    img = C.read_pgm(image_path)
    res = float(meta['resolution'])
    ox, oy = map(float, meta['origin'][:2])
    xmin, ymin, xmax, ymax = map(float, bounds)
    h, w = img.shape

    ix0 = max(0, int(np.floor((xmin - ox) / res)))
    iy0 = max(0, int(np.floor((ymin - oy) / res)))
    ix1 = min(w, int(np.ceil((xmax - ox) / res)))
    iy1 = min(h, int(np.ceil((ymax - oy) / res)))
    if ix1 <= ix0 or iy1 <= iy0:
        raise ValueError(f'{name}: crop bounds do not intersect source map')

    bottom_origin = img[::-1]
    cropped = bottom_origin[iy0:iy1, ix0:ix1][::-1]
    image_name = name + '.pgm'
    C.write_pgm(os.path.join(dst_dir, image_name), cropped)
    meta['image'] = image_name
    meta['origin'] = [round(ox + ix0 * res, 6), round(oy + iy0 * res, 6),
                      float(meta['origin'][2]) if len(meta['origin']) > 2 else 0.0]
    with open(os.path.join(dst_dir, name + '.yaml'), 'w', encoding='utf-8') as f:
        yaml.safe_dump(meta, f, sort_keys=False)
    return {'name': name, 'size_px': [int(cropped.shape[1]), int(cropped.shape[0])],
            'origin': meta['origin'], 'resolution': res}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source_dir')
    ap.add_argument('output_dir')
    ap.add_argument('--config', required=True)
    args = ap.parse_args()
    with open(args.config, encoding='utf-8') as f:
        bounds = yaml.safe_load(f)['grid_crop']['bounds']
    os.makedirs(args.output_dir, exist_ok=True)
    stats = [crop_one(args.source_dir, args.output_dir, n, bounds) for n in MAP_NAMES]
    for optional in ('map_projector_info.yaml', 'map_config.yaml', 'lanelet2_map.osm'):
        src = os.path.join(args.source_dir, optional)
        if os.path.isfile(src):
            shutil.copy2(src, os.path.join(args.output_dir, optional))
    for stat in stats:
        print(stat)


if __name__ == '__main__':
    main()

