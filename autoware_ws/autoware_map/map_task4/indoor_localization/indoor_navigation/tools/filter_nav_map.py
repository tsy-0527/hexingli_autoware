#!/usr/bin/env python3
"""Filter sparse point-cloud artefacts from a Nav2 PGM map.

The input map is kept as the geometric/known-space reference.  Existing
occupied cells are retained only when the source PCD contains a sufficiently
tall vertical structure in the same XY cell.  This removes floor clutter and
short scan ghosts without inventing new obstacles.  Tiny occupied components
and enclosed unknown islands are then removed conservatively.
"""

import argparse
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage
import yaml


def read_binary_xyzi_pcd(path: Path):
    data = path.read_bytes()
    marker = b"DATA binary\n"
    offset = data.find(marker)
    if offset < 0:
        raise ValueError(f"{path}: only DATA binary PCD is supported")
    header = data[:offset].decode("ascii")
    fields = next(line for line in header.splitlines() if line.startswith("FIELDS ")).split()[1:]
    if fields != ["x", "y", "z", "intensity"]:
        raise ValueError(f"{path}: expected x y z intensity, got {fields}")
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("intensity", "<f4")])
    return np.frombuffer(data, dtype=dtype, offset=offset + len(marker))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("map_yaml", type=Path)
    ap.add_argument("pointcloud", type=Path)
    ap.add_argument("output_pgm", type=Path)
    ap.add_argument("--z-min", type=float, default=0.08)
    ap.add_argument("--z-max", type=float, default=2.00)
    ap.add_argument("--min-vertical-span", type=float, default=0.60)
    ap.add_argument("--min-component-pixels", type=int, default=6)
    args = ap.parse_args()

    meta = yaml.safe_load(args.map_yaml.read_text())
    src = args.map_yaml.parent / meta["image"]
    pgm = Image.open(src)
    image = np.frombuffer(pgm.tobytes(), dtype=np.uint8).reshape(pgm.height, pgm.width).copy()
    height, width = image.shape
    resolution = float(meta["resolution"])
    origin_x, origin_y = (float(v) for v in meta["origin"][:2])

    points = read_binary_xyzi_pcd(args.pointcloud)
    ix = np.floor((points["x"] - origin_x) / resolution).astype(np.int64)
    iy = np.floor((points["y"] - origin_y) / resolution).astype(np.int64)
    z = points["z"]
    valid = (
        (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height) &
        (z > args.z_min) & (z < args.z_max)
    )
    flat = iy[valid] * width + ix[valid]
    z_lo = np.full(width * height, np.inf)
    z_hi = np.full(width * height, -np.inf)
    np.minimum.at(z_lo, flat, z[valid])
    np.maximum.at(z_hi, flat, z[valid])
    # PGM row zero is the maximum map Y, unlike the bottom-up map grid.
    vertical_span = np.flipud((z_hi - z_lo).reshape(height, width))

    output = image.copy()
    occupied = image < 100
    output[occupied & (vertical_span < args.min_vertical_span)] = 254

    # Close only one-cell scan gaps before finding indoor holes.  This fills
    # small unknown islands inside mapped rooms while the exterior remains the
    # large boundary-connected unknown region.
    unknown = (output >= 100) & (output <= 250)
    known = ~unknown
    sealed = ndimage.binary_closing(known, np.ones((3, 3), dtype=bool), iterations=1)
    indoor = ndimage.binary_fill_holes(sealed)
    output[unknown & indoor] = 254

    occupied = output < 100
    labels, _ = ndimage.label(occupied, np.ones((3, 3), dtype=np.uint8))
    sizes = np.bincount(labels.ravel())
    keep = sizes[labels] >= args.min_component_pixels
    output[occupied & ~keep] = 254

    args.output_pgm.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(output).save(args.output_pgm)
    print(
        f"{src} -> {args.output_pgm}: occupied {int((image < 100).sum())} -> "
        f"{int((output < 100).sum())}, unknown "
        f"{int(((image >= 100) & (image <= 250)).sum())} -> "
        f"{int(((output >= 100) & (output <= 250)).sum())}"
    )


if __name__ == "__main__":
    main()
