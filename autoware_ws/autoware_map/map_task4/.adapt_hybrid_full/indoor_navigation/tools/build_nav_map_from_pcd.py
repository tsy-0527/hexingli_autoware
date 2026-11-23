#!/usr/bin/env python3
"""Build a conservative Nav2 occupancy map from a binary PCD.

The obstacle test uses the vertical span in each XY cell, so it remains useful
when the mapped site is sloped and no single global floor-Z threshold exists.
"""

import argparse
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage


def read_binary_pcd(path: Path):
    with path.open("rb") as stream:
        header_lines = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"{path}: missing DATA line")
            header_lines.append(line.decode("ascii").strip())
            if line.startswith(b"DATA "):
                offset = stream.tell()
                break
    header = {line.split(maxsplit=1)[0]: line.split()[1:]
              for line in header_lines if line and not line.startswith("#")}
    if header["DATA"] != ["binary"]:
        raise ValueError(f"{path}: convert the PCD to DATA binary first")
    fields = header["FIELDS"]
    sizes = [int(v) for v in header["SIZE"]]
    types = header["TYPE"]
    counts = [int(v) for v in header.get("COUNT", ["1"] * len(fields))]
    if any(count != 1 for count in counts):
        raise ValueError(f"{path}: only COUNT 1 fields are supported")
    kinds = {("F", 4): "<f4", ("F", 8): "<f8",
             ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4",
             ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4"}
    dtype = np.dtype([(name, kinds[(kind, size)])
                      for name, kind, size in zip(fields, types, sizes)])
    point_count = int(header["POINTS"][0])
    points = np.memmap(path, dtype=dtype, mode="r", offset=offset,
                       shape=(point_count,))
    return points["x"], points["y"], points["z"]


def write_yaml(path: Path, image: str, resolution: float, ox: float, oy: float,
               mode: str = "trinary"):
    path.write_text(
        f"image: {image}\nmode: {mode}\nresolution: {resolution}\n"
        f"origin: [{ox:.3f}, {oy:.3f}, 0.0]\nnegate: 0\n"
        f"occupied_thresh: {'1.0' if mode == 'scale' else '0.65'}\n"
        f"free_thresh: {'0.0' if mode == 'scale' else '0.10'}\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pointcloud", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--padding", type=float, default=2.0)
    parser.add_argument("--min-span", type=float, default=0.60)
    parser.add_argument("--min-points", type=int, default=3)
    args = parser.parse_args()

    x, y, z = read_binary_pcd(args.pointcloud)
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[finite], y[finite], z[finite]
    res = args.resolution
    ox = np.floor((float(x.min()) - args.padding) / res) * res
    oy = np.floor((float(y.min()) - args.padding) / res) * res
    max_x = np.ceil((float(x.max()) + args.padding) / res) * res
    max_y = np.ceil((float(y.max()) + args.padding) / res) * res
    width = int(round((max_x - ox) / res))
    height = int(round((max_y - oy) / res))

    ix = np.floor((x - ox) / res).astype(np.int32)
    iy = np.floor((y - oy) / res).astype(np.int32)
    valid = (ix >= 0) & (ix < width) & (iy >= 0) & (iy < height)
    flat = iy[valid].astype(np.int64) * width + ix[valid]
    low = np.full(height * width, np.inf, dtype=np.float32)
    high = np.full(height * width, -np.inf, dtype=np.float32)
    count = np.zeros(height * width, dtype=np.uint16)
    np.minimum.at(low, flat, z[valid])
    np.maximum.at(high, flat, z[valid])
    np.add.at(count, flat, 1)

    span = (high - low).reshape(height, width)
    count = count.reshape(height, width)
    observed = count > 0
    # Bridge only small sampling gaps. Large unobserved regions remain unknown.
    known = ndimage.binary_closing(observed, np.ones((3, 3), dtype=bool))
    known |= observed
    occupied = (span >= args.min_span) & (count >= args.min_points)
    labels, _ = ndimage.label(occupied, np.ones((3, 3), dtype=np.uint8))
    sizes = np.bincount(labels.ravel())
    occupied &= sizes[labels] >= 3

    pixels = np.full((height, width), 205, dtype=np.uint8)
    pixels[known] = 254
    pixels[occupied] = 0
    pixels = np.flipud(pixels)

    args.output_dir.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pixels).save(args.output_dir / "nav_map.pgm")
    # The coarse-localization layer uses the same geometry and coordinate frame.
    Image.fromarray(pixels).save(args.output_dir / "localization_map.pgm")
    speed = np.full_like(pixels, 255)
    Image.fromarray(speed).save(args.output_dir / "speed_mask.pgm")
    write_yaml(args.output_dir / "nav_map.yaml", "nav_map.pgm", res, ox, oy)
    write_yaml(args.output_dir / "localization_map.yaml", "localization_map.pgm", res, ox, oy)
    write_yaml(args.output_dir / "speed_mask.yaml", "speed_mask.pgm", res, ox, oy, "scale")
    print(f"size={width}x{height} resolution={res} origin=({ox:.3f}, {oy:.3f})")
    print(f"known={int(known.sum())} occupied={int(occupied.sum())} "
          f"unknown={int((~known).sum())}")


if __name__ == "__main__":
    main()
