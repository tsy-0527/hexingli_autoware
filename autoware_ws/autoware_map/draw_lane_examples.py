from pathlib import Path
import math

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
CGI_IMAGE = ROOT / "_cgi-bin_mmwebwx-bin_webwxgetmsgimg__&MsgID=3655976797439981384&skey=@crypt_bd169c88_cf32d449db3a53877e157a4ee4584b6c&mmweb_appid=wx_webfilehelper.jpeg"
ROUTE_IMAGE = ROOT / "route.png"
PCD_FILE = ROOT / "test.pcd"


def draw_arrow(draw, start, end, color, width=5, head=16):
    draw.line([start, end], fill=color, width=width)
    angle = math.atan2(end[1] - start[1], end[0] - start[0])
    left = (
        end[0] - head * math.cos(angle - math.pi / 6),
        end[1] - head * math.sin(angle - math.pi / 6),
    )
    right = (
        end[0] - head * math.cos(angle + math.pi / 6),
        end[1] - head * math.sin(angle + math.pi / 6),
    )
    draw.polygon([end, left, right], fill=color)


def draw_lane_overlay(base, outer, inner, center, out_path, title):
    image = base.convert("RGBA")
    overlay = Image.new("RGBA", image.size, (0, 0, 0, 0))
    mask = Image.new("L", image.size, 0)
    mask_draw = ImageDraw.Draw(mask)
    mask_draw.polygon(outer, fill=95)
    mask_draw.polygon(inner, fill=0)
    lane_fill = Image.new("RGBA", image.size, (0, 155, 255, 0))
    lane_fill.putalpha(mask)
    overlay = Image.alpha_composite(overlay, lane_fill)

    draw = ImageDraw.Draw(overlay)
    draw.line(outer + [outer[0]], fill=(0, 220, 255, 230), width=4, joint="curve")
    draw.line(inner + [inner[0]], fill=(0, 220, 255, 230), width=4, joint="curve")
    draw.line(center + [center[0]], fill=(255, 214, 72, 255), width=5, joint="curve")

    for point in center:
        r = 5
        draw.ellipse((point[0] - r, point[1] - r, point[0] + r, point[1] + r), fill=(255, 214, 72, 255))

    arrow_segments = [
        (center[0], center[1]),
        (center[1], center[2]),
        (center[2], center[3]),
        (center[3], center[4]),
        (center[4], center[5]),
    ]
    for start, end in arrow_segments:
        sx = start[0] * 0.55 + end[0] * 0.45
        sy = start[1] * 0.55 + end[1] * 0.45
        ex = start[0] * 0.35 + end[0] * 0.65
        ey = start[1] * 0.35 + end[1] * 0.65
        draw_arrow(draw, (sx, sy), (ex, ey), (255, 255, 255, 240), width=4, head=15)

    try:
        font = ImageFont.truetype("DejaVuSans.ttf", 20)
        small = ImageFont.truetype("DejaVuSans.ttf", 16)
    except OSError:
        font = ImageFont.load_default()
        small = font

    legend_x, legend_y = 18, 18
    draw.rounded_rectangle((legend_x, legend_y, legend_x + 236, legend_y + 92), radius=8, fill=(0, 0, 0, 155))
    draw.text((legend_x + 12, legend_y + 10), title, fill=(255, 255, 255, 255), font=font)
    draw.line((legend_x + 14, legend_y + 48, legend_x + 58, legend_y + 48), fill=(0, 220, 255, 255), width=4)
    draw.text((legend_x + 68, legend_y + 39), "lane boundary", fill=(235, 245, 255, 255), font=small)
    draw.line((legend_x + 14, legend_y + 74, legend_x + 58, legend_y + 74), fill=(255, 214, 72, 255), width=5)
    draw.text((legend_x + 68, legend_y + 65), "centerline / direction", fill=(235, 245, 255, 255), font=small)

    result = Image.alpha_composite(image, overlay).convert("RGB")
    result.save(out_path)


def affine_from_points(src, dst):
    matrix = []
    vector = []
    for (x, y), (u, v) in zip(src, dst):
        matrix.append([x, y, 1, 0, 0, 0])
        matrix.append([0, 0, 0, x, y, 1])
        vector.extend([u, v])
    params, *_ = np.linalg.lstsq(np.array(matrix, dtype=float), np.array(vector, dtype=float), rcond=None)
    return params.reshape(2, 3)


def transform_points(points, affine):
    transformed = []
    for x, y in points:
        u = affine[0, 0] * x + affine[0, 1] * y + affine[0, 2]
        v = affine[1, 0] * x + affine[1, 1] * y + affine[1, 2]
        transformed.append((int(round(u)), int(round(v))))
    return transformed


def main():
    cgi = Image.open(CGI_IMAGE)
    cgi_outer = [(148, 96), (691, 148), (719, 177), (718, 548), (646, 609), (190, 588), (132, 516)]
    cgi_inner = [(175, 160), (657, 180), (688, 205), (679, 542), (609, 580), (169, 558), (156, 512)]
    cgi_center = [(162, 128), (673, 164), (704, 192), (700, 535), (628, 594), (181, 574), (144, 514)]
    draw_lane_overlay(cgi, cgi_outer, cgi_inner, cgi_center, ROOT / "lane_example_cgi.png", "Lane example")

    route = Image.open(ROUTE_IMAGE)
    route_outer = [(382, 48), (529, 56), (528, 233), (777, 313), (788, 573), (414, 556), (421, 478), (382, 333)]
    route_inner = [(429, 76), (500, 80), (500, 224), (529, 281), (730, 327), (737, 523), (456, 518), (454, 451), (425, 323)]
    route_center = [(405, 62), (514, 68), (514, 238), (533, 300), (754, 320), (762, 547), (435, 538), (438, 466), (403, 329)]
    draw_lane_overlay(route, route_outer, route_inner, route_center, ROOT / "lane_example_route.png", "Lane example")

    data = PCD_FILE.read_bytes()
    start = data.index(b"DATA binary") + len(b"DATA binary")
    start = data.index(b"\n", start) + 1
    points = np.frombuffer(data[start:], dtype=np.float32).reshape(-1, 8)[:, :3]
    points = points[np.isfinite(points).all(axis=1)]

    width, height, margin = 1200, 1000, 40
    x_min, y_min = points[:, 0].min(), points[:, 1].min()
    x_max, y_max = points[:, 0].max(), points[:, 1].max()
    scale = min((width - margin * 2) / (x_max - x_min), (height - margin * 2) / (y_max - y_min))
    px = ((points[:, 0] - x_min) * scale + margin).astype(np.int32)
    py = (height - ((points[:, 1] - y_min) * scale + margin)).astype(np.int32)
    valid = (px >= 0) & (px < width) & (py >= 0) & (py < height)
    px, py = px[valid], py[valid]
    z = points[:, 2][valid]
    z_low, z_high = np.percentile(z, [2, 98])
    shade = np.clip((z - z_low) / max(z_high - z_low, 1e-6), 0, 1)

    cloud = np.zeros((height, width, 3), dtype=np.uint8)
    cloud[py, px, 0] = np.maximum(cloud[py, px, 0], (180 + shade * 75).astype(np.uint8))
    cloud[py, px, 1] = np.maximum(cloud[py, px, 1], (180 + shade * 75).astype(np.uint8))
    cloud[py, px, 2] = np.maximum(cloud[py, px, 2], (180 + shade * 75).astype(np.uint8))
    cloud_image = Image.fromarray(cloud)
    cloud_image.save(ROOT / "test_pcd_projection.png")

    src_refs = [(148, 96), (691, 148), (718, 548), (132, 516), (175, 160), (657, 180), (679, 542), (169, 558)]
    dst_refs = [(286, 230), (865, 300), (829, 704), (276, 712), (319, 302), (820, 326), (781, 664), (319, 654)]
    affine = affine_from_points(src_refs, dst_refs)
    draw_lane_overlay(
        cloud_image,
        transform_points(cgi_outer, affine),
        transform_points(cgi_inner, affine),
        transform_points(cgi_center, affine),
        ROOT / "lane_example_pcd_projection.png",
        "Lane example",
    )


if __name__ == "__main__":
    main()
