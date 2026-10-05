"""Printable tag36h11 markers from OpenCV, decoded by the shared AprilTag detector.

All dimensions describe the outer black square, excluding the white quiet zone.
No network, camera, motor or SDK access. PDF export lives in the optional print tool.
"""
from __future__ import annotations

import hashlib
from pathlib import Path

import cv2
import numpy as np

from .common import Refused, atomic_json, finite
from .features import tags_from_bgr


def marker_grid(tag_id):
    if type(tag_id) is not int or not 0 <= tag_id < 587:
        raise Refused("Expected tag36h11 ID 0–586")
    try:
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        grid = cv2.aruco.generateImageMarker(dictionary, tag_id, 8, borderBits=1)
    except AttributeError as exc:
        raise Refused("Marker generation needs OpenCV >=4.7 with cv2.aruco; use the isolated vision environment") from exc
    return grid


def marker_image(tag_id, cell_px=40):
    # Eight cells across the black square, one white cell on every side.
    grid = np.pad(marker_grid(tag_id), 1, constant_values=255)
    gray = np.repeat(np.repeat(grid, cell_px, axis=0), cell_px, axis=1)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


def marker_svg(tag_id, size_mm):
    cell = size_mm / 8
    outer = 10 * cell
    cells = [f'<rect x="{(x+1)*cell:g}" y="{(y+1)*cell:g}" width="{cell:g}" height="{cell:g}"/>'
             for y, row in enumerate(marker_grid(tag_id)) for x, v in enumerate(row) if v == 0]
    return (f'<svg xmlns="http://www.w3.org/2000/svg" width="{outer:g}mm" height="{outer:g}mm" '
            f'viewBox="0 0 {outer:g} {outer:g}"><rect width="100%" height="100%" fill="white"/>'
            '<g fill="black" shape-rendering="crispEdges">' + ''.join(cells) + '</g></svg>')


def make_kit(out, anchor_mm=60, tool_mm=40, target_mm=40):
    sizes = [finite(n, "tag size") for n in (anchor_mm, tool_mm, target_mm)]
    if any(not 20 <= n <= 80 for n in sizes):
        raise Refused("Print sizes must be 20–80 mm across the black square")
    # Decode every generated marker using the actual controller detector before
    # publishing any files. An incorrect family/encoding cannot produce a kit.
    entries = []
    for (name, tag_id), size in zip((("anchor", 1), ("tool", 2), ("target", 3)), sizes):
        im = marker_image(tag_id)
        found = tags_from_bgr(im)
        if set(found) != {tag_id} or found[tag_id]["hamming"] != 0 or found[tag_id]["margin"] < 30:
            raise Refused(f"Generated marker {tag_id} failed shared-detector verification")
        entries.append({"name": name, "tag_id": tag_id, "black_square_mm": size,
                        "quiet_zone_mm_each_side": size / 8, "cut_square_mm": size * 1.25,
                        "grid_black_is_1": [''.join('1' if v == 0 else '0' for v in row) for row in marker_grid(tag_id)],
                        "verified_decode": found[tag_id], "png": f"{name}-{tag_id}.png", "svg": f"{name}-{tag_id}.svg"})
    folder = Path(out).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    for row in entries:
        if not cv2.imwrite(str(folder / row["png"]), marker_image(row["tag_id"])):
            raise Refused("Cannot save marker PNG")
        (folder / row["svg"]).write_text(marker_svg(row["tag_id"], row["black_square_mm"]))
        row["png_sha256"] = hashlib.sha256((folder / row["png"]).read_bytes()).hexdigest()
    cards = ''.join(f'<section><h2>{r["name"].upper()} / ID {r["tag_id"]}</h2>'
                    f'<p>{r["black_square_mm"]:g} mm black square; keep the white border.</p>'
                    f'<img src="{r["svg"]}" width="{r["cut_square_mm"]*3.77952756:g}" '
                    f'style="width:{r["cut_square_mm"]:g}mm;height:{r["cut_square_mm"]:g}mm"></section>' for r in entries)
    html = '''<!doctype html><html lang="en"><meta charset="utf-8"><title>Carton AprilTags</title>
<style>@page{size:A4;margin:12mm}body{font:12px Arial,sans-serif;color:#000;background:#fff;margin:0}
h1{font-size:24px}h2{font-size:16px}p{margin:5px 0}main{display:grid;grid-template-columns:1fr 1fr;gap:8mm}
section{break-inside:avoid}img{display:block}.ruler{width:100mm;border-top:1px solid black;margin-top:8mm}
</style><h1>Carton tracking / tag36h11</h1><p>Print at 100% / Actual Size. Disable Fit to Page.</p>
<p>Mount flat on matte backing. Cut outside the white border. Keep unused copies out of view.</p><main>'''
    (folder / "print.html").write_text(html + cards + '</main><div class="ruler"></div><p>This line must measure 100 mm.</p>'
        '<p>Anchor: fixed table. Tool: rigid gripper body. Target: paddle, outside its grip/contact area.</p>'
        '<p>Placement and printed size still need physical verification. No grasp or robot calibration is implied.</p></html>')
    result = {"schema": 1, "status": "TAG_KIT_GENERATED", "family": "tag36h11",
              "generator": f"OpenCV {cv2.__version__} DICT_APRILTAG_36h11",
              "detector": "farm.perception.tags / pupil-apriltags", "markers": entries,
              "print_html": str(folder / "print.html"), "motor_writes": 0, "physical_task_completed": False}
    atomic_json(folder / "kit.json", result)
    return result
