"""Printable box tags for the learned fold policy, where each one goes, and pictures of it. No robot, camera or motor.

The fold policy was trained on renders of a tagged carton (carton/folding_sim.py marker(), carton/folding_markers.py,
tools/diagnose_short_flap_brace.py extra wall markers): twelve tag36h11 tags on the walls, the inside floor and the
four flaps. This writes, from the same numbers:

- fold-box-tags.pdf: the twelve tags at exact size (600 dpi raster, print at 100%), each with its ID, place and "up";
- box-tags-<face>.svg: each face to scale, with each tag's black square measured from the face's edges (edge to edge);
- box-tags-overview.png (with --scene): the training carton rendered with every tag labelled.

    python tools/make_fold_box_tags.py --out docs/img/fold-policy-setup \
        [--scene <fold-demos>/batch-220-01/trial-020/run/scene.xml]

Carton frame (carton/geometry.Box, 379 x 283 x 108 mm, flaps 140 mm): x to the robot's right, y away from the robot,
z up, origin at the bottom centre. Every tag reads upright seen from outside its face (floor: from the robot's side).
"""
from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carton.geometry import Box  # noqa: E402
from carton.servo.tag_kit import marker_grid  # noqa: E402

BOX = Box()
L, W, H, F = BOX.length * 1000, BOX.width * 1000, BOX.height * 1000, BOX.flap * 1000   # mm
WALL_TAG, FLAP_TAG = 45.0, 35.0     # black square, mm (BOX_TAG_SIZE; flap markers in folding_sim.build_scene)
FLAP_UP = 90.0                      # flap tag centre above the fold line (folding_sim: tag_point z .090)


@dataclass(frozen=True)
class Tag:
    tag_id: int
    face: str
    size: float
    h: float          # centre, mm from the face diagram's left edge
    v: float          # centre, mm from the face diagram's bottom edge
    where: str        # plain words


def edges(tag):
    """Black-square edges (mm, rounded half up): from the face diagram's left edge to the square's left edge, and
    from its bottom edge to the square's bottom edge. The square is printed exactly; the white border depends on the cut."""
    return int(tag.h - tag.size / 2 + 0.5), int(tag.v - tag.size / 2 + 0.5)


@dataclass(frozen=True)
class Face:
    key: str
    title: str
    width: float
    height: float
    left: str
    right: str
    bottom: str
    top: str
    note: str


# Diagrams are drawn as seen from OUTSIDE each face (floor: from above, standing at the robot).
FACES = [
    Face('near-wall', 'Near wall (outside, facing the robot)', L, H, "robot's LEFT end", "robot's RIGHT end",
         'bottom edge (table)', 'top edge (fold line)', '3 tags, all at mid-height'),
    Face('left-wall', "Left wall (outside, robot's left end)", W, H, 'far edge', 'near edge (robot side)',
         'bottom edge (table)', 'top edge (fold line)', '2 tags, mid-height'),
    Face('right-wall', "Right wall (outside, robot's right end)", W, H, 'near edge (robot side)', 'far edge',
         'bottom edge (table)', 'top edge (fold line)', '1 tag, mid-height'),
    Face('floor', 'Inside floor (from above, standing at the robot)', L, W, "robot's LEFT", "robot's RIGHT",
         'near wall (robot side)', 'far wall', '2 tags, both centred front to back'),
    Face('short-left-flap', "Left short flap (outside face, standing up)", W - 8, F, 'far end', 'near end (robot side)',
         'fold line', 'free edge', "the left arm folds this flap"),
    Face('short-right-flap', "Right short flap (outside face, standing up)", W - 8, F, 'near end (robot side)',
         'far end', 'fold line', 'free edge', "the right arm folds this flap"),
    Face('long-far-flap', 'Far long flap (outside face, standing up)', L - 8, F, "robot's RIGHT end", "robot's LEFT end",
         'fold line', 'free edge', 'seen from behind the box'),
    Face('long-near-flap', 'Near long flap (outside face, standing up)', L - 8, F, "robot's LEFT end",
         "robot's RIGHT end", 'fold line', 'free edge', 'faces the robot'),
]

TAGS = [
    # near wall: carton x = -120 / 0 / +120 mm, z = H/2 (BOX_MARKERS 26, 10, 27)
    Tag(26, 'near-wall', WALL_TAG, L / 2 - 120, H / 2, "near wall, 120 mm left of centre"),
    Tag(10, 'near-wall', WALL_TAG, L / 2, H / 2, 'near wall, centre'),
    Tag(27, 'near-wall', WALL_TAG, L / 2 + 120, H / 2, "near wall, 120 mm right of centre"),
    # left wall: carton y = +40 (21) and -80 (28); seen from outside, the far edge is on the left
    Tag(21, 'left-wall', WALL_TAG, W / 2 - 40, H / 2, 'left wall, 40 mm toward the far side of centre'),
    Tag(28, 'left-wall', WALL_TAG, W / 2 + 80, H / 2, 'left wall, 80 mm toward the robot from centre'),
    # right wall: carton y = +40 (22); seen from outside, the near edge is on the left
    Tag(22, 'right-wall', WALL_TAG, W / 2 + 40, H / 2, 'right wall, 40 mm toward the far side of centre'),
    # inside floor: carton (0, 0) (25) and (+80, 0) (24)
    Tag(25, 'floor', WALL_TAG, L / 2, W / 2, 'inside floor, centre'),
    Tag(24, 'floor', WALL_TAG, L / 2 + 80, W / 2, "inside floor, 80 mm right of centre"),
    # flaps: 90 mm above the fold line; short flaps 70 mm toward the far end, long flaps 80 mm off centre
    Tag(11, 'short-left-flap', FLAP_TAG, (W - 8) / 2 - 70, FLAP_UP, 'left short flap, 70 mm toward the far end'),
    Tag(12, 'short-right-flap', FLAP_TAG, (W - 8) / 2 + 70, FLAP_UP, 'right short flap, 70 mm toward the far end'),
    Tag(13, 'long-far-flap', FLAP_TAG, (L - 8) / 2 + 80, FLAP_UP, "far long flap, 80 mm toward the robot's left"),
    Tag(14, 'long-near-flap', FLAP_TAG, (L - 8) / 2 + 80, FLAP_UP, "near long flap, 80 mm toward the robot's right"),
]
SCENE_BODY = {26: 'box_tag_near_left', 10: 'box_tag', 27: 'box_tag_near_right', 21: 'box_tag_left',
              28: 'box_tag_left_near', 22: 'box_tag_right', 25: 'box_tag_floor_center', 24: 'box_tag_floor',
              11: 'short_left_tag', 12: 'short_right_tag', 13: 'long_far_tag', 14: 'long_near_tag'}


def check_against_scene(scene_xml):
    """Every tag's centre in the training scene matches this table (within 1 mm); returns the deviations."""
    import xml.etree.ElementTree as ET
    root = ET.parse(scene_xml).getroot()
    pos = {b.get('name'): np.array([float(v) for v in b.get('pos').split()]) * 1000 for b in root.iter('body')
           if b.get('name') in SCENE_BODY.values()}
    expect = {26: (-120, -W / 2, H / 2), 10: (0, -W / 2, H / 2), 27: (120, -W / 2, H / 2), 21: (-L / 2, 40, H / 2),
              28: (-L / 2, -80, H / 2), 22: (L / 2, 40, H / 2), 25: (0, 0, 3.8), 24: (80, 0, 3.8),
              11: (0, 70, FLAP_UP), 12: (0, 70, FLAP_UP), 13: (-80, 0, FLAP_UP), 14: (80, 0, FLAP_UP)}
    # Wall and flap tags sit 1.8 mm proud of the cardboard in the scene; anything above 2 mm is a real mismatch.
    out = {tag_id: float(np.max(np.abs(pos[body] - np.array(expect[tag_id])))) for tag_id, body in SCENE_BODY.items()}
    return out


# ------------------------------------------------------------------------------------------------ printable PDF
def tag_image(tag_id, size_mm, dpi):
    """Black square of size_mm plus one white cell on every side (the quiet zone), as a PIL image."""
    from PIL import Image
    grid = np.pad(np.asarray(marker_grid(tag_id)), 1, constant_values=255).astype(np.uint8)
    px = int(round(size_mm / 8 * 10 / 25.4 * dpi))
    return Image.fromarray(grid).resize((px, px), Image.NEAREST)


def write_pdf(out, dpi=600):
    from PIL import Image, ImageDraw, ImageFont
    page_w, page_h = int(210 / 25.4 * dpi), int(297 / 25.4 * dpi)
    mm = dpi / 25.4
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', int(3.4 * mm))
        small = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', int(2.6 * mm))
        big = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', int(6 * mm))
    except OSError:
        font = small = big = ImageFont.load_default()
    pages, page, draw, x, y, row_h = [], None, None, 0, 0, 0

    def new_page():
        nonlocal page, draw, x, y, row_h
        page = Image.new('L', (page_w, page_h), 255)
        draw = ImageDraw.Draw(page)
        draw.text((12 * mm, 10 * mm), 'Fold-policy box tags (tag36h11) - print at 100% / actual size', font=big, fill=0)
        draw.text((12 * mm, 19 * mm), f'Check: wall and floor tags measure {WALL_TAG:.0f} mm across the black square, '
                  f'flap tags {FLAP_TAG:.0f} mm. Cut on the dashed line. Arrow = this edge up.', font=small, fill=0)
        pages.append(page)
        x, y, row_h = 12 * mm, 28 * mm, 0

    new_page()
    for tag in TAGS:
        img = tag_image(tag.tag_id, tag.size, dpi)
        cell_w = max(img.width, 62 * mm)
        need_h = img.height + 26 * mm
        if x + cell_w > page_w - 10 * mm:
            x, y, row_h = 12 * mm, y + row_h, 0
        if y + need_h > page_h - 10 * mm:
            new_page()
        ox = x + (cell_w - img.width) / 2
        oy = y + 8 * mm
        draw.text((ox, y), f'ID {tag.tag_id}  ({tag.size:.0f} mm)', font=font, fill=0)
        page.paste(img, (int(ox), int(oy)))
        # dashed cut line 3 mm outside the quiet zone, and an "up" arrow
        pad = 3 * mm
        x0, y0, x1, y1 = ox - pad, oy - pad, ox + img.width + pad, oy + img.height + pad
        dash = 2 * mm
        for a in np.arange(x0, x1, 2 * dash):
            draw.line([(a, y0), (min(a + dash, x1), y0)], fill=0, width=3)
            draw.line([(a, y1), (min(a + dash, x1), y1)], fill=0, width=3)
        for b in np.arange(y0, y1, 2 * dash):
            draw.line([(x0, b), (x0, min(b + dash, y1))], fill=0, width=3)
            draw.line([(x1, b), (x1, min(b + dash, y1))], fill=0, width=3)
        cx = ox + img.width / 2
        draw.polygon([(cx, y0 - 4.2 * mm), (cx - 1.8 * mm, y0 - 1.0 * mm), (cx + 1.8 * mm, y0 - 1.0 * mm)], fill=0)
        draw.text((cx + 2.5 * mm, y0 - 4.6 * mm), 'up', font=small, fill=0)
        face = next(f for f in FACES if f.key == tag.face)
        left_mm, bottom_mm = edges(tag)
        draw.text((ox, oy + img.height + 4.5 * mm), tag.where.split(',')[0], font=small, fill=0)
        draw.text((ox, oy + img.height + 8.5 * mm), f'black square edges: {left_mm} mm from the {face.left}, {bottom_mm} mm from the {face.bottom.split(" (")[0]}',
                  font=small, fill=0)
        x += cell_w + 8 * mm
        row_h = max(row_h, need_h + 6 * mm)
    out.parent.mkdir(parents=True, exist_ok=True)
    pages[0].save(out, save_all=True, append_images=pages[1:], resolution=dpi)
    return len(pages)


# ------------------------------------------------------------------------------------------------ face diagrams
def face_svg(face: Face, tags: list[Tag], scale=1.6):
    m = 70     # margin px for labels
    w, h = face.width * scale, face.height * scale
    lanes = 30 + 22 * len(tags)          # one ruler per tag under the face
    W_, H_ = w + 2 * m + 40, h + 2 * m + 46 + lanes
    def X(hmm): return m + hmm * scale
    def Y(vmm): return m + h - vmm * scale
    s = [f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W_:.0f} {H_:.0f}" font-family="Helvetica,Arial,sans-serif">',
         f'<rect x="{m}" y="{m}" width="{w:.1f}" height="{h:.1f}" fill="#c99a62" stroke="#5b3b17" stroke-width="2" rx="2"/>',
         f'<text x="{m + w / 2:.1f}" y="{m - 34}" font-size="17" font-weight="700" text-anchor="middle" fill="#222">{face.title}</text>',
         f'<text x="{m + w / 2:.1f}" y="{m - 12}" font-size="12" text-anchor="middle" fill="#555">{face.top}</text>',
         f'<text x="{m + w / 2:.1f}" y="{m + h + 18}" font-size="12" text-anchor="middle" fill="#555">{face.bottom}</text>',
         f'<text x="{m - 10}" y="{m + h / 2:.1f}" font-size="12" text-anchor="middle" fill="#555" transform="rotate(-90 {m - 10} {m + h / 2:.1f})">{face.left}</text>',
         f'<text x="{m + w + 14}" y="{m + h / 2:.1f}" font-size="12" text-anchor="middle" fill="#555" transform="rotate(90 {m + w + 14} {m + h / 2:.1f})">{face.right}</text>',
         f'<text x="{m + w / 2:.1f}" y="{m + h + 40 + lanes}" font-size="12" text-anchor="middle" fill="#333">{face.width:.0f} × {face.height:.0f} mm · {face.note}</text>']
    for tag in tags:
        grid = np.asarray(marker_grid(tag.tag_id))
        cell = tag.size / 8 * scale
        x0, y0 = X(tag.h) - 5 * cell, Y(tag.v) - 5 * cell
        s.append(f'<rect x="{x0:.1f}" y="{y0:.1f}" width="{10 * cell:.1f}" height="{10 * cell:.1f}" fill="white" stroke="#888" stroke-width="0.6"/>')
        for r in range(8):
            for c in range(8):
                if grid[r, c] == 0:
                    s.append(f'<rect x="{x0 + (c + 1) * cell:.2f}" y="{y0 + (r + 1) * cell:.2f}" width="{cell + .05:.2f}" height="{cell + .05:.2f}" fill="black"/>')
        # edge to edge: blue from the bottom edge up to the black square's bottom edge; pink ruler under the face from
        # the left edge across to the black square's left edge
        left_mm, bottom_mm = edges(tag)
        cx = X(tag.h)
        sx, sb = x0 + cell, y0 + 9 * cell          # black square: left edge x, bottom edge y (screen)
        k = sorted(tags, key=lambda t: t.h).index(tag)
        lane = m + h + 34 + 22 * k
        s.append(f'<line x1="{sx + 1.5 * cell:.1f}" y1="{m + h}" x2="{sx + 1.5 * cell:.1f}" y2="{sb:.1f}" stroke="#1971c2" stroke-width="1.8"/>'
                 f'<line x1="{sx + 0.5 * cell:.1f}" y1="{sb:.1f}" x2="{sx + 2.5 * cell:.1f}" y2="{sb:.1f}" stroke="#1971c2" stroke-width="1.8"/>')
        s.append(f'<text x="{sx + 1.5 * cell + 6:.1f}" y="{(m + h + sb) / 2 + 5:.1f}" font-size="13" font-weight="700" fill="#1971c2">{bottom_mm}</text>')
        s.append(f'<line x1="{m}" y1="{lane:.1f}" x2="{sx:.1f}" y2="{lane:.1f}" stroke="#d6336c" stroke-width="1.6"/>'
                 f'<line x1="{m}" y1="{lane - 5:.1f}" x2="{m}" y2="{lane + 5:.1f}" stroke="#d6336c" stroke-width="1.6"/>'
                 f'<line x1="{sx:.1f}" y1="{sb:.1f}" x2="{sx:.1f}" y2="{lane + 5:.1f}" stroke="#d6336c" stroke-width="1.2" stroke-dasharray="2 3"/>')
        s.append(f'<text x="{sx + 6:.1f}" y="{lane + 4:.1f}" font-size="13" font-weight="700" fill="#d6336c">{left_mm} mm → ID {tag.tag_id}</text>')
        s.append(f'<text x="{cx:.1f}" y="{y0 - 6:.1f}" font-size="14" font-weight="700" text-anchor="middle" fill="#111">ID {tag.tag_id}</text>')
    s.append(f'<text x="{m}" y="{H_ - 22:.0f}" font-size="12" fill="#555"><tspan fill="#d6336c" font-weight="700">pink</tspan>: '
             f'mm from the {face.left} to the LEFT EDGE of the black square</text>')
    s.append(f'<text x="{m}" y="{H_ - 6:.0f}" font-size="12" fill="#555"><tspan fill="#1971c2" font-weight="700">blue</tspan>: '
             f'mm from the {face.bottom} to the NEAREST EDGE of the black square</text>')
    s.append('</svg>')
    return '\n'.join(s)


# ------------------------------------------------------------------------------------------------ 3D overview
def look_at(eye, target):
    f = np.asarray(target, float) - eye
    f /= np.linalg.norm(f)
    r = np.cross(f, [0, 0, 1.])
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return r, u


def render_overview(scene_xml, out, views=((-.42, -.52, .50), (.44, .58, .46), (0, -.16, .72)), size=(600, 800)):
    """Training carton (start pose, flaps up), robot hidden, every visible tag labelled with its ID."""
    import mujoco
    from PIL import Image, ImageDraw, ImageFont
    xml = Path(scene_xml).read_text()
    cams = ''.join(f'<camera name="tagview{i}" pos="{e[0]} {e[1]} {e[2]}" xyaxes="{" ".join(map(str, np.r_[look_at(np.array(e), [0, .0, .07])[0], look_at(np.array(e), [0, 0, .07])[1]]))}" fovy="42"/>'
                   for i, e in enumerate(views))
    xml = xml.replace('</worldbody>', cams + '</worldbody>', 1)
    tmp = Path(scene_xml).with_name('_tag_overview.xml')
    tmp.write_text(xml)
    try:
        model = mujoco.MjModel.from_xml_path(str(tmp))
    finally:
        tmp.unlink()
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    carton = model.body('carton').id
    for g in range(model.ngeom):             # keep only the carton (with its flaps and tags) and the table
        body = model.geom_bodyid[g]
        if model.body_rootid[body] != carton and not (body == 0 and model.geom(g).name == 'table'):
            model.geom_rgba[g, 3] = 0
    # put the carton at the origin, square, so the pictures match the diagrams
    adr = model.jnt_qposadr[model.joint('carton_free').id]
    data.qpos[adr:adr + 7] = [0, 0, .001, 1, 0, 0, 0]
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, size[0], size[1])
    try:
        font = ImageFont.truetype('/System/Library/Fonts/Helvetica.ttc', 22)
    except OSError:
        font = ImageFont.load_default()
    panels = []
    for i in range(len(views)):
        cam = model.camera(f'tagview{i}').id
        renderer.update_scene(data, camera=cam)
        img = Image.fromarray(renderer.render())
        draw = ImageDraw.Draw(img)
        pos, mat = data.cam_xpos[cam], data.cam_xmat[cam].reshape(3, 3)
        f = size[0] / 2 / np.tan(np.radians(42) / 2)
        for tag_id, body in SCENE_BODY.items():
            p = data.site_xpos[model.site(body + '_center').id]
            rel = mat.T @ (p - pos)                       # camera frame: x right, y up, -z forward
            normal = data.xmat[model.body(body).id].reshape(3, 3)[:, 2]      # the tag's face normal
            if rel[2] >= 0 or np.dot(normal, (pos - p) / np.linalg.norm(pos - p)) < .3:
                continue                                  # behind the camera, or seen edge-on / from behind
            geomid = np.array([-1], np.int32)
            direction = (p - pos) / np.linalg.norm(p - pos)
            dist = mujoco.mj_ray(model, data, pos, direction, None, 1, -1, geomid)
            if dist >= 0 and dist < np.linalg.norm(p - pos) - .004 and not model.geom(geomid[0]).name.startswith(body):
                continue                                  # hidden behind the carton
            u, v = size[1] / 2 + f * rel[0] / -rel[2], size[0] / 2 - f * rel[1] / -rel[2]
            label = str(tag_id)
            tw = draw.textlength(label, font=font)
            bx, by = u + 22, v - 34
            draw.line([(u, v), (bx, by + 14)], fill=(220, 30, 90), width=3)
            draw.rounded_rectangle([bx - 6, by, bx + tw + 6, by + 28], 6, fill=(255, 255, 255), outline=(220, 30, 90), width=3)
            draw.text((bx, by + 2), label, font=font, fill=(20, 20, 20))
        panels.append(np.asarray(img))
    renderer.close()
    Image.fromarray(np.concatenate(panels, 1)).save(out)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--scene', type=Path, help='a 220 mm training scene.xml, for the overview render and a position check')
    args = ap.parse_args(argv)
    args.out.mkdir(parents=True, exist_ok=True)
    pages = write_pdf(args.out / 'fold-box-tags.pdf')
    for face in FACES:
        (args.out / f'box-tags-{face.key}.svg').write_text(face_svg(face, [t for t in TAGS if t.face == face.key]))
    print(f'fold-box-tags.pdf ({pages} pages), {len(FACES)} face diagrams')
    if args.scene:
        dev = check_against_scene(args.scene)
        print('scene check, max mm deviation per tag:', {k: round(v, 1) for k, v in dev.items()})
        render_overview(args.scene, args.out / 'box-tags-overview.png')
        print('box-tags-overview.png')


if __name__ == '__main__':
    main()
