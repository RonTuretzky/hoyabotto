"""Draw captions and 3D-anchored labels onto the Blender frames, then encode MP4 + GIF.

    python overlay.py FRAMES_DIR OUT_DIR
"""
import json, subprocess, sys
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

src, out = Path(sys.argv[1]), Path(sys.argv[2])
out.mkdir(parents=True, exist_ok=True)
anchors = {int(k): v for k, v in json.load(open(src / 'anchors.json')).items()}
F = '/System/Library/Fonts/Helvetica.ttc'
ORANGE, PINK, BLUE, GREEN = (235, 110, 10), (219, 26, 107), (13, 102, 217), (25, 140, 70)
CAPTIONS = [  # first, last frame, title, line
    (1, 75, 'Your robot at the table, as the fold policy learned it', 'XLeRobot model: cart, mast and head, two SO-101 arms, the table and the tagged carton'),
    (76, 175, 'ARM BASE = the bottom block of each arm', "It holds the shoulder motor and is bolted to the robot. It never moves; everything above it does."),
    (176, 290, 'PAN AXIS = the vertical line each arm swings around', 'Watch the arms swing left and right: each turns about its own pink line (the shoulder-pan motor).'),
    (291, 405, 'Measure 1: 220 mm between the two pan axes', 'Centre of one shoulder-pan motor to the centre of the other. The policy learned exactly this spacing.'),
    (406, 560, 'Measure 2: 120 mm, bottom of the arm bases down to the table top', 'Blue plane = the bottoms of both arm bases. Raise or lower the table until the gap is 120 mm.'),
    (561, 600, 'So: 220 mm apart, 120 mm above the table', 'The carton is 108 mm tall, so its rim sits just 12 mm below the arm bases.'),
]
LABELS = [  # key, text, colour, first, last, (dx, dy) offset of the box from the point
    ('base_left', 'arm base', ORANGE, 96, 290, (-150, -40)), ('base_right', 'arm base', ORANGE, 96, 290, (60, -40)),
    ('axis_left', 'pan axis', PINK, 190, 405, (-120, -30)), ('axis_right', 'pan axis', PINK, 190, 405, (40, -30)),
    ('mm220', '220 mm', PINK, 300, 600, (-55, -60)),
    ('plane', 'bottom of the arm bases', BLUE, 430, 560, (-330, -70)),
    ('table', 'table top (the carton stands on it)', BLUE, 436, 560, (-430, 40)),
    ('mm120', '120 mm', BLUE, 440, 600, (-150, -18)),
    ('carton_rim', 'carton rim: 12 mm below the bases', GREEN, 470, 560, (40, -110)),
]


def font(size, bold=False):
    try:
        return ImageFont.truetype(F, size, index=1 if bold else 0)
    except OSError:
        return ImageFont.load_default()


frames = sorted(int(p.stem[1:]) for p in src.glob('f*.png'))
for f in frames:
    im = Image.open(src / f'f{f:04d}.png').convert('RGB')
    W, H = im.size
    s = W / 1280
    d = ImageDraw.Draw(im, 'RGBA')
    for key, text, col, a, b, (dx, dy) in LABELS:
        if not a <= f <= b or f not in anchors:
            continue
        u, v = anchors[f][key]
        if not (0 <= u <= 1 and 0 <= v <= 1):
            continue
        x, y = u * W, (1 - v) * H
        ft = font(int(26 * s), True)
        tw = d.textlength(text, font=ft)
        bx, by = x + dx * s, y + dy * s
        bx = min(max(bx, 8), W - tw - 24 * s)
        d.line([(x, y), (bx + tw / 2 + 8 * s, by + 18 * s)], fill=col + (255,), width=max(2, int(3 * s)))
        d.ellipse([x - 5 * s, y - 5 * s, x + 5 * s, y + 5 * s], fill=col + (255,))
        d.rounded_rectangle([bx, by, bx + tw + 16 * s, by + 36 * s], int(8 * s), fill=(255, 255, 255, 235), outline=col + (255,), width=max(2, int(3 * s)))
        d.text((bx + 8 * s, by + 3 * s), text, font=ft, fill=col)
    for a, b, title, line in CAPTIONS:
        if a <= f <= b:
            d.rectangle([0, H - 96 * s, W, H], fill=(15, 18, 24, 215))
            d.text((24 * s, H - 88 * s), title, font=font(int(32 * s), True), fill=(255, 255, 255))
            d.text((24 * s, H - 44 * s), line, font=font(int(22 * s)), fill=(210, 215, 225))
    im.save(out / f'o{f:04d}.png')
print('overlaid', len(frames))
