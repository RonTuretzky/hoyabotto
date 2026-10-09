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
CAPTIONS = [
    # step 3
    (1, 39, 'Step 3: square the cart to the table', 'Start from the empty table in front of the robot.'),
    (40, 119, '111 mm from each pan axis straight out to the table edge', 'Pink: the pan axes (step 1). Orange: straight above the table edge. Both sides must read 111 mm.'),
    (120, 200, 'Left and right differ? The cart is turned', 'Rotate the cart until both sides read 111 mm again.'),
    (201, 228, 'Both 111 mm: the cart is square to the table', 'That puts the arm bases 150 mm behind the table edge.'),
    (229, 300, "Tape the carton's spot", "Near wall 10 mm from the table edge, centre 10 mm to the robot's left of the middle. Tape around the footprint."),
    # step 4
    (301, 379, 'Step 4: an empty carton, 379 x 283 x 108 mm', 'Flaps 140 mm. Tag it first (steps 4b-4g); the tags are left off here so the steps are easy to see.'),
    (380, 450, 'Stand all four flaps straight up', 'Open, not folded: each flap upright above its wall.'),
    (451, 570, 'Turn it so the SHORT flaps face the arms', 'Left short flap toward the left arm, right short flap toward the right arm. The long flaps are near and far.'),
    (571, 630, 'Slide it into its taped spot', "The blue painter's tape from step 3."),
    (631, 690, 'Near wall: 10 mm back from the table edge', 'Measure from the carton wall that faces the robot to the edge of the table.'),
    (691, 720, 'Ready: flaps up, short flaps to the arms, in its spot', 'Square to the table edge within 4 degrees.'),
    # tags
    (721, 775, 'Steps 4b-4g: the 12 box tags. Near wall: 26, 10, 27', 'Shown on the carton in its spot with the robot hidden. In practice, tag the carton before you put it down.'),
    (776, 805, 'Left end wall: 21 and 28', '28 is the one nearer the robot.'),
    (806, 826, 'Right end wall: 22', 'Toward the far side of the middle.'),
    (827, 855, 'Inside floor: 25 at the centre, 24 to its right', 'Both arrows point away from the robot.'),
    (856, 877, 'Flaps: 11 left, 12 right, 13 far, 14 near', 'Outside face of each flap; black square 73 mm above the fold line. Seen from the robot side.'),
    (878, 900, 'Camera now BEHIND the box, looking back at the robot', "So the robot's LEFT is on the RIGHT of this picture: 13 sits toward the robot's left, 14 toward its right."),
    # step 5
    (901, 934, 'Step 5: point the head camera', 'Head pan 0 (straight ahead). Start from level...'),
    (935, 990, '...and tilt the head 58 degrees down', 'The orange cone is what the head camera (OAK-D Lite) sees.'),
    (991, 1020, 'It must see all four flaps and both claws', 'Use the 4:3 full-sensor stream. The wrist cameras look along their jaws.'),
    # step 6a
    (1021, 1034, 'Step 6a: floor tags for the camera pose', 'Before the carton goes down, lay the four 80 mm tags flat.'),
    (1035, 1140, 'Tags 40-43, printed side up, top edge away from the robot', 'Distances from the base line between the arms: 40/41 at 25 cm out, 42/43 at 40 cm; 12 cm left or right.'),
    # step 8b
    (1141, 1235, 'Step 8b: pose the arms by hand at the training start', 'Torque off, arms supported: shoulders folded low, wrists bent up, jaws closed, right wrist rolled about 86 degrees.'),
    (1236, 1260, 'Every run starts from this pose', "Within about 30 ticks of the slide's table, read with robot_get_state."),
]
TAG_IDS = {'box_tag_near_left': (26, 745, 775), 'box_tag': (10, 752, 775), 'box_tag_near_right': (27, 760, 775),
           'box_tag_left': (21, 790, 805), 'box_tag_left_near': (28, 797, 805), 'box_tag_right': (22, 818, 826),
           'box_tag_floor_center': (25, 842, 855), 'box_tag_floor': (24, 849, 855), 'short_left_tag': (11, 866, 900),
           'short_right_tag': (12, 872, 900), 'long_far_tag': (13, 878, 900), 'long_near_tag': (14, 884, 900)}
LABELS = [
    ('axl', 'pan axis', PINK, 45, 228, (-150, -30)), ('axr', 'pan axis', PINK, 45, 228, (40, -30)),
    ('setl', '111 mm', PINK, 60, 135, (-120, -50)), ('setr', '111 mm', PINK, 60, 135, (30, -50)),
    ('setl', 'not equal', PINK, 145, 185, (-140, -50)), ('setr', 'not equal', PINK, 145, 185, (30, -50)),
    ('setl', '111 mm', PINK, 205, 228, (-120, -50)), ('setr', '111 mm', PINK, 205, 228, (30, -50)),
    ('edge', 'table edge', ORANGE, 55, 228, (20, 30)),
    ('ghost', 'carton footprint', GREEN, 236, 300, (40, -60)), ('tape', 'tape', BLUE, 274, 300, (30, -40)),
    ('mid', 'middle between the arms', PINK, 242, 300, (30, -50)), ('cen', 'carton centre: 10 mm left', GREEN, 248, 300, (-340, 30)),
    ('len', '379 mm', GREEN, 320, 379, (-40, -60)), ('wid', '283 mm', GREEN, 326, 379, (30, -50)), ('hgt', '108 mm tall', GREEN, 332, 379, (30, 10)),
    ('sl', 'left short flap', GREEN, 500, 570, (-210, -50)), ('sr', 'right short flap', GREEN, 500, 570, (40, -50)),
    ('ln', 'near long flap', BLUE, 515, 570, (-80, 40)), ('lf', 'far long flap', BLUE, 515, 570, (-60, -60)),
    ('larm', 'left arm', ORANGE, 500, 570, (-140, -30)), ('rarm', 'right arm', ORANGE, 500, 570, (40, -30)),
    ('tape', 'tape', BLUE, 590, 630, (30, -40)), ('gap', '10 mm: near wall to table edge', ORANGE, 640, 690, (-120, 60)),
    *[('tag_' + n, f'ID {i}', PINK, a, b, (20, -60)) for n, (i, a, b) in TAG_IDS.items()],
    ('farL', "robot's LEFT end", ORANGE, 880, 900, (-60, 20)), ('farR', "robot's RIGHT end", ORANGE, 880, 900, (-60, 20)),
    ('head', 'head camera (OAK-D Lite)', ORANGE, 906, 1020, (30, -60)),
    ('conefloor', 'what it sees', ORANGE, 965, 1020, (40, 40)),
    ('t40', '40: 12 cm left, 25 cm out', PINK, 1045, 1140, (-300, 40)), ('t41', '41: 12 cm right, 25 cm out', PINK, 1053, 1140, (40, 40)),
    ('t42', '42: 12 cm left, 40 cm out', PINK, 1061, 1140, (-300, -60)), ('t43', '43: 12 cm right, 40 cm out', PINK, 1069, 1140, (40, -60)),
    ('baseline', 'base line (between the arm bases)', BLUE, 1030, 1140, (40, -10)),
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
