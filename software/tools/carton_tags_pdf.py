"""Optional print export: python tools/carton_tags_pdf.py KIT_JSON OUTPUT_PDF.

Requires reportlab only; no robot/vision dependencies. The kit is generated and
decoded first with `python -m carton.servo tag-kit`. Draws exact vector cells.
"""
import argparse
import json
from pathlib import Path

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.pdfgen import canvas


def render(kit_path, out):
    kit = json.loads(Path(kit_path).read_text())
    if kit.get("status") != "TAG_KIT_GENERATED" or kit.get("family") != "tag36h11":
        raise ValueError("Expected a verified tag36h11 kit")
    rows = kit["markers"]
    if len(rows) != 3 or [r["tag_id"] for r in rows] != [1, 2, 3]:
        raise ValueError("Expected the three role markers")
    for row in rows:
        if len(row["grid_black_is_1"]) != 8 or any(len(s) != 8 or set(s)-{"0", "1"} for s in row["grid_black_is_1"]):
            raise ValueError("Invalid marker grid")
        if not 20 <= row["black_square_mm"] <= 80:
            raise ValueError("Invalid marker print size")
    if any(r["black_square_mm"] > limit for r, limit in zip(rows, (60, 40, 40))):
        raise ValueError("This A4 layout supports the default sizes or smaller; print larger tags from their individual SVGs")
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        raise FileExistsError("Existing PDF preserved; choose a new filename")
    c = canvas.Canvas(str(out), pagesize=A4, pageCompression=1)
    c.setTitle("Carton AprilTag mounting kit - tag36h11")
    c.setAuthor("XLeRobot farm")
    height = A4[1]
    def text(x, y, value, size=10, bold=False):
        c.setFont("Helvetica-Bold" if bold else "Helvetica", size)
        c.drawString(x*mm, height-y*mm, value)
    text(15, 19, "Carton tracking", 22, True)
    text(15, 28, "AprilTag / tag36h11 / camera-only setup", 11)
    text(15, 39, "Print at 100% / Actual Size. Turn off Fit to Page.", 11, True)
    text(15, 46, "Measure the 100 mm line before cutting. Keep the white margin around each tag.")
    placements = [(15, 66, "FIXED TABLE"), (112, 66, "GRIPPER BODY"), (112, 178, "PADDLE")]
    for row, (x, y, label) in zip(rows, placements):
        text(x, y-8, f'{row["name"].upper()} / ID {row["tag_id"]}', 12, True)
        text(x, y-2, f'{row["black_square_mm"]:g} mm black square', 9)
        cell = row["black_square_mm"]/8
        # Extra quiet zone is blank paper. Dashed cut marks stay outside it.
        outer = row["cut_square_mm"]
        c.setDash(2, 3); c.setLineWidth(.35)
        c.rect(x*mm, height-(y+outer)*mm, outer*mm, outer*mm, stroke=1, fill=0)
        c.setDash()
        for gy, grid_row in enumerate(row["grid_black_is_1"]):
            for gx, bit in enumerate(grid_row):
                if bit == "1":
                    c.rect((x+(gx+1)*cell)*mm, height-(y+(gy+2)*cell)*mm, cell*mm, cell*mm, stroke=0, fill=1)
        text(x, y+outer+6, label, 9, True)
    text(15, 169, "Mounting", 12, True)
    for i, line in enumerate([
        "1  Anchor: fixed table, in head view.",
        "2  Tool: rigid gripper body.",
        "3  Target: paddle, clear of the grip.",
        "Use flat, matte backing.",
        "Keep patterns clear of tape/glare.",
        "Keep unused copies out of view.",
        "Do not span a joint or fold.",
    ]):
        text(15, 178+i*6, line, 9)
    c.setLineWidth(.7)
    c.line(15*mm, height-255*mm, 115*mm, height-255*mm)
    for x in range(15, 116, 10):
        c.line(x*mm, height-253*mm, x*mm, height-257*mm)
    text(15, 264, "This line must measure exactly 100 mm.", 10, True)
    text(15, 275, "Print/mount sizes are starting points. Confirm visibility in both actual camera views.", 9)
    text(15, 281, "Tag centers are tracking points, not calibrated grasp points. No robot motion is authorized by this sheet.", 8)
    c.showPage(); c.save()
    return str(out.resolve())


if __name__ == "__main__":
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("kit"); p.add_argument("out")
    a = p.parse_args()
    print(render(a.kit, a.out))
