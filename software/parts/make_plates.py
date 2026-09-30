"""Arrange print-ready parts on 220 x 220 mm plates and write plain 3MF files any slicer can open.

Plate 1: cress trough + four wick insets.   Plate 2: cress holder + bottle rest + light paddle + two tag tiles.
Each mesh is translated so its lowest face sits on the bed. Run: python parts/make_plates.py [out_dir]
"""
from __future__ import annotations

import struct, sys, zipfile
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
DL = Path.home() / "Downloads" / "xlerobot-farm-parts"
BED = 220.0


def read_stl(p: Path) -> np.ndarray:
    b = p.read_bytes()
    if b[:5] == b"solid" and b"facet" in b[:400]:
        import re
        v = np.array([list(map(float, m)) for m in re.findall(rb"vertex\s+(\S+)\s+(\S+)\s+(\S+)", b)], float)
        return v.reshape(-1, 3, 3)
    n = struct.unpack("<I", b[80:84])[0]
    arr = np.frombuffer(b[84:84 + 50 * n], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return arr["v"].reshape(-1, 3, 3).astype(float)


def place(tris: np.ndarray, x: float, y: float, rot90: bool = False) -> np.ndarray:
    """Put the part's min corner at (x, y) with its lowest face on z = 0; optional 90 deg rotation about z."""
    t = tris.copy()
    if rot90:
        t = np.stack([-t[..., 1], t[..., 0], t[..., 2]], axis=-1)
    mn = t.reshape(-1, 3).min(0)
    t = t - mn + np.array([x, y, 0.0])
    return t


def write_3mf(path: Path, parts: list[tuple[str, np.ndarray]]) -> None:
    objs, items = [], []
    for i, (name, tris) in enumerate(parts, 1):
        v = tris.reshape(-1, 3)
        uniq, inv = np.unique(np.round(v, 4), axis=0, return_inverse=True)
        f = inv.reshape(-1, 3)
        verts = "".join(f'<vertex x="{a:.4f}" y="{b:.4f}" z="{c:.4f}"/>' for a, b, c in uniq)
        faces = "".join(f'<triangle v1="{p}" v2="{q}" v3="{r}"/>' for p, q, r in f)
        objs.append(f'<object id="{i}" name="{name}" type="model"><mesh><vertices>{verts}</vertices><triangles>{faces}</triangles></mesh></object>')
        items.append(f'<item objectid="{i}"/>')
    model = ('<?xml version="1.0" encoding="UTF-8"?><model unit="millimeter" xml:lang="en-US" xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">'
             f'<resources>{"".join(objs)}</resources><build>{"".join(items)}</build></model>')
    with zipfile.ZipFile(path, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr("[Content_Types].xml", '<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types"><Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/><Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
        z.writestr("_rels/.rels", '<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships"><Relationship Target="/3D/3dmodel.model" Id="rel0" Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
        z.writestr("3D/3dmodel.model", model)


def check(parts):
    allv = np.concatenate([t.reshape(-1, 3) for _, t in parts])
    mn, mx = allv.min(0), allv.max(0)
    assert mn[0] >= 5 and mn[1] >= 5 and mx[0] <= BED - 5 and mx[1] <= BED - 5, (mn, mx)
    return mn, mx


def main(out: Path):
    out.mkdir(parents=True, exist_ok=True)
    trough, holder, inset = (read_stl(DL / f"cressmaster-{n}.stl") for n in ("trough", "holder", "inset"))
    rest, paddle, tag1, tag2 = (read_stl(HERE / "out" / f) for f in ("bottle_rest.stl", "paddle_bh1750.stl", "tag_36h11_id1.stl", "tag_36h11_id2.stl"))
    # Plate 1: trough (180 x 85) across the front, four insets (9 x 69) lying along x behind it, 6 mm apart
    p1 = [("cress_trough", place(trough, 20, 12))]
    for k in range(4):
        p1.append((f"cress_inset_{k + 1}", place(inset, 20, 112 + k * 16, rot90=True)))   # rotated: 69 x 9 footprint, stacked in y
    mn, mx = check(p1)
    write_3mf(out / "plate1_cress_trough_insets.3mf", p1)
    print("plate1: trough + 4 insets   footprint", np.round(mn, 1), "→", np.round(mx, 1))
    # Plate 2: holder (178 x 83) front, bottle rest (70 x 70), tag tiles and paddle behind
    p2 = [("cress_holder", place(holder, 20, 10)), ("bottle_rest", place(rest, 20, 105)), ("tag_id1", place(tag1, 100, 105)), ("tag_id2", place(tag2, 150, 105)),
          ("light_paddle", place(paddle, 20, 185))]
    mn, mx = check(p2)
    write_3mf(out / "plate2_holder_rest_paddle_tags.3mf", p2)
    print("plate2: holder + rest + paddle + tags   footprint", np.round(mn, 1), "→", np.round(mx, 1))
    for f in out.glob("plate*.3mf"):
        with zipfile.ZipFile(f) as z:
            assert "3D/3dmodel.model" in z.namelist()
        print("wrote", f, f.stat().st_size // 1024, "KB")


if __name__ == "__main__":
    main(Path(sys.argv[1]) if len(sys.argv) > 1 else DL)
