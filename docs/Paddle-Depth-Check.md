# OAK paddle depth snapshot

Manifest seq 29326; captured age 0.12 s when copied. RGB/depth timestamps differed by 0.46 ms. Both file hashes matched. Camera is rectified CAM_A optical, 640×360, aligned axial depth in mm; no robot-frame calibration.

Paddle RGB location is clear, including the narrow handle crossing the near table edge. Existing `camera_point` 5×5 patch quality checks gave:

| Pixel (x,y) | Interpretation | Valid samples | p90−p10 | Depth result |
|---|---|---:|---:|---|
| (286,252) | Handle neck | 25/25 | 0 mm | usable, z=775 mm |
| (315,272) | Handle transition | 25/25 | 37 mm | rejected: patch crosses edge/dispersion |
| (330,282) | Handle shaft | 25/25 | 0 mm | usable, z=832 mm |
| (344,292) | Handle shaft | 25/25 | 0 mm | usable, z=874 mm |
| (358,302) | Narrow/edge area | 5/25 | — | rejected: insufficient valid pixels |
| (372,312) | Distal handle end/overhang | 25/25 | 0 mm | usable axial depth z=1364 mm; likely below/overhanging tabletop, not a supported-grasp reference |

A visibly clear part of the paddle blade at (205,143) also passed (25/25, p90−p10=10 mm, z=588 mm). Thus depth exists on parts of the object, but the narrow handle is not uniformly sampleable; the transition and one edge-area patch fail the quality rule. The far end's valid depth should not be mistaken for a safe grasp point.

Six visually selected tabletop points, all individually accepted by `camera_point`, were used for an *unoriented* plane fit in the OAK optical frame:

| Pixel | Camera XYZ (mm) |
|---|---|
| (416,16) | (213.8, −372.8, 1066.0) |
| (488,24) | (354.5, −344.9, 1033.0) |
| (376,72) | (108.8, −214.2, 897.0) |
| (480,96) | (272.3, −159.1, 832.0) |
| (328,144) | (19.4, −71.3, 741.0) |
| (352,184) | (50.3, −11.6, 682.0) |

SVD plane center is approximately (169.8, −195.7, 875.2) mm, normal ±(−0.0146, −0.7339, −0.6791), with 2.16 mm max residual for these six samples. The image supports these as tabletop points, but no independently measured camera-up hint was supplied, so the normal is intentionally unoriented and this is not a complete registered scene plane. RGB-depth validity confidence is high (hashes/timing/patch checks); association of the depth at the narrow overhanging end with a graspable paddle surface is low. Camera-to-robot transform remains uncalibrated.
