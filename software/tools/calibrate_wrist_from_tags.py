"""Calibrate a wrist camera's lens from saved frames of the tagged fold-policy carton. Offline: it opens no camera and
talks to no robot. Replaces the printed checkerboard (tools/calibrate_camera_checkerboard.py stays as the fallback) and
writes the same intrinsics JSON, plus 'method': 'box_tags'.

    python tools/calibrate_wrist_from_tags.py --images <folder of wrist PNG/JPEG frames> --out right_wrist-640x480.json

Targets: the twelve tag36h11 box tags (carton/box_tag_geometry.py; 45 mm walls/floor, 35 mm flaps). Every frame must
show at least two of them decoded cleanly (hamming 0, decision margin >= 30); at least 8 such frames are needed and the
result is refused above 1.0 px RMS reprojection error.

Method (and why). The tags sit on four walls, the floor and four flaps, so one frame's tag corners are NOT coplanar:
that is a strong constraint on the focal length (a planar board needs tilted views for the same thing), but it needs
the tags' 3D layout. The printed layout is only nominal on a real carton: tags are placed by hand, the walls bulge, the
flaps lean a few degrees and their tags are 2 mm off the scene's surfaces. So:

1. cv2.calibrateCamera on the nominal 3D layout (CALIB_USE_INTRINSIC_GUESS from the simulation's 90 deg vertical
   FOV, square pixels not enforced, k3 fixed at 0): a first lens and one carton pose per frame.
2. Bundle adjustment (scipy least_squares, sparse Jacobian): lens (fx, fy, cx, cy, k1, k2, p1, p2), every frame's
   carton pose and every tag's own pose on the carton (6 DOF, weak prior: 5 mm / 3 deg walls and floor, 15 mm /
   15 deg flaps). Each tag's printed square stays exact (its size is the metric scale); where it sits is estimated
   from all frames together, because the carton does not move between frames. A soft-L1 pass finds bad corners;
   tag sightings above 3 px are dropped and the final pass is plain least squares, whose RMS is reported.

Per-face planar calibration was the alternative: it needs no 3D layout but discards the flap and right-wall tags
(alone on their faces) and the near-wall tags span only 45 mm vertically, so it constrains the lens much less.

Pixel convention: OpenCV's (pixel centres at integer coordinates, as cornerSubPix and the checkerboard tool). The
AprilTag detector reports corners 0.5 px further along both axes; that is removed here.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from carton import box_tag_geometry as G  # noqa: E402

IMAGE_SUFFIXES = {'.png', '.jpg', '.jpeg'}
MIN_VIEWS = 8
MAX_RMS_PX = 1.0
MIN_MARGIN = 30.0          # carton/servo/features.py: decodes below 30 are not trusted for measurement
DETECTOR_OFFSET_PX = 0.5   # pupil-apriltags corner coordinates minus OpenCV's (measured on tag_kit.marker_image)
OUTLIER_PX = 3.0
PRIOR = {True: (.005, math.radians(3)), False: (.015, math.radians(15))}   # rigid tag / flap tag: (m, rad)


class CalibrationRefused(RuntimeError):
    def __init__(self, message, report=None):
        super().__init__(message)
        self.report = report or {}


@dataclass
class View:
    name: str
    size: tuple                                  # (width, height)
    tags: dict = field(default_factory=dict)     # id -> 4x2 corners, OpenCV pixel convention, detector order
    rejected: dict = field(default_factory=dict)  # id -> reason


# ------------------------------------------------------------------------------------------------ detection
def detect_view(name, bgr, *, ids=None, min_margin=MIN_MARGIN) -> View:
    from carton.servo.features import tags_from_bgr
    h, w = bgr.shape[:2]
    view = View(name, (w, h))
    wanted = set(G.TAGS if ids is None else ids)
    for tag_id, found in tags_from_bgr(bgr).items():
        if tag_id not in wanted:
            continue
        corners = np.asarray(found['corners'], float) - DETECTOR_OFFSET_PX
        if found['hamming'] != 0:
            view.rejected[tag_id] = f"hamming {found['hamming']}"
        elif found['margin'] < min_margin:
            view.rejected[tag_id] = f"margin {found['margin']:.0f} < {min_margin:.0f}"
        elif corners.shape != (4, 2) or not np.isfinite(corners).all():
            view.rejected[tag_id] = 'bad corners'
        elif min(np.linalg.norm(corners - np.roll(corners, 1, axis=0), axis=1)) < 10:
            view.rejected[tag_id] = 'smaller than 10 px'
        elif (corners < 2).any() or (corners[:, 0] > w - 3).any() or (corners[:, 1] > h - 3).any():
            view.rejected[tag_id] = 'touches the image border'
        else:
            view.tags[tag_id] = corners
    return view


def image_paths(folder) -> list[Path]:
    folder = Path(folder)
    if not folder.is_dir():
        raise CalibrationRefused(f'{folder}: not a folder of wrist frames')
    return sorted(p for p in folder.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES)


def detect_folder(folder, *, ids=None, min_margin=MIN_MARGIN) -> list[View]:
    views, size = [], None
    for path in image_paths(folder):
        bgr = cv2.imread(str(path), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        if size is not None and bgr.shape[1::-1] != size:
            raise CalibrationRefused(f'{path.name}: all frames must have the same resolution ({size[0]}x{size[1]})')
        size = bgr.shape[1::-1]
        views.append(detect_view(path.name, bgr, ids=ids, min_margin=min_margin))
    return views


# ------------------------------------------------------------------------------------------------ model
def _rotvec_to_matrix(v):
    """Rodrigues for an array of rotation vectors (..., 3) -> (..., 3, 3)."""
    v = np.asarray(v, float)
    theta = np.linalg.norm(v, axis=-1)[..., None, None]
    small = theta < 1e-12
    k = v / np.where(small[..., 0], 1.0, theta[..., 0])
    kx = np.zeros(v.shape[:-1] + (3, 3))
    kx[..., 0, 1], kx[..., 0, 2], kx[..., 1, 2] = -k[..., 2], k[..., 1], -k[..., 0]
    kx[..., 1, 0], kx[..., 2, 0], kx[..., 2, 1] = k[..., 2], -k[..., 1], k[..., 0]
    eye = np.broadcast_to(np.eye(3), kx.shape)
    return np.where(small, eye, eye + np.sin(theta) * kx + (1 - np.cos(theta)) * kx @ kx)


def distort_project(cam_points, intr):
    """OpenCV pinhole + (k1, k2, p1, p2, k3) projection of camera-frame points (N, 3)."""
    fx, fy, cx, cy, k1, k2, p1, p2, k3 = intr
    z = cam_points[:, 2]
    x, y = cam_points[:, 0] / z, cam_points[:, 1] / z
    r2 = x * x + y * y
    radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 ** 3
    xd = x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x)
    yd = y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y
    return np.column_stack((fx * xd + cx, fy * yd + cy))


class Problem:
    """Lens + per-frame carton pose + per-tag placement, over the tag sightings (view, tag)."""
    N_INTR = 8                                   # fx fy cx cy k1 k2 p1 p2 (k3 fixed)

    def __init__(self, views, obs):
        self.views = views
        self.obs = list(obs)                     # [(view index, tag id)]
        self.tag_ids = sorted({t for _, t in self.obs})
        self.tag_index = {t: i for i, t in enumerate(self.tag_ids)}
        self.nv, self.nt = len(views), len(self.tag_ids)
        frames = [G.tag_frame(t) for t in self.tag_ids]
        self.tag_r = np.array([f[0] for f in frames])                 # carton <- tag (nominal)
        self.tag_c = np.array([f[1] for f in frames])
        self.local = np.array([G.tag_local_corners(t) for t in self.tag_ids])
        self.img = np.array([views[v].tags[t] for v, t in self.obs])  # (n_obs, 4, 2)
        self.ov = np.array([v for v, _ in self.obs])
        self.ot = np.array([self.tag_index[t] for _, t in self.obs])
        self.sigma = np.array([PRIOR[G.TAGS[t].rigid] for t in self.tag_ids])   # (nt, 2)
        self.k3 = 0.0

    def split(self, x):
        n = self.N_INTR
        intr = np.r_[x[:n], self.k3]
        poses = x[n:n + 6 * self.nv].reshape(self.nv, 6)
        tags = x[n + 6 * self.nv:].reshape(self.nt, 6)
        return intr, poses, tags

    def corners_carton(self, tags):
        """(nt, 4, 3) tag corners in the carton frame with each tag's placement correction (about its centre)."""
        r = _rotvec_to_matrix(tags[:, :3]) @ self.tag_r
        return np.einsum('tij,tkj->tki', r, self.local) + (self.tag_c + tags[:, 3:])[:, None, :]

    def predict(self, x):
        intr, poses, tags = self.split(x)
        pts = self.corners_carton(tags)[self.ot]                     # (n_obs, 4, 3)
        r = _rotvec_to_matrix(poses[self.ov, :3])                    # camera <- carton
        cam = np.einsum('oij,okj->oki', r, pts) + poses[self.ov, None, 3:]
        return distort_project(cam.reshape(-1, 3), intr).reshape(-1, 4, 2), cam[..., 2]

    def residuals(self, x):
        uv, _ = self.predict(x)
        _, _, tags = self.split(x)
        prior = np.column_stack((tags[:, :3] / self.sigma[:, 1:2], tags[:, 3:] / self.sigma[:, :1])).ravel()
        return np.r_[(uv - self.img).ravel(), prior]

    def sparsity(self):
        from scipy.sparse import lil_matrix
        n_obs, n = len(self.obs), self.N_INTR
        a = lil_matrix((n_obs * 8 + self.nt * 6, n + 6 * (self.nv + self.nt)), dtype=int)
        for o, (v, t) in enumerate(zip(self.ov, self.ot)):
            rows = slice(8 * o, 8 * o + 8)
            a[rows, :n] = 1
            a[rows, n + 6 * v:n + 6 * v + 6] = 1
            a[rows, n + 6 * (self.nv + t):n + 6 * (self.nv + t) + 6] = 1
        for t in range(self.nt):
            for j in range(6):
                a[8 * n_obs + 6 * t + j, n + 6 * (self.nv + t) + j] = 1
        return a

    def solve(self, x0, loss='linear'):
        from scipy.optimize import least_squares
        return least_squares(self.residuals, x0, jac_sparsity=self.sparsity(), method='trf', loss=loss, f_scale=1.0,
                             x_scale='jac', max_nfev=2000, xtol=1e-8, ftol=1e-8, gtol=1e-8)

    def per_obs_rms(self, x):
        uv, _ = self.predict(x)
        return np.sqrt(((uv - self.img) ** 2).sum(-1).mean(-1))


# ------------------------------------------------------------------------------------------------ calibration
def nominal_points(view: View):
    obj = np.concatenate([G.tag_corners(t) for t in view.tags]).astype(np.float32)
    img = np.concatenate([view.tags[t] for t in view.tags]).astype(np.float32)
    return obj, img


def calibrate(views: list[View], *, fovy_guess_deg=90.0, min_views=MIN_VIEWS, max_rms=MAX_RMS_PX,
              min_tags=2) -> dict:
    """Lens from tag sightings. Raises CalibrationRefused (with the report) when it cannot be trusted."""
    if not views:
        raise CalibrationRefused('No readable frames')
    size = views[0].size
    w, h = size
    usable = [v for v in views if len(v.tags) >= min_tags]
    skipped = {v.name: f'{len(v.tags)} clean tag(s): {sorted(v.tags)}; rejected {v.rejected}'
               for v in views if len(v.tags) < min_tags}
    report = {'method': 'box_tags', 'width': w, 'height': h, 'frames_read': len(views), 'photos_used': len(usable),
              'frames_skipped': skipped}
    if len(usable) < min_views:
        raise CalibrationRefused(f'Only {len(usable)} frame(s) show >= {min_tags} clean box tags (need {min_views}); '
                                 'capture more views of the tagged carton', report)
    f0 = h / 2 / math.tan(math.radians(fovy_guess_deg) / 2)
    k0 = np.array([[f0, 0, (w - 1) / 2], [0, f0, (h - 1) / 2], [0, 0, 1]])
    obj, img = zip(*(nominal_points(v) for v in usable))
    flags = cv2.CALIB_USE_INTRINSIC_GUESS | cv2.CALIB_FIX_K3
    # Stage 1: OpenCV on the nominal (flaps upright, printed positions) layout.
    try:
        rms_a, k_a, dist_a, rvecs, tvecs = cv2.calibrateCamera(list(obj), list(img), size, k0.copy(), np.zeros(5),
                                                                flags=flags)
        if not (0.33 * f0 < k_a[1, 1] < 3 * f0 and np.isfinite(rms_a)):
            raise cv2.error('implausible focal length')
    except cv2.error:
        rms_a, k_a, dist_a = None, k0, np.zeros(5)
        rvecs, tvecs = [], []
        for o, i in zip(obj, img):
            ok, rv, tv = cv2.solvePnP(o, i, k0, None, flags=cv2.SOLVEPNP_SQPNP)
            rvecs.append(rv)
            tvecs.append(tv)
    report['nominal_layout_rms_px'] = None if rms_a is None else float(rms_a)
    # Stage 2: bundle adjustment with per-tag placement.
    obs = [(i, t) for i, v in enumerate(usable) for t in v.tags]
    problem = Problem(usable, obs)
    d = np.ravel(dist_a)
    x = np.r_[k_a[0, 0], k_a[1, 1], k_a[0, 2], k_a[1, 2], d[0], d[1], d[2], d[3],
              np.concatenate([np.r_[np.ravel(r), np.ravel(t)] for r, t in zip(rvecs, tvecs)]), np.zeros(6 * problem.nt)]
    x = problem.solve(x, loss='soft_l1').x
    err = problem.per_obs_rms(x)
    keep = err <= OUTLIER_PX
    dropped = {f'{usable[v].name}:{t}': round(float(e), 2) for (v, t), e, k in zip(obs, err, keep) if not k}
    if not keep.all():
        trimmed = []
        for i, v in enumerate(usable):
            kept = {t: c for t, c in v.tags.items() if keep[obs.index((i, t))]}
            if len(kept) >= min_tags:
                trimmed.append(View(v.name, v.size, kept, v.rejected))
        if len(trimmed) < min_views:
            report['dropped_sightings_px'] = dropped
            raise CalibrationRefused(f'Only {len(trimmed)} frame(s) keep >= {min_tags} tags after dropping sightings '
                                     f'above {OUTLIER_PX} px (need {min_views})', report)
        index = {v.name: i for i, v in enumerate(usable)}
        rows = [index[v.name] for v in trimmed]
        n = Problem.N_INTR
        poses = x[n:n + 6 * len(usable)].reshape(-1, 6)[rows]
        usable = trimmed
        obs = [(i, t) for i, v in enumerate(usable) for t in v.tags]
        old_tags = dict(zip(problem.tag_ids, x[n + 6 * problem.nv:].reshape(-1, 6)))
        problem = Problem(usable, obs)
        x = np.r_[x[:n], poses.ravel(), np.concatenate([old_tags.get(t, np.zeros(6)) for t in problem.tag_ids])]
    result = problem.solve(x, loss='linear')
    x = result.x
    intr, poses, tags = problem.split(x)
    uv, depth = problem.predict(x)
    sq = ((uv - problem.img) ** 2).sum(-1)                         # (n_obs, 4)
    rms = float(np.sqrt(sq.mean()))
    per_image = {}
    for i, v in enumerate(usable):
        sel = problem.ov == i
        per_image[v.name] = {'rms_px': round(float(np.sqrt(sq[sel].mean())), 3), 'tags': sorted(v.tags)}
    fx, fy, cx, cy = intr[:4]
    # 1-sigma of the focal length from the final Jacobian (pixel noise taken from the residuals).
    try:
        with np.errstate(all='ignore'):        # Accelerate BLAS raises spurious matmul warnings on macOS
            jac = result.jac.toarray() if hasattr(result.jac, 'toarray') else np.asarray(result.jac)
            dof = max(1, 8 * len(obs) - len(x))
            s2 = float((uv - problem.img).ravel() @ (uv - problem.img).ravel()) / dof
            cov = np.linalg.pinv(jac.T @ jac) * s2
            fy_sd = float(math.sqrt(max(cov[1, 1], 0)))
    except (np.linalg.LinAlgError, ValueError):
        fy_sd = float('nan')
    vfov = math.degrees(2 * math.atan(h / (2 * fy)))
    dvfov = abs(math.degrees(2 * math.atan(h / (2 * (fy + fy_sd)))) - vfov) if math.isfinite(fy_sd) else None
    report.update({
        'fx': float(fx), 'fy': float(fy), 'cx': float(cx), 'cy': float(cy),
        'distortion': [float(v) for v in (intr[4], intr[5], intr[6], intr[7], intr[8])],
        'rms_reprojection_px': rms, 'photos_used': len(usable),
        'vertical_fov_degrees': vfov, 'horizontal_fov_degrees': math.degrees(2 * math.atan(w / (2 * fx))),
        'vertical_fov_sd_degrees': dvfov,
        'per_image_rms_px': per_image,
        'tag_sightings': len(obs),
        'tag_adjustments': {int(t): {'moved_mm': round(float(np.linalg.norm(tags[k, 3:])) * 1000, 2),
                                     'turned_deg': round(math.degrees(float(np.linalg.norm(tags[k, :3]))), 2)}
                            for k, t in enumerate(problem.tag_ids)},
        'dropped_sightings_px': dropped,
        'fovy_guess_degrees': fovy_guess_deg,
        'model': 'OpenCV pinhole, distortion (k1, k2, p1, p2, k3) with k3 fixed at 0',
        'pixel_convention': 'OpenCV: pixel centres at integer coordinates',
        'solver_converged': bool(result.success), 'solver_evaluations': int(result.nfev),
    })
    if not result.success or not np.isfinite(rms) or (depth <= 0).any():
        raise CalibrationRefused(f'Bundle adjustment did not converge: {result.message}', report)
    if rms > max_rms:
        worst = sorted(per_image.items(), key=lambda kv: -kv[1]['rms_px'])[:3]
        raise CalibrationRefused(f'RMS reprojection error {rms:.2f} px is above {max_rms} px; worst frames: '
                                 f'{[(k, v["rms_px"]) for k, v in worst]}. Retake blurry frames, check the tags are '
                                 'flat and the carton did not move during capture', report)
    return report


def _clean(value):
    if isinstance(value, dict):
        return {str(k): _clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_clean(v) for v in value]
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    return value


def run(images, out: Path, *, fovy_guess_deg=90.0, min_views=MIN_VIEWS, max_rms=MAX_RMS_PX, rigid_only=False,
        min_margin=MIN_MARGIN) -> dict:
    views = detect_folder(images, ids=G.RIGID_IDS if rigid_only else None, min_margin=min_margin)
    result = calibrate(views, fovy_guess_deg=fovy_guess_deg, min_views=min_views, max_rms=max_rms)
    result['images'] = str(images)
    result['tags_used'] = 'walls and floor' if rigid_only else 'walls, floor and flaps'
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(_clean(result), indent=1) + '\n')
    return result


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--images', type=Path, required=True, help='folder of wrist frames (PNG/JPEG), one resolution')
    ap.add_argument('--out', type=Path, required=True, help='intrinsics JSON (written only when accepted)')
    ap.add_argument('--fovy-guess', type=float, default=90.0, help='starting vertical FOV, degrees (simulation: 90)')
    ap.add_argument('--min-views', type=int, default=MIN_VIEWS)
    ap.add_argument('--max-rms', type=float, default=MAX_RMS_PX)
    ap.add_argument('--rigid-only', action='store_true', help='ignore the four flap tags (flaps not upright or loose)')
    args = ap.parse_args(argv)
    try:
        result = run(args.images, args.out, fovy_guess_deg=args.fovy_guess, min_views=args.min_views,
                     max_rms=args.max_rms, rigid_only=args.rigid_only)
    except CalibrationRefused as exc:
        if exc.report:
            print(json.dumps(_clean(exc.report), indent=1), file=sys.stderr)
        print(f'REFUSED: {exc}', file=sys.stderr)
        return 2
    print(json.dumps(_clean({k: result[k] for k in ('fx', 'fy', 'cx', 'cy', 'distortion', 'vertical_fov_degrees',
                                                    'horizontal_fov_degrees', 'vertical_fov_sd_degrees',
                                                    'rms_reprojection_px', 'photos_used')}), indent=1))
    print(f'wrote {args.out}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
