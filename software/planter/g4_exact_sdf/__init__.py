"""Exact source-triangle SDF plugin for G4 contact audits.

Output-only diagnostics around ``exact_mesh_sdf.cpp``: build it against the
installed MuJoCo, load it, query an instance through MuJoCo's own collision
entry points, and evaluate an independent double/long-double reference field
from the original STL. Nothing here commands hardware or claims task success.
"""
from __future__ import annotations

import ctypes
import hashlib
import platform
import shutil
import struct
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np

PLUGIN_NAME = 'g4.exact_mesh_sdf.v2'
SOURCE = Path(__file__).resolve().with_name('exact_mesh_sdf.cpp')
COMPILED_DEVIATION_LIMIT_M = 5e-8


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def mujoco_paths():
    root = Path(mujoco.__file__).resolve().parent
    suffix = 'dylib' if platform.system() == 'Darwin' else 'so'
    library = root/f'libmujoco.{mujoco.__version__}.{suffix}'
    if not library.exists():
        candidates = sorted(root.glob(f'libmujoco*.{suffix}'))
        if not candidates:
            raise FileNotFoundError(f'no MuJoCo shared library under {root}')
        library = candidates[0]
    return dict(include=root/'include', library=library, root=root, suffix=suffix)


def build(out_dir, *, source=SOURCE, compiler=None):
    """Compile the plugin into ``out_dir``; returns a manifest with hashes."""
    paths = mujoco_paths()
    compiler = compiler or shutil.which('clang++') or shutil.which('g++')
    if compiler is None:
        raise RuntimeError('no C++ compiler (clang++ or g++) on PATH')
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    library = out_dir/f'libg4_exact_mesh_sdf.{paths["suffix"]}'
    link = paths['library'].name
    link = link[3:link.index('.' + paths['suffix'])]  # libmujoco.3.14.0.dylib -> mujoco.3.14.0
    command = [compiler, '-std=c++17', '-O2', '-shared', '-fPIC', '-Wall', f'-I{paths["include"]}',
               '-o', str(library), str(source), f'-L{paths["root"]}', f'-l{link}', f'-Wl,-rpath,{paths["root"]}']
    subprocess.run(command, check=True, capture_output=True, text=True)
    return dict(plugin_name=PLUGIN_NAME, source=str(source), source_sha256=sha(source), library=str(library),
                library_sha256=sha(library), command=command, compiler=compiler, mujoco_version=mujoco.__version__,
                mujoco_library=str(paths['library']), mujoco_library_sha256=sha(paths['library']), python=sys.version)


_LOADED = {}


def load(library):
    """Register the plugin with MuJoCo once per process and keep the handle."""
    library = str(Path(library).resolve())
    if library not in _LOADED:
        handle = ctypes.CDLL(library)
        handle.g4_exact_mesh_sdf_mode.argtypes = [ctypes.c_void_p, ctypes.c_int]
        handle.g4_exact_mesh_sdf_mode.restype = ctypes.c_char_p
        handle.g4_exact_mesh_sdf_compiled_deviation.argtypes = [ctypes.c_void_p, ctypes.c_int]
        handle.g4_exact_mesh_sdf_compiled_deviation.restype = ctypes.c_double
        handle.g4_exact_mesh_sdf_counts.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int)]
        handle.g4_exact_mesh_sdf_set_collider.argtypes = [ctypes.c_int]
        handle.g4_exact_mesh_sdf_set_collider.restype = ctypes.c_int
        handle.g4_exact_mesh_sdf_stats.argtypes = [ctypes.POINTER(ctypes.c_longlong), ctypes.c_int]
        handle.g4_exact_mesh_sdf_stats.restype = None
        mujoco.mj_loadPluginLibrary(library)
        _LOADED[library] = handle
    return _LOADED[library]


def set_collider(handle, enable):
    """Route mesh/exact-SDF pairs through the plugin's exhaustive narrowphase (process-wide).

    Off by default, so earlier audits reproduce under MuJoCo's stock mesh/SDF
    collider. Pairs against any other SDF always use the stock collider.
    """
    active = bool(handle.g4_exact_mesh_sdf_set_collider(int(bool(enable))))
    if active != bool(enable):
        raise RuntimeError('exact mesh/SDF collider did not switch')
    return active


def collider_stats(handle, *, reset=True):
    """Exact-collider counters since the last reset (single-threaded diagnostics)."""
    out = (ctypes.c_longlong*4)()
    handle.g4_exact_mesh_sdf_stats(out, int(reset))
    return dict(calls=out[0], faces_past_cull=out[1], faces_frank_wolfe=out[2], penetrating_faces=out[3])


def instance_info(handle, model, data, geom):
    """Mode and self-check deviation of the plugin instance behind ``geom``."""
    geom = geom if isinstance(geom, int) else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
    instance = int(model.geom_plugin[geom])
    assert instance >= 0, 'geom has no plugin instance'
    vertices, faces = ctypes.c_int(), ctypes.c_int()
    handle.g4_exact_mesh_sdf_counts(data._address, instance, ctypes.byref(vertices), ctypes.byref(faces))
    return dict(instance=instance, mode=handle.g4_exact_mesh_sdf_mode(data._address, instance).decode(),
                compiled_deviation_m=handle.g4_exact_mesh_sdf_compiled_deviation(data._address, instance),
                vertices=vertices.value, faces=faces.value)


class _SDF(ctypes.Structure):
    _fields_ = [('plugin', ctypes.POINTER(ctypes.c_void_p)), ('id', ctypes.POINTER(ctypes.c_int)),
                ('type', ctypes.c_int), ('relpos', ctypes.POINTER(ctypes.c_double)),
                ('relmat', ctypes.POINTER(ctypes.c_double)), ('geomtype', ctypes.POINTER(ctypes.c_int))]


class PluginQuery:
    """Query a plugin SDF geom through libmujoco's own distance/gradient entry points."""

    def __init__(self, model, data, geom):
        self.m, self.d = model, data
        self.g = geom if isinstance(geom, int) else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
        lib = ctypes.CDLL(str(mujoco_paths()['library']))
        lib.mjc_getSDF.argtypes = [ctypes.c_void_p, ctypes.c_int]
        lib.mjc_getSDF.restype = ctypes.c_void_p
        lib.mjc_distance.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(_SDF), ctypes.POINTER(ctypes.c_double)]
        lib.mjc_distance.restype = ctypes.c_double
        lib.mjc_gradient.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.POINTER(_SDF), ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_double)]
        lib.mjc_gradient.restype = None
        self.lib = lib
        instance = int(model.geom_plugin[self.g])
        assert instance >= 0, 'expected a registered plugin SDF geom'
        pointer = lib.mjc_getSDF(model._address, self.g)
        assert pointer, 'mjc_getSDF returned null'
        self._plugins = (ctypes.c_void_p*1)(pointer)
        self._ids = (ctypes.c_int*1)(instance)
        self._types = (ctypes.c_int*1)(int(mujoco.mjtGeom.mjGEOM_SDF))
        self.sdf = _SDF(self._plugins, self._ids, 0, None, None, self._types)

    def local(self, point):
        """Signed distance (m) and gradient in the geom's own frame."""
        p, g = (ctypes.c_double*3)(*map(float, point)), (ctypes.c_double*3)()
        v = self.lib.mjc_distance(self.m._address, self.d._address, ctypes.byref(self.sdf), p)
        self.lib.mjc_gradient(self.m._address, self.d._address, ctypes.byref(self.sdf), g, p)
        return float(v), np.array(g)

    def body(self, point):
        """Query a point given in the parent body frame (m)."""
        bid = int(self.m.geom_bodyid[self.g])
        rb = self.d.xmat[bid].reshape(3, 3)
        rg = self.d.geom_xmat[self.g].reshape(3, 3)
        world = rb@np.asarray(point, dtype=float)+self.d.xpos[bid]
        v, g = self.local(rg.T@(world-self.d.geom_xpos[self.g]))
        return v, rb.T@rg@g

    def world(self, point):
        """Query a world-frame point (m); gradient returned in world frame."""
        rg = self.d.geom_xmat[self.g].reshape(3, 3)
        v, g = self.local(rg.T@(np.asarray(point, dtype=float)-self.d.geom_xpos[self.g]))
        return v, rg@g


def stl_triangles(path):
    """Binary STL triangles as float64 (n, 3, 3), in the file's own units."""
    raw = Path(path).read_bytes()
    if len(raw) < 84:
        raise ValueError('STL too short')
    count = struct.unpack('<I', raw[80:84])[0]
    if len(raw) != 84+50*count:
        raise ValueError('not a binary STL with a matching triangle count')
    record = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    return np.frombuffer(raw[84:], dtype=record)['vertices'].astype(np.float64)


def compiled_triangles(model, geom):
    """Compiled float32 triangles of a mesh geom promoted to float64 (m, geom frame)."""
    geom = geom if isinstance(geom, int) else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
    mid = int(model.geom_dataid[geom])
    va, nv = int(model.mesh_vertadr[mid]), int(model.mesh_vertnum[mid])
    fa, nf = int(model.mesh_faceadr[mid]), int(model.mesh_facenum[mid])
    vertices = model.mesh_vert[va:va+nv].astype(np.float64)
    return vertices[model.mesh_face[fa:fa+nf]]


def source_triangles_in_geom_frame(model, geom, stl_path):
    """Original STL triangles moved into the compiled geom frame, in double (m)."""
    geom = geom if isinstance(geom, int) else mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom)
    mid = int(model.geom_dataid[geom])
    rotation = np.zeros(9)
    mujoco.mju_quat2Mat(rotation, model.mesh_quat[mid])
    rotation = rotation.reshape(3, 3)
    moved = stl_triangles(stl_path)*model.mesh_scale[mid]-model.mesh_pos[mid]
    return np.einsum('ij,fkj->fki', rotation.T, moved)


def _stable_closest(triangles, point):
    """Nearest point on every triangle: long-double plane projection plus edges."""
    tri = np.asarray(triangles, dtype=np.longdouble)
    p = np.asarray(point, dtype=np.longdouble)
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    ab, ac = b-a, c-a
    normal = np.cross(ab, ac)
    n2 = np.sum(normal*normal, axis=1)
    projected = p-normal*(np.sum((p-a)*normal, axis=1)/n2)[:, None]
    ap = projected-a
    beta = np.sum(np.cross(ap, ac)*normal, axis=1)/n2
    gamma = np.sum(np.cross(ab, ap)*normal, axis=1)/n2
    inside = (beta >= 0) & (gamma >= 0) & (beta+gamma <= 1)
    candidates = [projected]
    for start, end in [(a, b), (b, c), (c, a)]:
        edge = end-start
        t = np.clip(np.sum((p-start)*edge, axis=1)/np.sum(edge*edge, axis=1), 0, 1)
        candidates.append(start+t[:, None]*edge)
    candidates = np.stack(candidates, axis=1)
    d2 = np.sum((candidates-p)**2, axis=2)
    d2[~inside, 0] = np.inf
    best = np.argmin(d2, axis=1)
    return candidates[np.arange(len(tri)), best]


def winding_number(triangles, point):
    """Generalized winding number of a closed triangle soup around ``point``."""
    t = np.asarray(triangles, dtype=np.float64)-np.asarray(point, dtype=np.float64)
    lengths = np.linalg.norm(t, axis=2)
    a, b, c = t[:, 0], t[:, 1], t[:, 2]
    numerator = np.einsum('ij,ij->i', a, np.cross(b, c))
    denominator = (np.prod(lengths, axis=1)+np.einsum('ij,ij->i', a, b)*lengths[:, 2]
                   +np.einsum('ij,ij->i', b, c)*lengths[:, 0]+np.einsum('ij,ij->i', c, a)*lengths[:, 1])
    return float(np.arctan2(numerator, denominator).sum()/(2*np.pi))


def reference_field(triangles, point, *, unique_band=1e-9, gradient_band=1e-5):
    """Independent signed distance, sign and (where unique) gradient at ``point``.

    Sign comes from the solid winding number, not from any pseudonormal; the
    gradient is only reported when the nearest point is unique within
    ``unique_band`` and the point lies farther than ``gradient_band`` from the
    surface (10 nm by default, in the triangles' units), so that
    non-differentiable loci and sub-rounding distances are not scored.
    """
    closest = _stable_closest(triangles, point)
    p = np.asarray(point, dtype=np.longdouble)
    distances = np.sqrt(np.sum((closest-p)**2, axis=1))
    index = int(np.argmin(distances))
    distance = float(distances[index])
    inside = abs(winding_number(triangles, point)) > .5
    near = distances <= distances[index]+unique_band
    unique = bool(np.max(np.sqrt(np.sum((closest[near]-closest[index])**2, axis=1))) < unique_band)
    gradient = None
    if distance > gradient_band and unique:
        gradient = np.asarray((p-closest[index])/distances[index], dtype=np.float64)*(-1 if inside else 1)
    return dict(signed_distance=-distance if inside else distance, unsigned_distance=distance, inside=inside,
                unique=unique, gradient=gradient, nearest_face=index,
                face_distances=np.asarray(distances, dtype=np.float64))


def face_normals(triangles):
    n = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    return n/np.linalg.norm(n, axis=1)[:, None]
