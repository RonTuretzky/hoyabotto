"""Export the fold-policy training station + the XLeRobot model's cart/head to OBJ parts for Blender.

Parts (one OBJ object each, Z up, metres, training-scene frame: +x robot right, +y toward the table, z=0 table top):
  cart            upstream XLeRobot chassis, wheels, mast and head (arms excluded), moved into the training frame
  left_base, right_base          SO-101 base_link (the "arm base") from the training scene
  left_arm, right_arm            everything above the shoulder-pan joint
  table, carton                  training table and the tagged carton
"""
import json, sys
import numpy as np
import mujoco

TRIAL = '/Users/wk/Documents/ChatGPT/Hackatuson/output/fold-demos/batch-220-01/trial-020'
UPSTREAM = '/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/upstream/assets/robots/xlerobot/xlerobot.xml'
OUT = '/tmp/foldviz'
SPLIT = '--split' in sys.argv
FLAPS = ('short_left', 'short_right', 'long_far', 'long_near')
BASE_X, BASE_Y, BASE_PLANE_Z = -.09, .11, .7915 - .0624      # carton/xlerobot_cameras.py


def geom_mesh(m, d, g):
    """World-space vertices and triangle faces of one geom."""
    t = m.geom_type[g]
    R, p = d.geom_xmat[g].reshape(3, 3), d.geom_xpos[g]
    if t == mujoco.mjtGeom.mjGEOM_MESH:
        k = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[k]:m.mesh_vertadr[k] + m.mesh_vertnum[k]]
        f = m.mesh_face[m.mesh_faceadr[k]:m.mesh_faceadr[k] + m.mesh_facenum[k]]
    elif t == mujoco.mjtGeom.mjGEOM_BOX:
        s = m.geom_size[g]
        v = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * s
        f = np.array([[0, 1, 3], [0, 3, 2], [4, 6, 7], [4, 7, 5], [0, 4, 5], [0, 5, 1],
                      [2, 3, 7], [2, 7, 6], [0, 2, 6], [0, 6, 4], [1, 5, 7], [1, 7, 3]])
    elif t in (mujoco.mjtGeom.mjGEOM_CYLINDER, mujoco.mjtGeom.mjGEOM_CAPSULE):
        r, h, n = m.geom_size[g][0], m.geom_size[g][1], 32
        a = np.linspace(0, 2 * np.pi, n, endpoint=False)
        ring = np.c_[r * np.cos(a), r * np.sin(a)]
        v = np.r_[np.c_[ring, -h * np.ones(n)], np.c_[ring, h * np.ones(n)], [[0, 0, -h], [0, 0, h]]]
        f = []
        for i in range(n):
            j = (i + 1) % n
            f += [[i, j, n + j], [i, n + j, n + i], [2 * n, j, i], [2 * n + 1, n + i, n + j]]
        f = np.array(f)
    elif t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return None
    else:
        return None
    return (np.asarray(v) @ R.T) + p, np.asarray(f)


def rgba(m, g):
    k = m.geom_matid[g]
    return (m.mat_rgba[k] if k >= 0 else m.geom_rgba[g]).copy()


class Obj:
    def __init__(self):
        self.parts, self.colors = {}, {}

    def add(self, part, v, f, color):
        if color[3] < .05:
            return
        key = 'c' + ''.join(f'{int(round(c * 255)):02x}' for c in color[:3])
        self.colors[key] = color[:3]
        self.parts.setdefault(part, []).append((v, f, key))

    def write(self, path):
        lines, n = [f'mtllib {path.split("/")[-1][:-4]}.mtl'], 1
        for part, items in self.parts.items():
            lines.append(f'o {part}')
            for v, f, key in items:
                lines += [f'v {a:.5f} {b:.5f} {c:.5f}' for a, b, c in v]
                lines.append(f'usemtl {key}')
                lines += [f'f {a + n} {b + n} {c + n}' for a, b, c in f]
                n += len(v)
        open(path, 'w').write('\n'.join(lines) + '\n')
        mtl = []
        for key, (r, g_, b) in self.colors.items():
            mtl += [f'newmtl {key}', f'Kd {r:.4f} {g_:.4f} {b:.4f}', 'd 1.0', '']
        open(path[:-4] + '.mtl', 'w').write('\n'.join(mtl))


def main():
    out = Obj()
    info = {}
    # ---- training scene: arms, table, carton
    m = mujoco.MjModel.from_xml_path(TRIAL + '/run/scene.xml'); d = mujoco.MjData(m)
    z = np.load(TRIAL + '/demo.npz'); d.qpos[:] = z['qpos'][0]
    adr = m.jnt_qposadr[m.joint('carton_free').id]; d.qpos[adr:adr + 7] = [0, 0, .001, 1, 0, 0, 0]
    if SPLIT:
        for f in FLAPS:
            d.qpos[m.jnt_qposadr[m.joint(f + '_hinge').id]] = 0
    mujoco.mj_forward(m, d)
    if SPLIT:
        for f in FLAPS:
            j = m.joint(f + '_hinge').id
            info[f + '_hinge'] = {'anchor': d.xanchor[j].tolist(), 'axis': d.xaxis[j].tolist()}
    for g in range(m.ngeom):
        if m.geom_group[g] == 3:                       # collision copies
            continue
        b = m.geom_bodyid[g]; root = m.body(m.body_rootid[b]).name; name = m.body(b).name
        gname = m.geom(g).name
        if root in ('left_base_link', 'right_base_link'):
            side = root.split('_')[0]
            part = f'{side}_base' if name == root else f'{side}_arm'
        elif root == 'carton':
            part = 'carton'
            if SPLIT:
                b2 = b
                while m.body(b2).name not in FLAPS + ('carton',):
                    b2 = m.body_parentid[b2]
                part = 'carton' if m.body(b2).name == 'carton' else 'flap_' + m.body(b2).name
                if 'tag' in name.split('_'):
                    info.setdefault('tags', {})[name] = part
                    part = 'tag_' + name
        elif name == 'world' and gname == 'table':
            part = 'table'
        else:
            continue                                     # the simplified cart blocks and table tags
        mesh = geom_mesh(m, d, g)
        if mesh is not None:
            out.add(part, *mesh, rgba(m, g))
    for side in ('left', 'right'):
        j = m.joint(f'{side}_shoulder_pan').id
        info[f'{side}_pan_anchor'] = d.xanchor[j].tolist()
        info[f'{side}_base_origin'] = d.xpos[m.body(f'{side}_base_link').id].tolist()
    # ---- upstream XLeRobot: cart, wheels, mast, head (arms excluded), moved into the training frame
    u = mujoco.MjModel.from_xml_path(UPSTREAM); ud = mujoco.MjData(u); mujoco.mj_forward(u, ud)
    arm_roots = {'Base', 'Base_2'}
    def in_arm(b):
        while b != 0:
            if u.body(b).name in arm_roots:
                return True
            b = u.body_parentid[b]
        return False
    mid = (np.array(info['left_base_origin']) + np.array(info['right_base_origin'])) / 2
    # model -> arm frame (x_arm = y_model, y_arm = -(x_model - BASE_X), z_arm = z_model - BASE_PLANE_Z) -> training
    A = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1.]])
    # The model's legacy arm bases sit ON the top-tray rim (z 0.775); an SO-101 base is bolted there instead, so the
    # rim is put at the bottom of the SO-101 bases (the training base plane).
    off = np.array([-BASE_X, 0, -0.775])
    count = 0
    for g in range(u.ngeom):
        if in_arm(u.geom_bodyid[g]) or u.geom_contype[g] != 0 and u.geom_group[g] == 3:
            continue
        mesh = geom_mesh(u, ud, g)
        if mesh is None:
            continue
        v, f = mesh
        v = ((v + off) @ A.T) + mid
        col = rgba(u, g)
        body = u.body(u.geom_bodyid[g]).name
        part = 'head_tilt' if body in ('head_tilt_link', 'head_camera_link') else ('head_pan' if body == 'head_pan_link' else 'cart')
        out.add(part if SPLIT else 'cart', v, f, col); count += 1
    info['cart_geoms'] = count
    for jn in ('head_pan_joint', 'head_tilt_joint'):
        j = u.joint(jn).id
        info[jn] = {'anchor': (((ud.xanchor[j] + off) @ A.T) + mid).tolist(), 'axis': (ud.xaxis[j] @ A.T).tolist()}
    out.write(OUT + ('/station_split.obj' if SPLIT else '/station.obj'))
    json.dump(info, open(OUT + ('/station_split.json' if SPLIT else '/station.json'), 'w'), indent=1)
    print(json.dumps(info, indent=1))
    for p, items in out.parts.items():
        v = np.vstack([i[0] for i in items]); print(p, len(items), 'geoms', np.round(v.min(0), 3), np.round(v.max(0), 3))


main()


def export_arm_poses(n=12):
    """Arm geometry at n+1 poses from a released, hanging-forward stance to the training start pose (same vertex order)."""
    m = mujoco.MjModel.from_xml_path(TRIAL + '/run/scene.xml'); d = mujoco.MjData(m)
    z = np.load(TRIAL + '/demo.npz'); start = z['qpos'][0].copy()
    arm = [m.jnt_qposadr[m.joint(f'{s}_{j}').id] for s in ('left', 'right') for j in
           ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_roll', 'gripper')]
    rest = start.copy()
    rest[arm] = [0, -1.2, 1.2, 0.6, 0, .35, 0, -1.2, 1.2, 0.6, 0, .35]       # arms raised in front, jaws open
    for k in range(n + 1):
        d.qpos[:] = start; d.qpos[arm] = rest[arm] + (start[arm] - rest[arm]) * k / n
        mujoco.mj_forward(m, d)
        o = Obj()
        for g in range(m.ngeom):
            if m.geom_group[g] == 3:
                continue
            b = m.geom_bodyid[g]; root = m.body(m.body_rootid[b]).name
            if root in ('left_base_link', 'right_base_link') and m.body(b).name != root and not m.body(b).name.endswith('_tag'):
                mesh = geom_mesh(m, d, g)
                if mesh is not None:
                    o.add(root.split('_')[0] + '_arm', *mesh, rgba(m, g))
        o.write(f'{OUT}/arms_pose_{k:02d}.obj')
    print('arm poses', n + 1)


if '--poses' in sys.argv:
    export_arm_poses()
