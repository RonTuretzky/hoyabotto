"""Independent simulator contact evidence, never a substitute for real sensing."""
import numpy as np


def opposing_faces(fixed, moving):
    """Require loaded contacts on opposite broad faces, not two edge touches."""
    return any(abs(f['normal_dot']) >= .8 and abs(m['normal_dot']) >= .8
               and f['normal_dot'] * m['normal_dot'] < 0
               and f['normal_force_N'] > .02 and m['normal_force_N'] > .02
               for f in fixed for m in moving)


def panel_grasp_evidence(model, data, side, flap):
    import mujoco
    fixed, moving = [], []
    normal = data.body(flap).xmat.reshape(3, 3)[:, 1 if flap.startswith('long') else 0]
    panel = flap + '_cardboard'
    for i, contact in enumerate(data.contact):
        a, b = model.geom(contact.geom1).name, model.geom(contact.geom2).name
        if panel not in (a, b):
            continue
        other = b if a == panel else a
        if not other.startswith(side + '_'):
            continue
        destination = (moving if 'moving_jaw' in other else
                       fixed if 'wrist_roll_follower' in other else None)
        if destination is None:
            continue
        force = np.zeros(6)
        mujoco.mj_contactForce(model, data, i, force)
        destination.append(dict(geom=other, normal_force_N=float(force[0]),
            normal_dot=float(normal @ contact.frame[:3]) * (1 if a == panel else -1)))
    return dict(time=float(data.time), fixed=fixed, moving=moving,
                opposing_faces=opposing_faces(fixed, moving))
