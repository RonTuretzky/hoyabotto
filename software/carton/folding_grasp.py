"""Independent simulator contact evidence, never a substitute for real sensing."""
import numpy as np


class GraspIKMixin:
    """Preserve the left jaw-face normal under the ordinary 8 mm IK gate.

    Shared by offline pinch diagnostics. It does not change actuator forces,
    original joint limits, the carton state, or the passive contact model.
    """
    def ik(self, side, target, orientation=None):
        if isinstance(orientation, dict) and 'local_axis' in orientation and side == 'left':
            import mujoco
            from scipy.optimize import least_squares
            from carton.folding_sim import JOINTS
            ix = self.arm_indices[side][:5]
            ranges = self.model.jnt_range[[self.model.joint(side+'_'+j).id for j in JOINTS[:5]]]
            for indices in self.arm_indices.values():
                self.kin.qpos[indices] = self.data.qpos[indices]
            def objective(q):
                self.kin.qpos[ix] = q
                mujoco.mj_kinematics(self.model, self.kin)
                rotation = self.kin.body(side+'_gripper_link').xmat.reshape(3, 3)
                return np.r_[self.kin.site(self.control_sites[side]).xpos-target,
                             (rotation@orientation['local_axis']-orientation['direction'])*.2]
            solution = least_squares(objective,
                np.clip(self.seeds[side], ranges[:, 0]+1e-6, ranges[:, 1]-1e-6),
                bounds=(ranges[:, 0], ranges[:, 1]), max_nfev=200,
                ftol=1e-10, xtol=1e-10, gtol=1e-10)
            self.seeds[side] = solution.x.copy()
            return solution.x, float(np.linalg.norm(objective(solution.x)[:3]))
        return super().ik(side, target, orientation)


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
