"""Far-flap contact geometry diagnostics with original SO101 collision meshes.

Static diagnostics perform geometric planning in copied states. Hypothetical
flap angles are preparation, never executed folding evidence. The separate
fold_far_from_edge experiment commands simulated robot actuators using fresh
visual observations. Neither path commands hardware or resets a live carton.
"""
from __future__ import annotations
import math

import mujoco
import numpy as np
from scipy.optimize import least_squares

from carton.folding_paths import JointPathPlanner
from carton.folding_sim import JOINTS, FoldingSimulation, W, H


def refresh_geometry(model, data):
    """Evaluate geometry without constructing forces for hypothetical states."""
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)
    mujoco.mj_flex(model, data)
    mujoco.mj_collision(model, data)


def mesh_contact_vertices(model, data, side):
    """A small set of exact fixed-finger collision-mesh vertices, link local."""
    body = side + '_gripper_link'
    points = []
    for gid in range(model.ngeom):
        name = model.geom(gid).name
        if (not name.startswith(side + '_') or 'wrist_roll_follower' not in name
                or not model.geom_contype[gid]
                or model.geom_type[gid] != mujoco.mjtGeom.mjGEOM_MESH):
            continue
        mid = model.geom_dataid[gid]
        start, count = model.mesh_vertadr[mid], model.mesh_vertnum[mid]
        world = np.einsum('ij,kj->ki', data.geom_xmat[gid].reshape(3, 3), model.mesh_vert[start:start+count]) + data.geom_xpos[gid]
        local = np.einsum('ji,kj->ki', data.body(body).xmat.reshape(3, 3), world - data.body(body).xpos)
        points.extend(local)
    if not points:
        raise ValueError('Original collision meshes required')
    points = np.unique(np.round(points, 9), axis=0)
    # Only distal actual vertices. Wrist-body corners are not contact proposals.
    distal = points[points[:, 2] < points[:, 2].min() + .010]
    selected = {int(np.argmin(distal[:, 2]))}
    for axis in (0, 1):
        selected.add(int(np.argmin(distal[:, axis])))
        selected.add(int(np.argmax(distal[:, axis])))
    return [dict(body=body, local=distal[index].tolist()) for index in sorted(selected)]


def far_contact_target(box, degrees, along, radius, clearance=.0015):
    """Outside face point with panel thickness accounted for separately."""
    if not all(math.isfinite(value) for value in (degrees, along, radius, clearance)):
        raise ValueError('Finite far-contact geometry required')
    if not -.1855 <= along <= .1855 or not 0 < radius <= .14 or not 0 <= clearance <= .03:
        raise ValueError('Contact must remain on the declared far panel')
    theta = math.radians(degrees)
    local = np.array([along, W / 2 - radius * math.sin(theta) + clearance * math.cos(theta),
                      H + .0035 + radius * math.cos(theta) + clearance * math.sin(theta)])
    return box[:3, :3] @ local + box[:3, 3]


class FarContactGeometry:
    """Static diagnostic copy; no live plant, sensors, or dynamics controller."""
    def __init__(self, model, state, side, *, near_degrees=None, other_arm_park=False, short_degrees=None):
        if side not in ('left', 'right'):
            raise ValueError('Choose an original left or right arm')
        state = np.asarray(state, dtype=float)
        if state.shape != (model.nq,) or not np.isfinite(state).all():
            raise ValueError('A complete finite recorded qpos state is required')
        if near_degrees is not None:
            limits = model.joint('long_near_hinge').range
            if not math.isfinite(near_degrees) or not limits[0] <= math.radians(near_degrees) <= limits[1]:
                raise ValueError('Hypothetical near angle exceeds original hinge limits')
        self.model = model
        self.data = mujoco.MjData(model)
        self.data.qpos[:] = state
        self.side = side
        self.arm_indices = {s: [model.joint(s + '_' + j).qposadr[0] for j in JOINTS]
                            for s in ('left', 'right')}
        self.hypothetical_changes = {}
        if near_degrees is not None:
            self.data.qpos[model.joint('long_near_hinge').qposadr[0]] = math.radians(near_degrees)
            self.hypothetical_changes['long_near_degrees'] = near_degrees
        if short_degrees is not None:
            if not math.isfinite(short_degrees):
                raise ValueError('Finite hypothetical short angles required')
            for flap in ('short_left', 'short_right'):
                joint = model.joint(flap + '_hinge')
                angle = math.radians(short_degrees)
                if not joint.range[0] <= angle <= joint.range[1]:
                    raise ValueError('Hypothetical short angle exceeds original hinge limits')
                self.data.qpos[joint.qposadr[0]] = angle
                self.hypothetical_changes[flap + '_degrees'] = short_degrees
        if other_arm_park:
            other = 'left' if side == 'right' else 'right'
            # Upright compact encoder configuration, a diagnostic obstacle choice.
            values = [0., -.6, -.4, .9, 0., -.17]
            self.data.qpos[self.arm_indices[other]] = values
            self.hypothetical_changes[other + '_joints_radians'] = values
        self.data.qpos[self.arm_indices[side][5]] = -.17
        self.hypothetical_changes[side + '_gripper_radians'] = -.17
        refresh_geometry(model, self.data)
        self.limits = model.jnt_range[[model.joint(side + '_' + j).id for j in JOINTS[:5]]]
        self.ix = self.arm_indices[side][:5]
        self.kin = mujoco.MjData(model)
        self.kin.qpos[:] = self.data.qpos
        self.planner = JointPathPlanner(self, side, clearance=.006,
                                       allowed_flaps=('long_far_cardboard',))
        self.box = np.eye(4)
        self.box[:3, :3] = self.data.body('carton').xmat.reshape(3, 3)
        self.box[:3, 3] = self.data.body('carton').xpos
        self.vertices = mesh_contact_vertices(model, self.data, side)

    forbidden_contact = FoldingSimulation.forbidden_contact

    def point(self, q, vertex):
        self.kin.qpos[self.ix] = q
        mujoco.mj_kinematics(self.model, self.kin)
        body = self.kin.body(vertex['body'])
        return body.xpos + body.xmat.reshape(3, 3) @ vertex['local']

    def solve(self, target, vertex, seed, *, direction=None):
        seed = np.clip(seed, self.limits[:, 0] + 1e-7, self.limits[:, 1] - 1e-7)
        def residual(q):
            position = self.point(q, vertex) - target
            # Small null-space preference keeps repeated seed choices distinct;
            # it is not a reach or collision tolerance relaxation.
            rest = (q-seed) * .00008
            if direction is not None:
                axis = self.kin.body(self.side + '_gripper_link').xmat.reshape(3, 3)[:, 2]
                rest = np.r_[rest, (axis - direction) * .01]
            return np.r_[position, rest]
        result = least_squares(residual, seed, bounds=(self.limits[:, 0], self.limits[:, 1]),
                               max_nfev=100, ftol=1e-9, xtol=1e-9, gtol=1e-9)
        error = float(np.linalg.norm(self.point(result.x, vertex) - target))
        valid = error <= .008 and self.planner.valid(result.x)
        return dict(q=result.x.tolist(), error_mm=error*1000, clear=bool(valid),
                    collision=None if valid or error > .008 else self.planner.last_collision)

    def contact_geometry(self, q):
        """Closest exact jaw-to-far-panel distance, independent geometric score."""
        self.data.qpos[self.ix] = q
        refresh_geometry(self.model, self.data)
        distances = []
        panel = self.model.geom('long_far_cardboard').id
        for gid in range(self.model.ngeom):
            name = self.model.geom(gid).name
            if (not name.startswith(self.side + '_') or not self.model.geom_contype[gid]
                    or not any(kind in name for kind in ('wrist_roll_follower', 'moving_jaw'))):
                continue
            segment = np.zeros(6)
            distance = mujoco.mj_geomDistance(self.model, self.data, gid, panel, .20, segment)
            distances.append(dict(geom=name, distance_mm=float(distance)*1000,
                                  closest_points=segment.reshape(2, 3).tolist()))
        return min(distances, key=lambda row: row['distance_mm'])


def trace_far_sweep(geometry, candidate, *, start_degrees=-5.6, end_degrees=90.,
                    end_radius=.115, clearance=.0015, adapt_clearance=False, end_along=None):
    """Hypothetical moving-flap geometric trace, explicitly not dynamics.

    Changing far angle here only constructs the obstacles for each declared
    geometric waypoint. No executing FoldingSimulation instance is accepted.
    """
    if type(geometry) is not FarContactGeometry:
        raise TypeError('A standalone static diagnostic copy is required')
    q = np.asarray(candidate['q'], dtype=float)
    rows = []
    original = geometry.data.qpos.copy()
    far_ix = geometry.model.joint('long_far_hinge').qposadr[0]
    try:
        for theta in np.linspace(start_degrees, end_degrees, 97):
            radius = candidate['radius_m'] + (end_radius-candidate['radius_m']) * np.clip((theta-60)/30, 0, 1)
            geometry.data.qpos[far_ix] = math.radians(theta)
            geometry.kin.qpos[far_ix] = math.radians(theta)
            geometry.planner.data.qpos[far_ix] = math.radians(theta)
            along = candidate['along_m'] if end_along is None else candidate['along_m'] + (end_along-candidate['along_m'])*np.clip((theta-10)/50, 0, 1)
            target = far_contact_target(geometry.box, theta, along, radius, clearance)
            row = geometry.solve(target, candidate['vertex'], q)
            used_clearance = clearance
            if adapt_clearance:
                for offset in np.arange(clearance, .021, .0005):
                    proposal_target = far_contact_target(geometry.box, theta, along, radius, offset)
                    proposal = geometry.solve(proposal_target, candidate['vertex'], q)
                    distance = geometry.contact_geometry(proposal['q'])['distance_mm']
                    if proposal['clear'] and -.3 <= distance <= .8:
                        row = proposal
                        target = proposal_target
                        used_clearance = float(offset)
                        break
            row.update(along_m=float(along), clearance_m=used_clearance, degrees=float(theta), radius_m=float(radius), target_world=target.tolist())
            row['exact_jaw_distance'] = geometry.contact_geometry(row['q'])
            rows.append(row)
            if not row['clear']:
                break
            q = np.asarray(row['q'])
    finally:
        geometry.data.qpos[:] = original
        geometry.kin.qpos[:] = original
        geometry.planner.data.qpos[:] = original
        refresh_geometry(geometry.model, geometry.data)
    return dict(static_geometry_only=True, hypothetical_far_angles=True,
                executed_fold=False, full_task_complete=False,
                sweep_clear=bool(rows and rows[-1]['clear'] and rows[-1]['degrees'] >= end_degrees),
                start_degrees=start_degrees, end_degrees=end_degrees,
                candidate=candidate, waypoints=rows)


def validate_static_sweep(geometry, report):
    """Densely check joint interpolation while prescribing planning-copy flaps."""
    if type(geometry) is not FarContactGeometry:
        raise TypeError('A standalone static diagnostic copy is required')
    if not report.get('sweep_clear') or len(report.get('waypoints', [])) < 2:
        return dict(static_geometry_only=True, prescribed_far_angles=True,
                    dense_checks=0, clear=False, error='Incomplete or failed geometric sweep')
    original = geometry.data.qpos.copy()
    far_ix = geometry.model.joint('long_far_hinge').qposadr[0]
    checked = 0
    error = None
    try:
        for a, b in zip(report['waypoints'], report['waypoints'][1:]):
            qa, qb = np.asarray(a['q']), np.asarray(b['q'])
            steps = max(5, math.ceil(float(np.max(np.abs(qb-qa))) / .005))
            for u in np.linspace(0, 1, steps+1):
                theta = (1-u)*a['degrees'] + u*b['degrees']
                geometry.planner.data.qpos[far_ix] = math.radians(theta)
                checked += 1
                if not geometry.planner.valid((1-u)*qa+u*qb):
                    error = dict(degrees=theta, collision=geometry.planner.last_collision)
                    break
            if error:
                break
    finally:
        geometry.data.qpos[:] = original
        geometry.planner.data.qpos[:] = original
        refresh_geometry(geometry.model, geometry.data)
    return dict(static_geometry_only=True, prescribed_far_angles=True,
                dense_checks=checked, clear=error is None, error=error)


def plan_static_approach(geometry, candidate, park_joints, *, height=.02):
    """6 mm free transit followed by checked intended far-edge approach."""
    if type(geometry) is not FarContactGeometry:
        raise TypeError('A standalone static diagnostic copy is required')
    original = geometry.data.qpos.copy()
    target = np.asarray(candidate['target_world']) + np.array([0., 0., height])
    pre = geometry.solve(target, candidate['vertex'], np.asarray(candidate['q']))
    if not pre['clear']:
        raise ValueError('Pre-contact pose fails original IK or clearance gate')
    try:
        geometry.data.qpos[geometry.ix] = park_joints
        transit = JointPathPlanner(geometry, geometry.side, clearance=.006).plan(pre['q'])
        geometry.data.qpos[geometry.ix] = pre['q']
        approach = JointPathPlanner(geometry, geometry.side, clearance=.006,
                                    allowed_flaps=('long_far_cardboard',))
        if not approach.edge(np.asarray(pre['q']), np.asarray(candidate['q'])):
            raise ValueError(f'Edge approach intersects: {approach.last_collision}')
    finally:
        geometry.data.qpos[:] = original
        refresh_geometry(geometry.model, geometry.data)
    return dict(static_geometry_only=True, free_transit_clearance_m=.006,
                allowed_panel_penetration_m=.001, pre_height_m=height,
                transit=[q.tolist() for q in transit],
                contact_approach=[pre['q'], candidate['q']])


def scan_bridge_clearance(model, state, *, y_values=(-.08, -.04, 0., .04, .08, .11),
                          height=.111, near_angles=(-15., 0., 15., 30., 45., 60., 75., 90.)):
    """Alternative two-jaw bridge poses and near-flap sweep, static only."""
    from carton.folding_retention import support_vertices
    data = mujoco.MjData(model)
    data.qpos[:] = state
    refresh_geometry(model, data)
    vertices = support_vertices(model, data)
    ix = np.array([model.joint('right_' + j).qposadr[0] for j in JOINTS])
    limits = model.jnt_range[[model.joint('right_' + j).id for j in JOINTS]]
    arm_indices = {s: [model.joint(s+'_'+j).qposadr[0] for j in JOINTS] for s in ('left','right')}
    sim = type('PlanningCopy', (), dict(model=model, data=data, arm_indices=arm_indices,
                                       forbidden_contact=FoldingSimulation.forbidden_contact))()
    planner = JointPathPlanner(sim, 'right', clearance=.006,
                               allowed_flaps=('short_left_cardboard','short_right_cardboard'))
    kin = mujoco.MjData(model);kin.qpos[:] = state
    box = np.eye(4);box[:3,:3] = data.body('carton').xmat.reshape(3,3);box[:3,3] = data.body('carton').xpos
    original_q = data.qpos[ix].copy()
    def endpoints(q):
        kin.qpos[ix] = q;mujoco.mj_kinematics(model,kin)
        return np.array([kin.body(body).xpos+kin.body(body).xmat.reshape(3,3)@point
                         for body,point in vertices.items()])
    def valid(q):
        planner.data.qpos[ix[5]]=q[5]
        return planner.valid(q[:5])
    start_points = endpoints(original_q)
    near_ix = model.joint('long_near_hinge').qposadr[0]
    rows=[]
    for y in y_values:
        q=original_q.copy();path=[]
        end_points=np.array([[-.052,y,height],[.052,y,height]])@box[:3,:3].T+box[:3,3]
        row=dict(y_m=y,height_m=height,static_geometry_only=True,contact_sweep=[],near_sweep=[])
        planner.data.qpos[:] = state
        for u in np.linspace(0,1,81)[1:]:
            target=(1-u)*start_points+u*end_points
            solution=least_squares(lambda values:(endpoints(values)-target).ravel(),
                np.clip(q,limits[:,0]+1e-8,limits[:,1]-1e-8),bounds=(limits[:,0],limits[:,1]),
                max_nfev=180,ftol=1e-10,xtol=1e-10,gtol=1e-10)
            goal=solution.x
            error=float(np.max(np.linalg.norm(endpoints(goal)-target,axis=1)))
            steps=max(1,math.ceil(float(np.max(np.abs(goal-q)))/.025))
            clear=error<=.002 and all(valid(q+(goal-q)*v) for v in np.linspace(0,1,steps+1))
            row['contact_sweep'].append(dict(progress=float(u),error_mm=error*1000,clear=clear,
                collision=None if clear or error>.002 else planner.last_collision))
            if not clear:break
            path.append(goal.tolist());q=goal
        row['shift_clear']=bool(len(path)==80);row['q']=q.tolist();row['path']=path
        if row['shift_clear']:
            for theta in near_angles:
                planner.data.qpos[near_ix]=math.radians(theta)
                clear=valid(q)
                row['near_sweep'].append(dict(degrees=theta,clear=clear,
                    collision=None if clear else planner.last_collision))
        rows.append(row)
    return dict(static_geometry_only=True, executed_bridge_shift=False,
                full_task_complete=False, source_vertices={k:v.tolist() for k,v in vertices.items()},
                near_sweep_angles_prescribed_in_planning_copy=True, rows=rows)


def scan_diagonal_bridges(model, state, *, seed=2, random_seeds=12, height=.111):
    """Exact CAD fingertip bridge placement and alternate IK branch scan."""
    from carton.folding_retention import support_vertices
    data=mujoco.MjData(model);data.qpos[:]=state;refresh_geometry(model,data)
    vertices=support_vertices(model,data)
    ix=np.array([model.joint('right_'+j).qposadr[0] for j in JOINTS])
    limits=model.jnt_range[[model.joint('right_'+j).id for j in JOINTS]]
    kin=mujoco.MjData(model);kin.qpos[:]=state
    box=np.eye(4);box[:3,:3]=data.body('carton').xmat.reshape(3,3);box[:3,3]=data.body('carton').xpos
    arm_indices={s:[model.joint(s+'_'+j).qposadr[0] for j in JOINTS] for s in ('left','right')}
    sim=type('PlanningCopy',(),dict(model=model,data=data,arm_indices=arm_indices,
                                   forbidden_contact=FoldingSimulation.forbidden_contact))()
    planner=JointPathPlanner(sim,'right',clearance=.006,
                            allowed_flaps=('short_left_cardboard','short_right_cardboard'))
    near_ix=model.joint('long_near_hinge').qposadr[0]
    near_angles=np.arange(-15.,91.,5.)
    rng=np.random.default_rng(seed)
    seeds=[data.qpos[ix].copy()]+[rng.uniform(limits[:,0],limits[:,1]) for _ in range(random_seeds)]
    def points(q):
        kin.qpos[ix]=q;mujoco.mj_kinematics(model,kin)
        return np.array([kin.body(b).xpos+kin.body(b).xmat.reshape(3,3)@v for b,v in vertices.items()])
    rows=[]
    for swap in (False,True):
        for half_x in (.052,.060,.065):
            for center_y in (-.08,-.04,0.,.04,.08):
                for difference_y in (-.08,-.04,0.,.04,.08):
                    if math.hypot(2*half_x,difference_y)>.1413:continue
                    local=np.array([[-half_x,center_y-difference_y/2,height],
                                    [half_x,center_y+difference_y/2,height]])
                    if swap:local=local[::-1]
                    target=local@box[:3,:3].T+box[:3,3]
                    for index,initial in enumerate(seeds):
                        solution=least_squares(lambda q:(points(q)-target).ravel(),
                            np.clip(initial,limits[:,0]+1e-8,limits[:,1]-1e-8),bounds=(limits[:,0],limits[:,1]),
                            max_nfev=100,ftol=1e-9,xtol=1e-9,gtol=1e-9)
                        q=solution.x;error=float(np.max(np.linalg.norm(points(q)-target,axis=1)))
                        row=dict(height_m=height,half_x_m=half_x,center_y_m=center_y,difference_y_m=difference_y,
                                 swap=swap,seed_index=index,error_mm=error*1000,q=q.tolist(),local_targets=local.tolist(),
                                 clearance_angles=[],first_collision=None)
                        if error<=.002:
                            for theta in near_angles:
                                planner.data.qpos[near_ix]=math.radians(theta);planner.data.qpos[ix[5]]=q[5]
                                if not planner.valid(q[:5]):
                                    row['first_collision']=dict(near_degrees=float(theta),collision=planner.last_collision)
                                    break
                                row['clearance_angles'].append(float(theta))
                        rows.append(row)
    return dict(static_geometry_only=True,executed_bridge_shift=False,
                near_angles_prescribed_in_copy=True,full_task_complete=False,
                rows=rows)


# Explicit controller proposal derived from the labeled static CAD trace.
# These are contact-point offsets, not prescribed flap states or observations.
_FAR_HOOK_OFFSET_SCHEDULE = np.array([
    [-15.00000000, 0.0015],
    [-3.60833333, 0.0015],
    [-2.61250000, 0.0020],
    [10.33333333, 0.0020],
    [11.32916667, 0.0055],
    [12.32500000, 0.0055],
    [13.32083333, 0.0085],
    [14.31666667, 0.0090],
    [15.31250000, 0.0100],
    [16.30833333, 0.0105],
    [17.30416667, 0.0105],
    [18.30000000, 0.0110],
    [19.29583333, 0.0105],
    [20.29166667, 0.0100],
    [21.28750000, 0.0095],
    [22.28333333, 0.0090],
    [23.27916667, 0.0085],
    [24.27500000, 0.0080],
    [25.27083333, 0.0080],
    [26.26666667, 0.0075],
    [27.26250000, 0.0075],
    [28.25833333, 0.0070],
    [29.25416667, 0.0070],
    [30.25000000, 0.0065],
    [32.24166667, 0.0065],
    [33.23750000, 0.0060],
    [36.22500000, 0.0060],
    [37.22083333, 0.0055],
    [42.20000000, 0.0055],
    [43.19583333, 0.0050],
    [49.17083333, 0.0050],
    [50.16666667, 0.0045],
    [58.13333333, 0.0045],
    [59.12916667, 0.0050],
    [62.11666667, 0.0050],
    [63.11250000, 0.0055],
    [64.10833333, 0.0055],
    [65.10416667, 0.0060],
    [67.09583333, 0.0060],
    [68.09166667, 0.0065],
    [70.08333333, 0.0065],
    [71.07916667, 0.0070],
    [73.07083333, 0.0070],
    [74.06666667, 0.0075],
    [75.06250000, 0.0075],
    [76.05833333, 0.0080],
    [78.05000000, 0.0080],
    [79.04583333, 0.0085],
    [81.03750000, 0.0085],
    [82.03333333, 0.0090],
    [85.02083333, 0.0090],
    [86.01666667, 0.0095],
    [88.00833333, 0.0095],
    [89.00416667, 0.0100],
    [90.00000000, 0.0100],
])
_FAR_HOOK_VERTEX = np.array([-.010900730,-.006113951,-.094425959])
_FAR_HOOK_SEED = np.array([0.10047303510419783, 0.93061275394957, -1.2177548644221954, 0.3666092395378051, -1.9085400298207411])
_FAR_HOOK_INITIAL_ALONG = .180


class RobotVertexIK:
    """Original CAD FK and original arm bounds; no object pose enters targets."""
    def __init__(self, sim, side, vertex):
        self.sim=sim;self.side=side;self.vertex=vertex
        self.kin=mujoco.MjData(sim.model)
        self.ix=sim.arm_indices[side][:5]
        self.limits=sim.model.jnt_range[[sim.model.joint(side+'_'+j).id for j in JOINTS[:5]]]
        for indices in sim.arm_indices.values():
            self.kin.qpos[indices]=sim.data.qpos[indices]

    def point(self,q):
        self.kin.qpos[self.ix]=q
        mujoco.mj_kinematics(self.sim.model,self.kin)
        body=self.kin.body(self.vertex['body'])
        return body.xpos+body.xmat.reshape(3,3)@self.vertex['local']

    def solve(self,target,seed):
        seed=np.clip(seed,self.limits[:,0]+1e-7,self.limits[:,1]-1e-7)
        solution=least_squares(lambda q:np.r_[self.point(q)-target,(q-seed)*.00008],
            seed,bounds=(self.limits[:,0],self.limits[:,1]),
            max_nfev=150,ftol=1e-10,xtol=1e-10,gtol=1e-10)
        error=float(np.linalg.norm(self.point(solution.x)-target))
        if error>.008:
            raise ValueError(f'Actual CAD far-contact IK exceeds 8 mm: {error*1000:.2f}')
        return solution.x,error


def fold_far_from_edge(sim,controller,*,capture=False,slide_left_to=-.08):
    """Physical right-claw far-edge attempt after a visually held near closure.

    The known narrow CAD hook/drag path is only a proposal. It is generated
    from current rendered RGB-D box registration and then commanded through
    robot joints. No executing or planning flap angle is assigned here. The
    simulator-backed transit obstacle copy is an offline diagnostic, not a
    calibrated hardware obstacle model. Success still requires fresh images.
    """
    from carton.folding_progress import ContactProgressGuard
    from carton.folding_paths import execute_path
    c=controller
    if slide_left_to is not None and (not math.isfinite(slide_left_to) or not -.12<=slide_left_to<=-.04):
        raise ValueError('Left near hold slide must be between -120 and -40 mm')
    report=dict(simulation_only=True,hardware_commands=0,full_task_complete=False,
                held_only=True,stage='register far flap',joint_driven_physics=True,
                target_source='fresh RGB-D box pose and far angle plus declared CAD contact proposal',
                contact_offset_schedule=_FAR_HOOK_OFFSET_SCHEDULE.tolist(),checks=[])
    c.far_edge_attempt=report

    def settle_soft_joint_overshoot(side):
        indices=sim.arm_indices[side][:5]
        limits=sim.model.jnt_range[[sim.model.joint(side+'_'+joint).id for joint in JOINTS[:5]]]
        actual=sim.data.qpos[indices].copy()
        if np.any(actual<limits[:,0]) or np.any(actual>limits[:,1]):
            # MuJoCo joints have compliant stops. Correct the executing robot
            # with an in-range command; never clip or rewrite its actual qpos.
            bounded=np.clip(actual,limits[:,0]+.0002,limits[:,1]-.0002)
            event=sim.move({},.30,'Physically settle '+side+' within original joint bounds',
                           capture=capture,joint_targets={side:bounded})
            report.setdefault('bounded_joint_settles',[]).append(dict(
                side=side,before=actual.tolist(),command=bounded.tolist(),
                after=sim.data.qpos[indices].tolist()))
            if event.get('step_error') or event['bad_penetration_mm']>1:
                raise ValueError(event.get('step_error') or 'Collision during bounded joint settle')
            if event.get('max_joint_tracking_error_radians',0)>.08:
                raise ValueError('Bounded joint settle tracking exceeds 0.08 rad')

    reading=c.sense('Register far flap after physical near closure')
    c.require_folded(reading,['long_near'])
    if slide_left_to is not None:
        if not 85<=reading['angles']['long_near']['degrees']<=95:
            raise ValueError('Fresh near closure within 85--95 degrees required before hold slide')
        hold_guard=ContactProgressGuard('long_near',reading)
        report['left_hold_slide']=dict(target_box=[slide_left_to,-.0365,.1215],
                                      visual_checks=hold_guard.checks)
        actual=sim.actual_control_position('left').copy()
        start_local=c.box[:3,:3].T@(actual-c.box[:3,3])
        end_local=np.array([slide_left_to,-.0365,.1215])
        report['stage']='physically slide left near hold for far-arm clearance'
        settle_soft_joint_overshoot('left')
        for u in np.linspace(.025,1,40):
            target=(1-u)*start_local+u*end_local
            q,error=sim.ik('left',c.box[:3,:3]@target+c.box[:3,3],'down')
            if error>.008:
                raise ValueError('Left near-hold slide IK exceeds 8 mm')
            planner=JointPathPlanner(sim,'left',clearance=.006,allowed_flaps=('long_near_cardboard',))
            path=planner.plan(q)
            execute_path(sim,'left',path,'Physically slide left near hold toward side',capture=capture)
            if np.linalg.norm(sim.actual_control_position('left')-(c.box[:3,:3]@target+c.box[:3,3]))>.035:
                raise ValueError('Left near-hold Cartesian tracking exceeds 35 mm')
            reading=c.sense('Observe near support and free carton during left hold slide')
            hold_guard.check(reading,90.)
            if not 85<=reading['angles']['long_near']['degrees']<=95:
                raise ValueError('Near closure left 85--95 degree band during hold slide')
        reading=c.sense('Re-register far flap after physical left near-hold slide')
        c.require_folded(reading,['long_near'])
    observed=reading['angles'].get('long_far')
    if observed is None or not -15<=observed['degrees']<=15:
        raise ValueError('Fresh far angle in -15 to 15 degrees required for the declared edge hook')
    start=float(observed['degrees'])
    guard=ContactProgressGuard('long_far',reading)
    report['visual_progress_checks']=guard.checks
    vertices=mesh_contact_vertices(sim.model,sim.data,'right')
    vertex=min(vertices,key=lambda row:np.linalg.norm(np.asarray(row['local'])-_FAR_HOOK_VERTEX))
    if np.linalg.norm(np.asarray(vertex['local'])-_FAR_HOOK_VERTEX)>1e-6:
        raise ValueError('Declared hook vertex is absent from original SO101 CAD collision meshes')
    report['actual_cad_vertex']=vertex
    c.port.set_grippers({'right':-.17},.4,'Close parked right claw before far-edge approach')
    ik=RobotVertexIK(sim,'right',vertex)

    def target_at(degrees):
        along=_FAR_HOOK_INITIAL_ALONG+(.12-_FAR_HOOK_INITIAL_ALONG)*np.clip((degrees-10)/50,0,1)
        radius=.14+(.115-.14)*np.clip((degrees-60)/30,0,1)
        clearance=float(np.interp(degrees,_FAR_HOOK_OFFSET_SCHEDULE[:,0],
                                   _FAR_HOOK_OFFSET_SCHEDULE[:,1]))
        return far_contact_target(c.box,degrees,along,radius,clearance)

    def command(q,point,seconds,label):
        event=sim.move({},seconds,label,capture=capture,joint_targets={'right':q})
        if event.get('step_error') or event['bad_penetration_mm']>1:
            raise ValueError(event.get('step_error') or 'Forbidden contact during far-edge attempt')
        if event.get('max_joint_tracking_error_radians',0)>.08:
            raise ValueError('Far-edge joint tracking exceeds original 0.08 rad bound')
        body=sim.data.body(vertex['body'])
        actual=body.xpos+body.xmat.reshape(3,3)@vertex['local']
        if np.linalg.norm(actual-point)>.035:
            raise ValueError('Far-edge CAD point tracking exceeds original 35 mm Cartesian bound')
        report['checks'].append(dict(time=float(sim.data.time),label=label,
            independent_angles=sim.truth_angles(),motion=dict(sim.motion_stats),
            joint_tracking_radians=event.get('max_joint_tracking_error_radians'),
            max_robot_flap_penetration_mm=event['max_robot_flap_penetration_mm']))

    point=target_at(start)
    q_contact,error=ik.solve(point,_FAR_HOOK_SEED)
    pre=point+[0,0,.020]
    q_pre,error=ik.solve(pre,q_contact)
    settle_soft_joint_overshoot('right')
    path=JointPathPlanner(sim,'right',clearance=.006).plan(q_pre)
    report['stage']='collision-checked free far-edge approach'
    execute_path(sim,'right',path,'Reach 20 mm above far edge with closed right claw',capture=capture)
    reading=c.sense('Verify carton after free far-edge transit')
    guard.check(reading,start)
    previous=q_pre
    for u in np.linspace(.1,1,10):
        goal=(1-u)*pre+u*point
        q,error=ik.solve(goal,previous)
        planner=JointPathPlanner(sim,'right',clearance=.006,allowed_flaps=('long_far_cardboard',))
        path=planner.plan(q)
        execute_path(sim,'right',path,'Approach actual CAD hook at far edge',capture=capture)
        command(q,goal,.20,'Settle actual CAD hook at far edge')
        previous=q
        guard.check(c.sense('Observe far edge during contact approach'),start)
    report['stage']='joint-driven far-edge fold attempt'
    for theta in np.linspace(start,90.,max(2,math.ceil(90-start)+1))[1:]:
        goal=target_at(float(theta))
        q,error=ik.solve(goal,previous)
        command(q,goal,.25,f'Physically move far-edge contact toward {theta:.1f} degrees')
        previous=q
        reading=c.sense('Observe far-flap progress and free carton during contact')
        guard.check(reading,float(theta))
        c.require_folded(reading,['long_near'])
    c.port.move_arms({},2.,'Hold physically folded far flap',None)
    reading=c.sense('Verify visible far and near major holds')
    c.require_folded(reading,['long_far','long_near'])
    report.update(stage='far and near majors visually held',visual_angles=reading['angles'],
                  independent_final_angles=sim.truth_angles(),motion=dict(sim.motion_stats),
                  physical_far_fold_verified=True)
    return report
