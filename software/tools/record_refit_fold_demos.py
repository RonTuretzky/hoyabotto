"""Record on the approved DCM scene. Offline, uncalibrated, with collision gates.

The synthetic group-4 table anchor is visible only to the scripted teacher;
fold_demos_to_lerobot hides it from policy images. No robot clients are used.
"""
from __future__ import annotations
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import record_fold_demos as recorder

PRELUDE = '''
from pathlib import Path
import json
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
import carton.folding_station as station_module
import carton.folding_sim as sim_module
from carton.xlerobot_cameras import BASE_PLANE_Z
from carton.folding_station_measured import look_at_axes, words
from carton.folding_refit_preview import configure_scene
from carton.refit_camera_contract import load_contract, save_contract
_station = station_module.FoldingStation
def refit_station(*args, **kwargs):
    return _station(BASE_PLANE_Z-.7, .18, .01, base_spacing=.22, table_size=(.5,.48),
        table_marker_xy=(-.20,.27))
station_module.FoldingStation = refit_station
_init = sim_module.FoldingSimulation.__init__
def refit_init(self, source, out, **kwargs):
    st = kwargs['station']
    # Start-pose jitter: the real arms reach the start pose only within a few ticks (the right wrist stops ~40
    # ticks short of its target), so each demo starts from a slightly different pose, seeded by the trial seed.
    import sys as _sys
    seed = int(_sys.argv[_sys.argv.index('--seed')+1]) if '--seed' in _sys.argv else 0
    jitter = {start_jitter_m!r}
    rng = np.random.default_rng(seed + 7919)
    kwargs['initial_arm_targets'] = {{s:(np.r_[sign*.16,st.base_y+.12,st.base_height+.36]
        + (rng.uniform(-jitter, jitter, 3) if jitter else 0.)).tolist()
        for s,sign in [('left',-1),('right',1)]}}
    kwargs['initial_right_roll'] = None
    _init(self, source, out, **kwargs)
    configure_scene(self, Path({upstream!r}), Path(out), controller_anchor=True)
    self.option.geomgroup[4] = 1
    contract = load_contract()
    self.model.cam_fovy[self.model.camera('front').id] = contract['policy']['vertical_fov_deg']
    tree=ET.parse(Path(out)/'scene.xml')
    tree.getroot().find("worldbody/camera[@name='front']").set('fovy',str(contract['policy']['vertical_fov_deg']))
    front_pose = {front_pose!r}
    if front_pose:
        # Measured head camera pose (arm-base frame): overrides the model's assumed head pose.
        fpos = np.asarray(front_pose['pos'], float); fx = np.asarray(front_pose['xyaxes'][:3], float)
        fy = np.asarray(front_pose['xyaxes'][3:], float)
        front = tree.getroot().find("worldbody/camera[@name='front']")
        front.set('pos', words(fpos)); front.set('xyaxes', words(np.r_[fx, fy]))
        fid = self.model.camera('front').id; self.model.cam_pos[fid] = fpos
        mujoco.mju_mat2Quat(self.model.cam_quat[fid], np.column_stack([fx, fy, np.cross(fx, fy)]).ravel())
    save_contract(out, contract)
    pos={teacher_position!r};rx,ry=look_at_axes(pos,[0,0,.15])
    teacher=tree.getroot().find("worldbody/camera[@name='station']")
    teacher.set('pos',words(pos));teacher.set('xyaxes',words(np.r_[rx,ry]))
    cid=self.model.camera('station').id;self.model.cam_pos[cid]=pos
    mujoco.mju_mat2Quat(self.model.cam_quat[cid],np.column_stack([rx,ry,np.cross(rx,ry)]).ravel())
    mujoco.mj_forward(self.model,self.data)
    tree.write(Path(out)/'scene.xml',encoding='unicode')
    (Path(out)/'refit-provenance.json').write_text(json.dumps(dict(
        station=st.report(), physical_registration_verified=False,
        head_input=contract['policy']['sampling'],
        camera_contract_sha256=contract['contract_sha256'],
        head_extrinsics=('measured head pose override: '+str(front_pose.get('source'))) if front_pose
            else 'model assumption; no calibrated transform from current head ticks',
        start_pose_jitter_m=jitter, teacher_anchor_group=4, collision_visual_group=3,
        hardware_commands=False), indent=2))
sim_module.FoldingSimulation.__init__ = refit_init
'''

def main(argv=None):
    argv = list(sys.argv[1:] if argv is None else argv)
    ap = argparse.ArgumentParser(description=__doc__)
    if '--' not in argv:
        ap.error('Separate station options from recorder options with --')
    split = argv.index('--')
    ap.add_argument('--upstream', type=Path, required=True)
    ap.add_argument('--along', type=float)
    ap.add_argument('--radius', type=float)
    ap.add_argument('--pinch-normal-tilt-degrees', type=float)
    ap.add_argument('--pre-height', type=float)
    ap.add_argument('--clearance', type=float)
    ap.add_argument('--axis-sign', type=int, choices=(-1,1))
    ap.add_argument('--teacher-position', type=float, nargs=3, default=[-.5,-.7,.75])
    ap.add_argument('--start-jitter-m', type=float, default=0.,
                    help='uniform +/- jitter of each arm\'s initial Cartesian target (m), seeded per trial')
    ap.add_argument('--front-camera-pose', type=Path,
                    help='JSON {pos:[3], xyaxes:[6], source:...}: measured head camera pose replacing the model one')
    args = ap.parse_args(argv[:split])
    if args.start_jitter_m < 0 or args.start_jitter_m > .03:
        ap.error('--start-jitter-m must be within 0..0.03')
    front_pose = json.loads(args.front_camera_pose.read_text()) if args.front_camera_pose else None
    if front_pose and (len(front_pose.get('pos', [])) != 3 or len(front_pose.get('xyaxes', [])) != 6):
        ap.error('--front-camera-pose needs pos[3] and xyaxes[6]')
    for flag in ('--along', '--radius', '--pinch-normal-tilt-degrees', '--pre-height', '--axis-sign', '--clearance'):
        value = getattr(args, flag[2:].replace('-', '_'))
        if value is not None:
            if flag in recorder.BASE_ARGS:
                recorder.BASE_ARGS[recorder.BASE_ARGS.index(flag)+1] = str(value)
            else:
                recorder.BASE_ARGS += [flag, str(value)]
    recorder.TRIAL = PRELUDE.format(upstream=str(args.upstream.resolve()),
                                   teacher_position=args.teacher_position,
                                   start_jitter_m=float(args.start_jitter_m),
                                   front_pose=front_pose) + recorder.TRIAL
    recorder.main(argv[split+1:])

if __name__ == '__main__':
    main()
