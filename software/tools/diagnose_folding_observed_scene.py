"""Offline saved-state sensor replay -> PixelPort -> conservative collision scene.

No dynamics step, robot action, physical calibration, camera-accuracy claim or
fold-success evaluation. Only the explicitly supported local station layout
is admitted. Recorded object qpos is private to SensorReplay's renderer.
"""
from __future__ import annotations

import argparse
from contextlib import contextmanager
import copy
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import platform
import re
import sys
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np

from carton.folding_markers import BOX_MARKERS
from carton.folding_vision import SIZES
from carton.folding_hinge_tags import carton_pose_from_short_flaps  # Include deferred estimator source in manifest.
from carton.folding_paths import JointPathPlanner  # Include deferred planner source in manifest.
from carton.folding_observed_scene import (
    ENCODER_NAMES, FLAPS, FRAME_CONVENTION, MeasuredCarton, ObservedSceneBuilder,
    SceneCalibration, StationBox,
)
from carton.folding_observed_adapter import (
    OfflineObservationAssumptions, OfflinePixelSceneAdapter, OfflineSensorCapture,
    export_offline_robot_model,
)
from carton.folding_station import FoldingStation
from tools.simulate_bimanual_folding import PixelPort


PROFILE_NAME = 'synthetic-envelope-v1'
SETUP_NAME = 'offline:rear-cart-060-150-010-station-floor24-25-v1'
SUPPORTED_SETUP_SHA256 = '2cbe4b69b2c7c883d77fd43abc64d4e95c7123441d640045187f6ec64bbe87e8'
ROOTS = {'left': 'left_base_link', 'right': 'right_base_link'}
POLICIES = (('all_required', FLAPS, 'refuse'),
            ('majors_required_optional_sweeps', ('long_near', 'long_far'), 'swept'))
PROFILE = dict(name=PROFILE_NAME, physical_measurement=False, base_position_error_m=.0005,
    base_rotation_error_deg=.1, robot_geometry_error_m=.0002, station_point_error_m=.0005,
    carton_dimension_error_m=.0005, carton_position_error_m=.001, carton_rotation_error_deg=.1,
    flap_error_deg=.5, encoder_error_rad=.001, uncertainty_horizon_s=.1,
    width=1280, height=720, camera='station', depth_noise_std_m=.0008, depth_dropout_fraction=.25)
FLOOR_MARKERS = {24: ('box_tag_floor', [.08, 0, .0038], [1, 0, 0, 0, 1, 0]),
                 25: ('box_tag_floor_center', [0, 0, .0038], [1, 0, 0, 0, 1, 0])}


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False, separators=(',', ':')).encode()).hexdigest()


def jsonable(value):
    if isinstance(value, np.ndarray):return value.tolist()
    if isinstance(value, np.generic):return value.item()
    if isinstance(value, dict):return {str(key):jsonable(item) for key,item in value.items()}
    if isinstance(value, (tuple,list)):return [jsonable(item) for item in value]
    return value


def parse_frames(values):
    try:frames = [int(part) for value in values for part in value.split(',')]
    except ValueError as exc:raise ValueError('Frames must be comma- or space-separated nonnegative integers') from exc
    if not frames or any(index < 0 for index in frames) or frames != sorted(set(frames)):
        raise ValueError('Frames must be unique nonnegative indices in ascending order')
    return frames


def setup_declaration(scene_xml):
    """Geometric setup fingerprint; no movable object's world state is used.

    Includes static obstacle layout, marker patterns/mounts, carton-local
    geometry, robot-base transforms and station camera. The carton's initial
    world pose, joint rest positions, forces and material parameters are
    excluded. A matching fingerprint is an offline allowlist, not calibration.
    """
    root = ET.fromstring(scene_xml)
    if (root.tag != 'mujoco' or root.findall('.//include') or root.find('extension') is not None
            or root.find('deformable') is not None or root.find('equality') is not None):
        raise ValueError('Supported flattened rigid scene without extensions/equalities required')
    world = root.find('worldbody')
    if world is None:raise ValueError('Scene worldbody missing')
    by_name = {body.get('name'):body for body in world.findall('body')}
    expected = {*ROOTS.values(), 'carton', 'table_tag', 'table_tag_backup'}
    if set(by_name) != expected or len(world.findall('body')) != len(expected):
        raise ValueError('Unsupported scene roots or unmodelled movable obstacles')
    numeric = {'pos','quat','euler','axisangle','xyaxes','zaxis','size','axis','range','rgba',
               'contype','conaffinity','group','fovy'}
    def canonical(element, *, omit_world_pose=False):
        attrs = {}
        for key,value in element.attrib.items():
            if omit_world_pose and key in ('pos','quat','euler','axisangle','xyaxes','zaxis'):continue
            if key in numeric:
                numbers = [float(item) for item in value.split()]
                if not np.isfinite(numbers).all():raise ValueError('Nonfinite source geometry')
                attrs[key] = numbers
            elif key in ('name','type','mesh','material','class','childclass','angle'):attrs[key] = value
        return [element.tag,attrs,[canonical(child) for child in element if child.tag != 'inertial']]
    camera = world.findall("camera[@name='station']")
    if len(camera) != 1:raise ValueError('Declared world-mounted station camera missing')
    robots = {}
    for side,name in ROOTS.items():
        robot = by_name[name]
        housing = robot.findall(f".//body[@name='{side}_tag']")
        if len(housing) != 1:raise ValueError('Declared encoder-FK housing marker missing')
        robots[side] = dict(base=canonical(ET.Element('body',robot.attrib)), housing=canonical(housing[0]),
                           joints=[canonical(joint) for joint in robot.findall('.//joint')])
    return dict(setup=SETUP_NAME, compiler_angle=root.find('compiler').get('angle') if root.find('compiler') is not None else None,
        world_children=[child.tag for child in world if child.tag not in ('body','geom','camera','light')],
        defaults=[canonical(item) for item in root.findall('default')], robots=robots,
        station_geoms=[canonical(item) for item in world.findall('geom')], camera=canonical(camera[0]),
        anchors={name:canonical(by_name[name]) for name in ('table_tag','table_tag_backup')},
        carton_local_geometry=canonical(by_name['carton'],omit_world_pose=True))


def frozen_assets(scene_xml, asset_root):
    """Freeze file-backed render assets and resolve them independently of cwd."""
    root = ET.fromstring(scene_xml); compiler = root.find('compiler')
    if compiler is None or compiler.get('angle') != 'radian':raise ValueError('Explicit radian source model required')
    manifest, blobs = {}, {}
    for asset in root.findall('./asset/*'):
        if asset.tag not in ('mesh','texture','material'):raise ValueError('Unsupported renderer asset type')
        if asset.get('file') is None:continue
        directory = compiler.get('meshdir' if asset.tag == 'mesh' else 'texturedir','')
        path = (Path(asset_root)/directory/asset.get('file')).resolve()
        content = path.read_bytes(); sha = hashlib.sha256(content).hexdigest()
        name = sha[:16] + '-' + path.name
        manifest[str(path)] = sha; blobs[name] = content; asset.set('file',name)
    compiler.attrib.pop('meshdir',None); compiler.attrib.pop('texturedir',None)
    return ET.tostring(root,encoding='unicode'), blobs, manifest


@contextmanager
def declared_marker_registry():
    """Use exactly the supported offline marker layout; restore caller globals."""
    saved_markers, saved_sizes = copy.deepcopy(BOX_MARKERS), copy.deepcopy(SIZES)
    try:
        if set(BOX_MARKERS) - {10,21,22,24,25}:raise ValueError('Unsupported pre-existing marker registry')
        BOX_MARKERS.update(copy.deepcopy(FLOOR_MARKERS))
        SIZES.update({24:.045,25:.045})
        yield
    finally:
        BOX_MARKERS.clear();BOX_MARKERS.update(saved_markers)
        SIZES.clear();SIZES.update(saved_sizes)


def declared_setup(scene_xml, run_dir, run_id, profile):
    declaration = setup_declaration(scene_xml)
    if digest(declaration) != SUPPORTED_SETUP_SHA256:
        raise ValueError('Unsupported source setup fingerprint; no station calibration inferred')
    robot = export_offline_robot_model(scene_xml,root_body_names=ROOTS,asset_root=str(run_dir),
        model_id='offline:source-so101',measurement_id='offline:source-cad-not-physical',
        geometry_error_m=profile['robot_geometry_error_m'])
    station = FoldingStation(.06,.15,.01,table_marker_xy=(-.5,.55),backup_table_marker_xy=(.45,.70))
    base_poses = {}
    for side,sign in (('left',-1),('right',1)):
        pose=np.eye(4);pose[:3,:3]=[[0,-1,0],[1,0,0],[0,0,1]]
        pose[:3,3]=[sign*.15,station.base_y,.06];base_poses[side]=pose
    obstacles = []
    for geom in ET.fromstring(scene_xml).findall('./worldbody/geom'):
        pose=np.eye(4);pose[:3,3]=np.fromstring(geom.get('pos','0 0 0'),sep=' ')
        if any(geom.get(key) is not None for key in ('quat','euler','axisangle','xyaxes','zaxis')):
            raise ValueError('Supported static station obstacles must be axis-aligned')
        size=np.fromstring(geom.get('size',''),sep=' ')
        if geom.get('type')=='box' and len(size)==3:half=tuple(size)
        elif geom.get('type')=='sphere' and len(size)==1:half=(size[0],)*3
        else:raise ValueError('Unsupported static station primitive')
        obstacles.append(StationBox(re.sub('[^A-Za-z0-9_]','_',geom.get('name','')),pose,half,profile['station_point_error_m']))
    calibration=SceneCalibration(calibration_id=SETUP_NAME,measurement_id='offline:allowlisted-source-geometry',
        measurements_verified=True,world_frame_id='offline:declared-tabletop',clock_id='offline:saved-simulation-seconds',
        sensor_id='offline:station-renderer',robot_id='offline:source-so101',carton_id='offline:declared-empty-carton',
        inventory_id='offline:30-static-obstacles-plus-empty-carton',world_from_robot_bases=base_poses,
        base_position_error_m=profile['base_position_error_m'],base_rotation_error_deg=profile['base_rotation_error_deg'],
        anchor_ids=(1,20),station_boxes=tuple(obstacles),
        carton=MeasuredCarton(length_m=.379,width_m=.283,height_m=.108,flap_m=.14,thickness_m=.003,
            major_hinge_offset_m=.0035,dimension_error_m=profile['carton_dimension_error_m'],
            hinge_limits_deg={name:tuple(np.degrees([-1.7,3.05])) for name in FLAPS},
            flap_tag_ids=dict(zip(FLAPS,(11,12,13,14))),frame_convention=FRAME_CONVENTION,contents_height_m=None))
    assumptions=OfflineObservationAssumptions(run_id=run_id,assumption_id='offline:'+profile['name'],camera_name='station',
        carton_position_error_m=profile['carton_position_error_m'],carton_rotation_error_deg=profile['carton_rotation_error_deg'],
        flap_error_bounds_deg=dict.fromkeys(FLAPS,profile['flap_error_deg']),
        encoder_error_bounds_rad=dict.fromkeys(ENCODER_NAMES,profile['encoder_error_rad']),
        uncertainty_horizon_s=profile['uncertainty_horizon_s'],allow_paired_short_hinge_registration=True)
    return robot,station,calibration,assumptions,declaration


class SensorReplay:
    """Only this class restores recorded full qpos, exclusively for rendering.

    PixelPort sees a clock and declared camera intrinsics, not render MjData.
    Its housing-marker check uses a separate robot-only FK model populated
    with the twelve named robot encoders. No dynamics is advanced.
    """
    def __init__(self, render_model, encoder_model, station, profile):
        self._render_model=render_model;self._render_data=mujoco.MjData(render_model)
        self._encoder_model=encoder_model;self._encoder_data=mujoco.MjData(encoder_model)
        self._renderer=None;self._profile=profile;self._encoders={};self.images={}
        self._option=mujoco.MjvOption();self._option.geomgroup[3]=0
        self.station=station;self.data=SimpleNamespace(time=0.)
        self.model=SimpleNamespace(camera=self._camera)

    def _camera(self,name):
        if name!='station':raise ValueError('Only declared station intrinsics supported')
        return SimpleNamespace(fovy=np.array([48.]))

    def load(self, frame):
        self.images={}
        if not isinstance(frame,dict):raise ValueError('Recorded frame must be an object')
        state=np.asarray(frame.get('qpos'),dtype=float)
        stamp=frame.get('time')
        if (state.shape!=(self._render_model.nq,) or not np.isfinite(state).all()
                or isinstance(stamp,bool) or not isinstance(stamp,(int,float)) or not math.isfinite(stamp) or stamp<0):
            raise ValueError('Finite complete recorded render qpos and simulation time required')
        self._render_data.qpos[:]=state;self._render_data.time=stamp
        mujoco.mj_kinematics(self._render_model,self._render_data)
        mujoco.mj_comPos(self._render_model,self._render_data)
        mujoco.mj_camlight(self._render_model,self._render_data)
        self.data.time=float(stamp);self.images={}
        self._encoders={name:float(state[self._render_model.joint(name).qposadr[0]]) for name in ENCODER_NAMES}
        for name,value in self._encoders.items():self._encoder_data.qpos[self._encoder_model.joint(name).qposadr[0]]=value
        mujoco.mj_kinematics(self._encoder_model,self._encoder_data)

    def named_encoders(self):return self._encoders.copy()

    def arm_tag_fk(self,side):
        if side not in ROOTS:raise ValueError('Named robot arm required')
        pose=np.eye(4)
        pose[:3,:3]=self._encoder_data.body(side+'_tag').xmat.reshape(3,3)@np.diag([-1,1,-1])
        pose[:3,3]=self._encoder_data.site(side+'_tag_center').xpos
        return pose

    def render(self,camera='station',depth=False):
        self._camera(camera)
        if self._renderer is None:
            self._renderer=mujoco.Renderer(self._render_model,height=self._profile['height'],width=self._profile['width'])
        self._renderer.update_scene(self._render_data,camera=camera,scene_option=self._option)
        if depth:self._renderer.enable_depth_rendering()
        else:self._renderer.disable_depth_rendering()
        result=self._renderer.render().copy()
        # PixelPort adds depth noise/dropout to this same returned array. Its
        # saved copy after observe() is therefore the actual exposed input.
        self.images['depth' if depth else 'rgb']=result
        return result

    def close(self):
        if self._renderer is not None:self._renderer.close()


def source_snapshot(repo, output):
    paths={Path(__file__).resolve()}
    for module in tuple(sys.modules.values()):
        file=getattr(module,'__file__',None)
        if not file:continue
        path=Path(file).resolve()
        # Some extension modules (torch.classes) report a bare relative file name.
        if path.suffix=='.py' and path.is_file() and path.is_relative_to(repo):paths.add(path)
    manifest={}
    for path in sorted(paths):
        content=path.read_bytes();relative=path.relative_to(repo)
        target=output/'sources'/relative;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(content)
        manifest[str(relative)]=hashlib.sha256(content).hexdigest()
    return manifest


def evaluate_reading(reading, history, capture, robot, calibration, assumptions):
    outcomes={}
    for label,required,unknown in POLICIES:
        row={'required_flaps':list(required),'unknown_flaps':unknown,'status':'scene_refused','scene_built':False}
        try:
            adapter=OfflinePixelSceneAdapter(ObservedSceneBuilder(robot,calibration),assumptions)
            packets=adapter.adapt(reading,history,capture,inventory_complete=True,unmodelled_obstacles=[])
            row['packets']=jsonable(asdict(packets))
            result=adapter.build(reading,history,capture,now_s=capture.rgb_timestamp_s+.001,
                required_flaps=required,unknown_flaps=unknown,inventory_complete=True,unmodelled_obstacles=[])
            scene=result.scene;row.update(scene_built=True,status='configuration_refused',scene_metadata=scene.metadata)
            checks={}
            for side in ROOTS:
                planner=scene.planner(side,now_s=capture.rgb_timestamp_s+.001,
                    clock_id=calibration.clock_id,calibration_id=calibration.calibration_id)
                clear=planner.valid(scene.data.qpos[scene.arm_indices[side][:5]])
                checks[side]={'clear':bool(clear),'collision':planner.last_collision}
            row['configuration_checks']=checks
            row['status']='configuration_clear' if all(item['clear'] for item in checks.values()) else 'configuration_refused'
        except (ValueError,TypeError) as exc:row['refusal']=str(exc)
        outcomes[label]=row
    return outcomes


def run(args):
    run_dir=Path(args.run).resolve();output=Path(args.out).resolve()
    indices=parse_frames(args.frames)
    if args.uncertainty_profile!=PROFILE_NAME:raise ValueError('Explicit supported synthetic uncertainty profile required')
    output.mkdir(parents=True,exist_ok=False)
    repo=Path(__file__).resolve().parents[1]
    sources=source_snapshot(repo,output)
    profile=copy.deepcopy(PROFILE)
    report=dict(status='setup_refused',component_test_only=True,simulation_only=True,hardware_commands=False,
        physical_calibration_verified=False,camera_accuracy_measured=False,fold_success_assessed=False,
        source_run=str(run_dir),requested_frames=indices,seed=args.seed,uncertainty_profile=profile,
        uncertainty_profile_sha256=digest(profile),executed_sources_sha256=sources,
        executed_source_manifest_sha256=digest(sources),cases=[],
        asset_files_sha256=None,asset_manifest_sha256=None,assumptions_sha256=None,
        versions=dict(python=platform.python_version(),mujoco=mujoco.__version__,numpy=np.__version__,opencv=cv2.__version__))
    replay=None
    try:
        inputs={}
        for name in ('scene.xml','folding-frames.json'):
            content=(run_dir/name).read_bytes();inputs[name]=content
            target=output/'inputs'/name;target.parent.mkdir(exist_ok=True);target.write_bytes(content)
        report['input_files_sha256']={name:hashlib.sha256(content).hexdigest() for name,content in inputs.items()}
        archived={}
        for path in sorted((run_dir/'sources').glob('*.py')):
            content=path.read_bytes();archived[path.name]=hashlib.sha256(content).hexdigest()
            target=output/'archived_sources'/path.name;target.parent.mkdir(exist_ok=True);target.write_bytes(content)
        report['archived_sources_sha256']=archived
        report['archived_source_manifest_sha256']=digest(archived)
        scene_xml=inputs['scene.xml'].decode();frames=json.loads(inputs['folding-frames.json'])
        if not isinstance(frames,list) or indices[-1]>=len(frames):raise ValueError('Requested frame outside saved recording')
        setup=setup_declaration(scene_xml);report['observed_setup_sha256']=digest(setup)
        report['supported_setup_sha256']=SUPPORTED_SETUP_SHA256
        run_id='offline:saved-run-'+report['input_files_sha256']['folding-frames.json'][:16]
        robot,station,calibration,assumptions,_=declared_setup(scene_xml,run_dir,run_id,profile)
        render_xml,render_assets,assets=frozen_assets(scene_xml,run_dir)
        robot_xml,robot_assets,robot_manifest=frozen_assets(robot.xml,run_dir)
        if assets!=robot_manifest or robot_manifest!=robot.asset_sha256:raise ValueError('Asset bytes changed during setup')
        report.update(asset_files_sha256=assets,asset_manifest_sha256=digest(assets),
            robot_template_sha256=robot.sha256,static_station_obstacle_count=len(calibration.station_boxes),
            calibration_declaration=jsonable(asdict(calibration)),observation_assumptions=jsonable(asdict(assumptions)))
        report['assumptions_sha256']=digest(dict(profile=profile,calibration=report['calibration_declaration'],
                                                observation=report['observation_assumptions']))
        (output/'robot-only.xml').write_text(robot.xml)
        render_model=mujoco.MjModel.from_xml_string(render_xml,assets=render_assets)
        encoder_model=mujoco.MjModel.from_xml_string(robot_xml,assets=robot_assets)
        replay=SensorReplay(render_model,encoder_model,station,profile)
        previous=-1.
        with declared_marker_registry():
            port=PixelPort(replay,record=False,camera='station',seed=args.seed,
                           noise=profile['depth_noise_std_m'],dropout=profile['depth_dropout_fraction'])
            for index in indices:
                row={'frame_index':index,'status':'sensor_refused'}
                try:
                    replay.load(frames[index]);stamp=replay.data.time
                    if stamp<=previous:raise ValueError('Selected render capture times must strictly increase')
                    previous=stamp
                    reading=port.observe(f'Offline sensor replay frame {index}')
                    history=copy.deepcopy(port.observer.history[-1])
                    capture=OfflineSensorCapture(run_id=run_id,clock_id=calibration.clock_id,observation_seq=reading['seq'],
                        rgb_timestamp_s=stamp,depth_timestamp_s=stamp,world_from_camera=port.observer.world_from_camera.copy(),
                        encoder_seq=reading['seq'],encoder_timestamp_s=stamp,joint_positions=replay.named_encoders())
                    row.update(status='observed',pixelport_reading=reading,observer_history=history,
                        captured_camera_and_encoders=jsonable(asdict(capture)),
                        outcomes=evaluate_reading(reading,history,capture,robot,calibration,assumptions))
                except (ValueError,TypeError) as exc:row['refusal']=str(exc)
                row['sensor_files']={}
                for kind,value in replay.images.items():
                    name=f'frame-{index:06d}-{kind}.npy';np.save(output/name,value,allow_pickle=False)
                    row['sensor_files'][kind]=dict(path=name,sha256=hashlib.sha256((output/name).read_bytes()).hexdigest(),
                                                   shape=list(value.shape),dtype=str(value.dtype))
                report['cases'].append(row)
                if row['status']=='sensor_refused':
                    # A failed first-frame registration must not let the next
                    # observation bypass PixelPort's initial housing-tag check.
                    port=PixelPort(replay,record=False,camera='station',seed=args.seed,
                                   noise=profile['depth_noise_std_m'],dropout=profile['depth_dropout_fraction'])
        report['status']='completed_offline_component'
    except (ValueError,TypeError,OSError,ET.ParseError,UnicodeError,json.JSONDecodeError) as exc:
        report['refusal']=str(exc)
    finally:
        if replay is not None:replay.close()
        changed=[name for name,sha in sources.items() if not (repo/name).exists() or hashlib.sha256((repo/name).read_bytes()).hexdigest()!=sha]
        report['executed_source_changed_during_run']=changed
        if changed:report.update(status='source_changed_during_run',refusal='Source snapshots preserved; current files changed during execution')
        asset_changes=[name for name,sha in (report['asset_files_sha256'] or {}).items()
                       if not Path(name).exists() or hashlib.sha256(Path(name).read_bytes()).hexdigest()!=sha]
        report['asset_changed_during_run']=asset_changes
        if asset_changes:report.update(status='assets_changed_during_run',refusal='Recorded asset identities changed during execution')
        (output/'result.json').write_text(json.dumps(jsonable(report),indent=2,allow_nan=False)+'\n')
    print(json.dumps({'out':str(output),'status':report['status'],'cases':[
        {'frame_index':row['frame_index'],'status':row['status'],
         'outcomes':{name:value['status'] for name,value in row.get('outcomes',{}).items()},
         'refusal':row.get('refusal')} for row in report['cases']]},indent=2))
    return report


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run',required=True,help='Saved offline folder containing scene.xml and folding-frames.json')
    parser.add_argument('--frames',required=True,nargs='+',help='Ascending frame indices, separated by spaces or commas')
    parser.add_argument('--out',required=True,help='Fresh output directory; existing evidence is never overwritten')
    parser.add_argument('--uncertainty-profile',required=True,choices=[PROFILE_NAME],help='Named synthetic assumptions; never physical measurement')
    parser.add_argument('--seed',type=int,default=0)
    args=parser.parse_args(argv)
    try:report=run(args)
    except (ValueError,OSError) as exc:parser.error(str(exc))
    return 0 if report['status']=='completed_offline_component' else 2


if __name__=='__main__':raise SystemExit(main())
