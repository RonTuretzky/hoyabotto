"""Independent local safety binding around the published continuous executor.

Only the sole hardware owner constructs this object. Inject registered immutable
camera readers, its health/STOP guard, motor-write callback and release/STOP
callback. Gripper release-generation binding remains an outer owner check: the
published trajectory-vision protocol does not carry that field.
"""
import hashlib
import time
from pathlib import Path

from .common import Refused, camera_names, finite, read_json
from .continuous import TrajectoryExecutor, trajectory, validate_profile


class ContinuousOwnerBinding:
    def __init__(self, profile_path, config_path, selected, *, cameras, guard,
                 write_goals, stop, clock=time.monotonic, wall=time.time, vision_path=None):
        self.clock,self.wall,self.guard=clock,wall,guard
        self.stop_callback,self.write_callback=stop,write_goals
        self.stopped=False;self.command_count=0;self.update_count=0
        self.check_latency_max_s=0.;self.first_tick=self.last_tick=None;self.last_error=None
        self.profile_path=Path(profile_path).resolve();self.config_path=Path(config_path).resolve()
        self.profile=read_json(self.profile_path);self.config=read_json(self.config_path)
        self.profile_file_sha=self._sha(self.profile_path);self.config_file_sha=self._sha(self.config_path)
        # All paths are resolved against the independently loaded station config.
        for field in ('calibration_file','session_dir'):
            self.config[field]=str(self._resolve(self.config[field]))
        for camera in self.config['cameras'].values():
            for field in ('reference','manifest'):
                camera[field]=str(self._resolve(camera[field]))
        self._calibration()
        validate_profile(self.profile,self.config)
        expected={f"{self.config['arm']}_arm_{name}" for name in
                  ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')}
        if len(selected)!=6 or set(selected)!=expected or set(self.profile['joints'])!=expected:
            raise Refused('Continuous binding requires exactly the selected arm')
        for name in expected:
            if not 0<finite(self.profile['velocity'][name])<=100 or not 0<finite(self.profile['acceleration'][name])<=200:
                raise Refused('Local continuous rate exceeds100velocity/200acceleration')
        self.cameras=dict(cameras)
        if set(self.cameras)!=camera_names(self.config) or set(self.config['cameras'])!=set(self.cameras):
            raise Refused('Both independently configured cameras are required')
        self.registered_streams={}
        for name,frame in self._frames().items():self.registered_streams[name]=frame.stream
        self.vision_path=self._resolve(vision_path) if vision_path is not None else Path(self.config['session_dir'])/'trajectory-vision.json'
        self.engine=TrajectoryExecutor(self.profile,self._write,self._stop,clock=clock,wall=wall)

    @staticmethod
    def _sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()

    def _resolve(self,value):
        path=Path(value)
        return path.resolve() if path.is_absolute() else (self.config_path.parent/path).resolve()

    def _calibration(self):
        if self._sha(self.config['calibration_file'])!=self.config.get('calibration_sha256'):
            raise Refused('Local motor calibration bytes changed')

    def _frames(self):
        frames={}
        for name,reader in self.cameras.items():
            identity=self.config['cameras'][name]['camera_id']
            if getattr(reader,'camera_id',None)!=identity:
                raise Refused('Injected camera reader registration differs from local config')
            frame=reader.read()
            if frame.camera_id!=identity or not isinstance(frame.stream,str) or not frame.stream:
                raise Refused('Camera identity missing or changed')
            if (type(frame.seq) is not int or frame.seq<0
                    or not 0<=self.wall()-finite(frame.stamp)<=self.profile['vision_age_s']):
                raise Refused('Registered camera manifest stale or invalid')
            frames[name]=frame
        if max(f.stamp for f in frames.values())-min(f.stamp for f in frames.values())>.3:
            raise Refused('Registered cameras are not synchronized')
        return frames

    def _vision(self,command):
        evidence=read_json(self.vision_path)
        if (evidence.get('command_id')!=command['id'] or evidence.get('session_started')!=command['session_started']
                or evidence.get('streams')!=self.registered_streams
                or command.get('camera_streams')!=self.registered_streams):
            raise Refused('Vision evidence is not bound to independently registered cameras/owner/command')
        stamps=evidence.get('captured_at',{});sequences=evidence.get('sequences',{})
        if set(stamps)!=set(self.cameras) or set(sequences)!=set(self.cameras):
            raise Refused('Vision evidence omits a registered camera')
        for name,frame in self._frames().items():
            seq=sequences[name];stamp=finite(stamps[name])
            if (frame.stream!=self.registered_streams[name] or type(seq) is not int or seq<0
                    or not 0<=frame.seq-seq<=5
                    or not 0<=frame.stamp-stamp<=self.profile['vision_age_s']
                    or frame.seq==seq and frame.stamp!=stamp):
                raise Refused('Vision evidence does not match current immutable camera manifests')
        return evidence

    def _telemetry(self,telemetry_at):
        if isinstance(telemetry_at,dict):
            if set(telemetry_at)!=set(self.profile['joints']):
                raise Refused('Per-motor telemetry capture times incomplete')
            stamp=min(finite(value) for value in telemetry_at.values())
        else:stamp=finite(telemetry_at)
        if not 0<=self.wall()-stamp<=self.profile['max_tick_gap_s']:
            raise Refused('Oldest motor telemetry capture is stale')
        return stamp

    def _stop(self):
        if not self.stopped:
            self.stopped=True
            if hasattr(self,'engine'):self.engine.active=False
            self.stop_callback()

    def _write(self,goals):
        self.guard()
        if self.stopped:raise Refused('Continuous binding already stopped')
        for name,q in goals.items():
            if type(q) is not int or abs(q-self.engine.previous_goal[name])>68:
                raise Refused('Continuous sampled increment exceeds68ticks')
        self.write_callback(goals)

    def capabilities(self):return self.engine.capabilities()

    @property
    def active(self):return self.engine.active

    def start(self,command,rows,telemetry_at,*,session_started,lease_remaining):
        begin=self.clock()
        try:
            self.guard()
            if self.stopped:raise Refused('Continuous binding already stopped')
            if self._sha(self.profile_path)!=self.profile_file_sha or self._sha(self.config_path)!=self.config_file_sha:
                raise Refused('Owner local profile/config changed')
            self._calibration();self._telemetry(telemetry_at)
            path=trajectory(self.profile,command['waypoints'])
            if path.duration+self.profile['settle_timeout_s']>30:
                raise Refused('Local trajectory plus settling exceeds30seconds')
            evidence=self._vision(command)
            result=self.engine.start(command,rows,evidence,session_started=session_started,lease_remaining=lease_remaining)
            self.command=dict(command);self.command_count+=1
            return result
        except BaseException as exc:
            self.last_error=str(exc);self._stop();raise
        finally:self.check_latency_max_s=max(self.check_latency_max_s,self.clock()-begin)

    def tick(self,rows,telemetry_at,*,stop_requested=False):
        begin=self.clock();self.update_count+=1
        if self.first_tick is None:self.first_tick=begin
        self.last_tick=begin
        try:
            self.guard()
            if self.stopped or stop_requested:raise Refused('Continuous owner STOP requested')
            return self.engine.tick(rows,self._vision(self.command),telemetry_at=self._telemetry(telemetry_at))
        except BaseException as exc:
            self.last_error=str(exc);self._stop();raise
        finally:self.check_latency_max_s=max(self.check_latency_max_s,self.clock()-begin)

    def metrics(self):
        elapsed=0 if self.first_tick is None else self.last_tick-self.first_tick
        return {**self.engine.metrics,'whole_trajectory_commands':self.command_count,
                'owner_update_count':self.update_count,'observed_loop_hz':(self.update_count-1)/elapsed if elapsed>0 else None,
                'check_latency_max_s':self.check_latency_max_s,'stopped':self.stopped,'last_error':self.last_error}
