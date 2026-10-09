#!/usr/bin/env python3
"""Local operator UI + native Joy-Con input -> existing pinned robot mTLS API.

Starts DISARMED. No automatic arm/retry/reconnect. No serial access or model calls.
"""
import argparse, json, os, secrets, signal, ssl, subprocess, threading, time, urllib.request, urllib.parse, socket, sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from mapping import Mapping
class PreviewRobot:
    preview = True
    simulation = False
    def call(self, path, body=None, timeout=None):
        if path == "status": return {"preview": True, "phase": "preview", "motors": {}, "message": "Local preview: no robot connection"}
        raise ValueError("Local preview cannot activate or command the robot")

DEFAULT_CONFIG = '/Users/wk/Documents/ChatGPT/Hackatuson/output/gemma-xlerobot/pilot/.private/robot.json'
MOTOR_CONTROL_LOCK = Path(__file__).resolve().parents[1]/'MOTOR_CONTROL_DISABLED'

class Robot:
    preview = False
    simulation = False
    def __init__(self, config):
        c=json.loads(Path(config).read_text())
        self.context=ssl.create_default_context(cafile=c['server_certificate'])
        self.context.load_cert_chain(c['client_certificate'],c['client_key'])
        self.url=c['url'].rstrip('/')
        if c.get('lan_url'):
            u=urllib.parse.urlsplit(c['lan_url'])
            try:
                with socket.create_connection((u.hostname,u.port or 443),timeout=.8):pass
                self.url=c['lan_url'].rstrip('/');self.context.check_hostname=False
            except OSError:pass
        # LAN bypasses hostname only, exactly like robot_admin; pinned CA and client identity remain verified.
    def call(self,path,body=None,timeout=.4):
        req=urllib.request.Request(self.url+'/teleop/'+path,data=None if body is None else json.dumps(body,allow_nan=False).encode(),headers={'Content-Type':'application/json'})
        try:
            with urllib.request.urlopen(req,context=self.context,timeout=timeout) as r:out=json.load(r)
        except urllib.error.HTTPError as e:
            try:message=json.load(e).get('error','Robot rejected request')
            except Exception:message='Robot rejected request'
            raise ValueError(message) from None
        if not out.get('ok'):raise ValueError(out.get('error','Robot request failed'))
        return out['result']

class Bridge:
    def __init__(self,robot,reader,*,start=True,mapping=None,input_backend='apple',preview3d=None,practice_factory=None):
        self.robot=robot;self.reader=reader;self.mapping=mapping or Mapping();self.lock=threading.RLock()
        self.input_backend=input_backend;self.preview3d=preview3d
        self.live_robot=robot;self.live_mapping=self.mapping;self.practice_factory=practice_factory
        self.practice_robot=None;self.practice_mapping=None;self.control_target="robot"
        self.frame=None;self.decoded=None;self.reader_error='Waiting for controller input'
        self.generation=0;self.armed=False;self.busy=False;self.scope='left';self.session=None;self.identity=None
        self.reason='Disarmed — test both triggers, then release them and center the sticks'
        if getattr(self.mapping,'mode',None)=='cartesian':self.reason='Disarmed — test upper L/R buttons, release all buttons and center the sticks'
        if getattr(self.mapping,'mode',None)=='upstream' and getattr(robot,'supports_upstream',False):self.reason='Disarmed — check SL or SR on each Joy-Con, release all controls and center sticks'
        self.robot_state={};self.ui_seen=0;self.sequence=0;self.rtt=None;self.closing=False
        self.release_pending=0;self.worker=None;self.reader_thread=None;self.proc=None
        if start:self.start()
    def start(self):
        self.start_reader()
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
    def start_reader(self):
        command=([sys.executable,str(Path(__file__).with_name('hid_reader.py'))] if self.input_backend=='hid' else [str(self.reader)])
        self.proc=subprocess.Popen(command+['--json','--hz',str(getattr(self.mapping,'reader_hz',30)),'--deadzone','0.12'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,bufsize=1)
        self.reader_thread=threading.Thread(target=self.read_frames,daemon=True);self.reader_thread.start()
    def stop_reader(self):
        if self.proc:
            self.proc.terminate()
            try:self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait(timeout=3)
        if self.reader_thread:self.reader_thread.join(timeout=3)
        self.proc=None;self.reader_thread=None
    def receive(self, frame):
        with self.lock:
            self.frame=frame
            try:self.decoded=self.mapping.decode(frame);self.reader_error=None
            except (ValueError,KeyError,TypeError,AttributeError) as e:
                self.decoded=None;self.reader_error=str(e)
                if frame.get('diagnostics'):self.reader_error+=' '+'; '.join(str(k).title()+': '+str(v) for k,v in frame['diagnostics'].items())
    def read_frames(self):
        try:
            for line in self.proc.stdout:
                try:self.receive(json.loads(line))
                except (ValueError,TypeError):
                    with self.lock:self.frame=None;self.decoded=None;self.reader_error='Invalid reader JSON'
        finally:
            with self.lock:self.frame=None;self.decoded=None;self.reader_error='Native reader stopped'
    def valid(self):
        if self.closing:raise ValueError('Teleop is closing')
        if not self.frame or not self.decoded:raise ValueError(self.reader_error or 'Controller unavailable')
        if not 0<=time.time()-self.frame['timestamp']<=.2:raise ValueError('Controller input timed out')
        if self.decoded['stop']:raise ValueError('Minus / Options STOP')
        if not getattr(self.robot,'allow_background_input',False) and time.monotonic()-self.ui_seen>.8:raise ValueError('Operator screen lost focus or stopped responding')
        return self.decoded
    def snapshot(self):
        with self.lock:
            d=self.decoded or {}
            state=dict(self.robot_state)
            if self.preview3d and not self.robot.simulation:state['simulator']=self.preview3d.info()
            return dict(background_practice=getattr(self.robot,'allow_background_input',False),practice_available=self.practice_factory is not None,control_target=self.control_target,preview=self.robot.preview,simulation=self.robot.simulation,armed=self.armed,busy=self.busy or bool(self.release_pending),scope=self.scope,layer=self.mapping.layer,reason=self.reason,
                        control_mode=getattr(self.mapping,'mode','joint'),input_backend=self.input_backend,control_info=getattr(self.mapping,'info',{})|{'gyro_enabled':getattr(self.mapping,'gyro_enabled',False)},reference_frame=getattr(self.mapping,'reference_frame','robot'),
                        controller=d,reader_error=self.reader_error,robot=state,rtt_ms=self.rtt,
                        checked_triggers=sorted(self.mapping.checked),input_age_s=round(time.time()-self.frame['timestamp'],3) if self.frame else None)
    def detach(self, reason):
        # Cancel synchronously before scheduling I/O. An old response must never
        # arm a session or release a new session that began in another thread.
        with self.lock:
            self.generation+=1
            session=self.session;self.session=None;self.armed=False;self.reason=reason
            self.release_pending+=int(session is not None)
            return session,self.generation
    def finish_release(self, session, generation, reason):
        if session is None:return
        try:
            result=self.robot.call('release',{'token':session['token']},timeout=6)
            suffix=' — release confirmed' if result.get('released') else ' — release unconfirmed'
        except Exception:
            suffix=' — release unconfirmed; checking owner status'
        finally:
            with self.lock:self.release_pending-=1
        with self.lock:
            if self.generation==generation:self.reason=reason+suffix
    def release(self, reason):
        session,generation=self.detach(reason)
        self.finish_release(session,generation,reason)
    def action(self,b):
        op=b.get('op')
        if op=='control_target':
            target=b.get('target')
            with self.lock:
                if not self.practice_factory or target not in ('robot','practice'):raise ValueError('Unknown control target')
                if self.armed or self.busy or self.release_pending:raise ValueError('Stop controls before changing target')
                self.busy=True;self.generation+=1
            try:
                if target=='practice' and self.practice_robot is None:
                    self.practice_robot=self.practice_factory();self.practice_mapping=self.practice_robot.make_mapping()
                with self.lock:
                    self.robot=self.practice_robot if target=='practice' else self.live_robot
                    self.mapping=self.practice_mapping if target=='practice' else self.live_mapping
                    self.mapping.reset();self.decoded=None;self.robot_state={};self.control_target=target
                    self.reason='Practice stopped — Start practice to animate; no physical robot commands' if target=='practice' else 'Disarmed — live robot requires rail hold-to-run'
            finally:
                with self.lock:self.busy=False
            return
        if op=='input_backend':
            with self.lock:
                if not self.robot.simulation:raise ValueError('Reader selection is simulation-only')
                if self.armed or self.busy or self.release_pending:raise ValueError('Stop practice before switching input')
                if b.get('backend') not in ('apple','hid'):raise ValueError('Unknown reader')
                if getattr(self.mapping,'mode',None)=='upstream' and b['backend']!='hid':raise ValueError('Original controls require the Mac HID reader')
                self.busy=True
            try:
                self.stop_reader()
                with self.lock:
                    self.input_backend=b['backend'];self.frame=None;self.decoded=None
                    self.mapping.checked.clear()
                    if hasattr(self.mapping,'checked_bumpers'):self.mapping.checked_bumpers.clear()
                    if hasattr(self.mapping,'reset'):self.mapping.reset();self.mapping.gyro_enabled=False
                    self.reason='Reader changed — check L and R again'
                self.start_reader()
            except OSError as e:raise ValueError('Reader could not start: '+str(e)) from e
            finally:
                with self.lock:self.busy=False
            return
        if op=='stop':
            reason='Operator STOP';session,generation=self.detach(reason)
            threading.Thread(target=self.finish_release,args=(session,generation,reason),daemon=True).start();return
        with self.lock:
            if self.busy or self.release_pending:raise ValueError('Wait for the current operation')
            if op=='reference_frame':
                if self.armed:raise ValueError('Stop practice before changing the movement frame')
                if getattr(self.mapping,'mode',None)!='cartesian' or b.get('frame') not in ('robot','hand'):raise ValueError('Unknown hand-space frame')
                self.mapping.reference_frame=b['frame'];self.mapping.reset();return
            if op=='gyro':
                if self.armed:raise ValueError('Stop practice before changing gyro mode')
                if getattr(self.mapping,'mode',None) not in ('cartesian','upstream'):raise ValueError('Hand-space mode required')
                if b.get('enabled') is True and not self.valid().get('gyro_available'):raise ValueError('Both independent calibrated gyro streams required')
                self.mapping.gyro_enabled=b.get('enabled') is True;self.mapping.reset();return
            if op=='layer':
                d=self.valid()
                if not d['neutral']:raise ValueError('Release triggers and center sticks before changing layers')
                if type(b.get('layer')) is not int or not 0<=b['layer']<=2:raise ValueError('Invalid layer')
                self.mapping.layer=b['layer'];return
            if op not in ('arm','practice'):raise ValueError('Unknown operation')
            if op=='arm' and self.robot.preview:raise ValueError('Local preview cannot activate motors')
            if op=='practice' and not self.robot.simulation:raise ValueError('Practice requires the local simulator')
            d=self.valid()
            if self.armed:raise ValueError('Already armed')
            if not d['ready'] or not d['neutral']:
                raise ValueError('Set both Joy-Cons down to calibrate, release all buttons, and center sticks' if getattr(self.mapping,'mode',None)=='upstream' else 'Test both hold-to-run buttons, release all buttons, and center sticks before arming')
            scope=b.get('scope')
            if scope not in ('left','right','both','head','drive','wholebody'):raise ValueError('Unknown scope')
            if scope=='wholebody' and (not (self.robot.simulation or getattr(self.robot,'supports_upstream',False)) or getattr(self.mapping,'mode',None) not in ('cartesian','upstream')):raise ValueError('Whole-body control requires local hand-space simulation')
            if hasattr(self.mapping,'reset'):self.mapping.reset()
            if hasattr(self.mapping,'prepare_start'):self.mapping.prepare_start(d)
            self.busy=True;generation=self.generation;self.scope=scope;self.identity=d['identity'];self.reason='Starting local practice…' if self.robot.simulation else 'Arming at measured positions…'
        try:
            session=self.robot.call('claim',{'scope':scope},timeout=6)
            with self.lock:
                self.session=session;self.sequence=0
                if generation!=self.generation:raise ValueError('STOP cancelled arm')
                d=self.valid()
                if d['identity']!=self.identity or not d['neutral']:raise ValueError('Controls changed while arming')
                # Prove neutral synchronously; the background loop cannot replace
                # this packet before the owner's initial neutral gate is satisfied.
                self.sequence=1
                neutral=self.mapping.command(d,scope) | {k:session[k] for k in ('token','permit','owner_started')} | {'sequence':1}
            result=self.robot.call('input',neutral)
            with self.lock:
                if generation!=self.generation:raise ValueError('STOP cancelled arm')
                self.valid()
                self.session=result;self.robot_state=result.get('status',self.robot_state)
                self.mapping.update_robot_status(self.robot_state);self.armed=True
                self.reason='PRACTICE — simulated components only' if self.robot.simulation else ('Armed — hold the side-rail SL or SR buttons to move' if getattr(self.mapping,'mode',None)=='upstream' else 'Armed — hold trigger to move')
        except Exception as e:
            self.release('Start failed: '+str(e));raise
        finally:
            with self.lock:self.busy=False
    def step(self):
        started=time.monotonic()
        with self.lock:
            if not self.armed:return
            session=dict(self.session);generation=self.generation
        try:
            with self.lock:
                d=self.valid()
                if d['identity']!=self.identity:raise ValueError('Controller reconnected; re-arm required')
                if generation!=self.generation:return
                self.sequence+=1
                self.mapping.update_robot_status(self.robot_state)
                body=self.mapping.command(d,self.scope) | {k:session[k] for k in ('token','permit','owner_started')} | {'sequence':self.sequence}
            result=self.robot.call('input',body)
            with self.lock:
                if self.armed and self.generation==generation:
                    self.session=result;self.robot_state=result.get('status',self.robot_state)
                    if self.preview3d and not self.robot.simulation:self.preview3d.update(self.robot_state)
                    self.rtt=round((time.monotonic()-started)*1000,1)
        except Exception as e:
            with self.lock:
                if self.generation!=generation:return
            self.release(str(e)+'; start again to continue')
    def run(self):
        last_status=0
        while not self.closing:
            started=time.monotonic()
            if self.armed:self.step()
            elif not self.busy and not self.release_pending and time.monotonic()-last_status>1:
                try:
                    with self.lock:robot=self.robot;generation=self.generation
                    state=robot.call('status',timeout=2)
                    with self.lock:
                        if robot is self.robot and generation==self.generation:self.robot_state=state
                    if self.preview3d and robot is self.robot and not robot.simulation:self.preview3d.update(state)
                except Exception as e:
                    with self.lock:self.robot_state={'error':str(e)}
                last_status=time.monotonic()
            time.sleep(max(.001,getattr(self.mapping,'interval',.06)-(time.monotonic()-started)))
    def close(self):
        self.closing=True;self.release('Teleop closed')
        self.stop_reader()
        if self.worker:self.worker.join(timeout=3)
        if self.preview3d:self.preview3d.close()
        if self.practice_robot:self.practice_robot.close()
        if hasattr(self.live_robot,'close'):self.live_robot.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    ap.add_argument('--simulator',choices=('registers','mujoco'),default='registers',help='Local simulator backend')
    ap.add_argument('--model',type=Path,help='Optional XLeRobot MuJoCo model XML')
    ap.add_argument('--control-mode',choices=('joint','cartesian','upstream'),default='joint')
    ap.add_argument('--upstream-reference',type=Path,help='Original native-unit calibration binding (or explicit geometric reference)')
    ap.add_argument('--input-backend',choices=('apple','hid','auto'),default='apple')
    ap.add_argument('--connect-robot',action='store_true',help='Explicitly connect to the robot. Default is a local input preview with no network access.')
    ap.add_argument('--config',default=os.environ.get('XLEROBOT_ADMIN_CONFIG',DEFAULT_CONFIG))
    ap.add_argument('--reader',type=Path,default=Path(__file__).resolve().parents[1]/'.build/release/MacJoyConReader')
    ap.add_argument('--start-in-practice',action='store_true',help='Start the physical-capable screen in virtual practice; never arms the robot')
    ap.add_argument('--readback-preview',action='store_true',help='Read-only MuJoCo model view of physical encoder feedback')
    ap.add_argument('--ui-session-file',type=Path,help='Private local UI token file for a stable operator-screen restart')
    ap.add_argument('--no-browser',action='store_true');ap.add_argument('--port',type=int,default=0)
    a=ap.parse_args()
    if a.start_in_practice and not (a.connect_robot and a.control_mode=='upstream'):ap.error('Practice selector requires the physical-capable upstream screen')
    if a.readback_preview and not (a.connect_robot and a.control_mode=='upstream'):ap.error('Readback preview requires the physical upstream adapter')
    if a.connect_robot and MOTOR_CONTROL_LOCK.exists():ap.error('Motor control disabled for this installation (MOTOR_CONTROL_DISABLED)')
    if a.connect_robot and a.simulator=='mujoco':ap.error('MuJoCo practice cannot be combined with a robot connection')
    if a.control_mode=='cartesian' and (a.simulator!='mujoco' or a.connect_robot):ap.error('Cartesian control requires local MuJoCo simulation')
    if a.control_mode=='upstream' and not a.connect_robot and a.simulator!='mujoco':ap.error('Local original controls require MuJoCo')
    if a.control_mode=='upstream' and a.connect_robot and not a.upstream_reference:ap.error('Exact --upstream-reference calibration binding is required before connecting')
    if a.control_mode=='upstream' and a.input_backend=='apple':ap.error('Original controls require --input-backend hid (or auto)')
    if a.connect_robot and a.input_backend!='apple' and a.control_mode!='upstream':ap.error('Physical HID input requires original controls and a measured reference')
    token=secrets.token_urlsafe(32)
    if a.ui_session_file:
        if a.ui_session_file.exists():
            token=a.ui_session_file.read_text().strip()
            if len(token)<32 or any(c not in 'ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789_-' for c in token):ap.error('Invalid local UI session file')
        else:
            a.ui_session_file.parent.mkdir(parents=True,exist_ok=True)
            fd=os.open(a.ui_session_file,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
            with os.fdopen(fd,'w') as stream:stream.write(token)
    if a.connect_robot and a.control_mode=='upstream':
        from upstream_hardware import UpstreamHardware,PhysicalReference
        reference=PhysicalReference.load(a.upstream_reference)
        robot=UpstreamHardware(Robot(a.config),reference)
    elif a.connect_robot:robot=Robot(a.config)
    elif a.control_mode=='upstream':
        from upstream_simulator import UpstreamSimulator
        robot=UpstreamSimulator(a.model)
    elif a.simulator=='mujoco':
        from mujoco_simulator import MujocoRobot
        robot=MujocoRobot(a.model)
    else:
        from simulator import SimulatedRobot
        robot=SimulatedRobot()
    mapping=robot.make_mapping() if a.control_mode=='upstream' else None
    if a.control_mode=='cartesian':
        mapping=robot.make_cartesian_mapping();mapping.feedback=robot.controller_feedback
    backend='hid' if a.control_mode=='upstream' else a.input_backend
    if backend=='auto':
        try:
            import hid
            ids={r['product_id'] for r in hid.enumerate(0x057e,0)}
            backend='hid' if {0x2006,0x2007}<=ids else 'apple'
        except ImportError:backend='apple'
    preview3d=None
    if a.readback_preview:
        from readback_preview import ReadbackPreview
        preview3d=ReadbackPreview(reference,a.model)
    practice_factory=None
    if a.connect_robot and a.control_mode=='upstream':
        from practice040 import Practice040
        practice_factory=Practice040
    try:bridge=Bridge(robot,a.reader,mapping=mapping,input_backend=backend,preview3d=preview3d,practice_factory=practice_factory)
    except Exception:
        if preview3d:preview3d.close()
        if hasattr(robot,'close'):robot.close()
        raise
    if a.start_in_practice:bridge.action({'op':'control_target','target':'practice'})
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def send(self,code,value,kind='application/json'):
            data=json.dumps(value,allow_nan=False).encode() if kind=='application/json' else value
            self.send_response(code);self.send_header('Content-Type',kind);self.send_header('Content-Length',str(len(data)));self.send_header('Cache-Control','no-store');self.send_header('X-Frame-Options','DENY');self.end_headers();self.wfile.write(data)
        def authorized(self):
            return self.headers.get('X-Teleop-Token')==token and self.headers.get('Origin',origin)==origin and self.headers.get('Host')==origin.split('//')[1]
        def do_GET(self):
            if self.path=='/':return self.send(200,Path(__file__).with_name('index.html').read_bytes(),'text/html; charset=utf-8')
            if not self.authorized():return self.send(403,{'error':'Local session required'})
            if self.path=='/state':return self.send(200,bridge.snapshot())
            viewer=bridge.robot if bridge.robot.simulation else preview3d or bridge.robot
            if self.path=='/frame.jpg' and hasattr(viewer,'frame'):
                frame=viewer.frame()
                if frame is None:return self.send(503,{'error':'Waiting for simulator renderer'})
                return self.send(200,frame,'image/jpeg')
            return self.send(404,{'error':'Not found'})
        def do_POST(self):
            if not self.authorized():return self.send(403,{'error':'Local session required'})
            try:
                n=int(self.headers.get('Content-Length','0'))
                if not 0<n<=2048:raise ValueError('Invalid body size')
                b=json.loads(self.rfile.read(n))
                if self.path=='/heartbeat':
                    if b.get('focused') is True:bridge.ui_seen=time.monotonic()
                    else:bridge.ui_seen=0
                elif self.path=='/view':
                    viewer=bridge.robot if bridge.robot.simulation else preview3d or bridge.robot
                    if not hasattr(viewer,'set_view'):raise ValueError('No 3D view available')
                    viewer.set_view(b.get('view'))
                elif self.path=='/action':bridge.action(b)
                else:raise ValueError('Unknown route')
                self.send(200,{'ok':True})
            except Exception as e:self.send(400,{'error':str(e)})
    server=ThreadingHTTPServer(('127.0.0.1',a.port),Handler);origin=f'http://127.0.0.1:{server.server_port}'
    url=origin+'/#'+token
    # The token stays on the local machine and is never sent to the robot or included in HTTP logs.
    print('Joy-Con operator screen: '+url,flush=True)
    if not a.no_browser:subprocess.run(['open','-a','Google Chrome',url],check=False)
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    try:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:bridge.close();server.server_close()
if __name__=='__main__':main()
