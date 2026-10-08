#!/usr/bin/env python3
"""Local operator UI + native Joy-Con input -> existing pinned robot mTLS API.

Starts DISARMED. No automatic arm/retry/reconnect. No serial access or model calls.
"""
import argparse, json, os, secrets, signal, ssl, subprocess, threading, time, urllib.request, urllib.parse, socket
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
    def __init__(self,robot,reader,*,start=True):
        self.robot=robot;self.reader=reader;self.mapping=Mapping();self.lock=threading.RLock()
        self.frame=None;self.decoded=None;self.reader_error='Waiting for controller input'
        self.generation=0;self.armed=False;self.busy=False;self.scope='left';self.session=None;self.identity=None
        self.reason='Disarmed — test both triggers, then release them and center the sticks'
        self.robot_state={};self.ui_seen=0;self.sequence=0;self.rtt=None;self.closing=False
        self.release_pending=0;self.worker=None;self.reader_thread=None;self.proc=None
        if start:self.start()
    def start(self):
        self.proc=subprocess.Popen([str(self.reader),'--json','--hz','30','--deadzone','0.12'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL,text=True,bufsize=1)
        self.reader_thread=threading.Thread(target=self.read_frames,daemon=True);self.reader_thread.start()
        self.worker=threading.Thread(target=self.run,daemon=True);self.worker.start()
    def receive(self, frame):
        with self.lock:
            self.frame=frame
            try:self.decoded=self.mapping.decode(frame);self.reader_error=None
            except (ValueError,KeyError,TypeError,AttributeError) as e:
                self.decoded=None;self.reader_error=str(e)
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
        if time.monotonic()-self.ui_seen>.8:raise ValueError('Operator screen lost focus or stopped responding')
        return self.decoded
    def snapshot(self):
        with self.lock:
            d=self.decoded or {}
            return dict(preview=self.robot.preview,simulation=self.robot.simulation,armed=self.armed,busy=self.busy or bool(self.release_pending),scope=self.scope,layer=self.mapping.layer,reason=self.reason,
                        controller=d,reader_error=self.reader_error,robot=self.robot_state,rtt_ms=self.rtt,
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
        if op=='stop':
            reason='Operator STOP';session,generation=self.detach(reason)
            threading.Thread(target=self.finish_release,args=(session,generation,reason),daemon=True).start();return
        with self.lock:
            if self.busy or self.release_pending:raise ValueError('Wait for the current operation')
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
            if not d['ready'] or not d['neutral']:raise ValueError('Test both triggers, release them, and center sticks before arming')
            scope=b.get('scope')
            if scope not in ('left','right','both','head','drive'):raise ValueError('Unknown scope')
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
                self.session=result;self.armed=True
                self.reason='PRACTICE — simulated components only' if self.robot.simulation else 'Armed — hold trigger to move'
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
                body=self.mapping.command(d,self.scope) | {k:session[k] for k in ('token','permit','owner_started')} | {'sequence':self.sequence}
            result=self.robot.call('input',body)
            with self.lock:
                if self.armed and self.generation==generation:
                    self.session=result;self.robot_state=result.get('status',self.robot_state)
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
                    state=self.robot.call('status',timeout=2)
                    with self.lock:self.robot_state=state
                except Exception as e:
                    with self.lock:self.robot_state={'error':str(e)}
                last_status=time.monotonic()
            time.sleep(max(.005,.06-(time.monotonic()-started)))
    def close(self):
        self.closing=True;self.release('Teleop closed')
        if self.proc:
            self.proc.terminate()
            try:self.proc.wait(timeout=3)
            except subprocess.TimeoutExpired:self.proc.kill();self.proc.wait()
        if self.worker:self.worker.join(timeout=3)
        if self.reader_thread:self.reader_thread.join(timeout=3)
        if hasattr(self.robot,'close'):self.robot.close()


def main():
    ap=argparse.ArgumentParser(description=__doc__,allow_abbrev=False)
    ap.add_argument('--simulator',choices=('registers','mujoco'),default='registers',help='Local simulator backend')
    ap.add_argument('--model',type=Path,help='Optional XLeRobot MuJoCo model XML')
    ap.add_argument('--connect-robot',action='store_true',help='Explicitly connect to the robot. Default is a local input preview with no network access.')
    ap.add_argument('--config',default=os.environ.get('XLEROBOT_ADMIN_CONFIG',DEFAULT_CONFIG))
    ap.add_argument('--reader',type=Path,default=Path(__file__).resolve().parents[1]/'.build/release/MacJoyConReader')
    ap.add_argument('--no-browser',action='store_true');ap.add_argument('--port',type=int,default=0)
    a=ap.parse_args()
    if a.connect_robot and a.simulator=='mujoco':ap.error('MuJoCo practice cannot be combined with a robot connection')
    if a.connect_robot:robot=Robot(a.config)
    elif a.simulator=='mujoco':
        from mujoco_simulator import MujocoRobot
        robot=MujocoRobot(a.model)
    else:
        from simulator import SimulatedRobot
        robot=SimulatedRobot()
    try:bridge=Bridge(robot,a.reader)
    except Exception:
        if hasattr(robot,'close'):robot.close()
        raise
    token=secrets.token_urlsafe(32)
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
            if self.path=='/frame.jpg' and hasattr(robot,'frame'):
                frame=robot.frame()
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
                elif self.path=='/view' and hasattr(robot,'set_view'):robot.set_view(b.get('view'))
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
