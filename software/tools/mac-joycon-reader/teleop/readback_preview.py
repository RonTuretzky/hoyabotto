"""Read-only MuJoCo illustration of existing encoder feedback. No reader or transport.

Uses the upstream logical-angle convention; it is not a calibrated digital twin
or a collision check. After model initialization it never steps physics, sends commands, or claims motors.
"""
import copy
import math
import threading
import time
import mujoco
from model040_preview import Model040PreviewBus
from upstream import to_model


class ReadbackPreview:
    def __init__(self,reference,model_path=None):
        self.reference=reference
        self.bus=Model040PreviewBus(model_path,render=True,forward_arms=True)
        self.bus.render_interval=.1
        self.lock=threading.Lock();self.state=None;self.received=None
        self.stopping=threading.Event();self.error=None
        self.thread=threading.Thread(target=self.run,name='read-only-mujoco-preview',daemon=True)
        self.thread.start()

    def update(self,state):
        # Copy only observed feedback; no shared owner/command objects.
        with self.lock:self.state=copy.deepcopy(state);self.received=time.monotonic()

    def info(self):
        with self.lock:
            age=None if self.received is None else time.monotonic()-self.received
        return dict(engine=self.bus.engine,model_hardware='0.4 kit layout · two SO-101 arms · two drive wheels · schematic OAK',arm_revision=self.bus.arm_manifest['revision'],read_only=True,physics=False,
                    frame_sequence=self.bus.frame_sequence,
                    frame_age_s=None if not self.bus.last_render else time.monotonic()-self.bus.last_render,
                    feedback_age_s=age,render_error=self.error,
                    limitations='Maker SO-101 joint frames/meshes with a two-wheel cart layout. Mounting and encoder zeroes are unvalidated preview assumptions. Base travel is not reconstructed.')

    def run(self):
        try:
            while not self.stopping.is_set():
                with self.lock:state=self.state;received=self.received
                try:
                    if state and received is not None and time.monotonic()-received+state.get('status_age_s',10)<1.5:
                        positions={}
                        for name in self.bus.map:
                            ticks=state['motors'][name]['Present_Position']
                            q=self.bus.pose_ticks(self.reference,name,ticks)
                            if not math.isfinite(q):raise ValueError('Nonfinite preview feedback')
                            positions[name]=q
                        for name,q in positions.items():
                            aid,qadr,_,_,_=self.bus.map[name]
                            self.bus.data.qpos[qadr]=q
                            if aid is not None:self.bus.data.ctrl[aid]=q
                        self.bus.data.qvel[:]=0
                        mujoco.mj_forward(self.bus.model,self.bus.data)
                        self.error=None
                    else:self.error='Preview waiting for fresh encoder feedback'
                    self.bus.render_frame()
                except Exception as e:self.error=str(e)
                self.stopping.wait(.05)
        finally:self.bus.close()

    def frame(self):return self.bus.frame()
    def set_view(self,view):self.bus.set_view(view)
    def close(self):
        self.stopping.set();self.thread.join(timeout=3)
        if self.thread.is_alive():raise RuntimeError('Preview renderer did not exit')
