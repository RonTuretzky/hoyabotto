"""Local, memory-only 16-servo plant exercising the real owner + mailbox API.

No sockets, credentials, robot runtime paths, serial libraries, or devices are
used. Positions/ranges/loads are invented test values, not robot measurements.
"""
import math
import sys
import tempfile
import threading
import time
import json
from pathlib import Path
from types import SimpleNamespace

BRIDGE = Path(__file__).resolve().parents[3] / 'docs/commissioning/2026-10-07-paddle-success/qwen-bridge'
if str(BRIDGE) not in sys.path:
    sys.path.insert(0, str(BRIDGE))
from gemma_hardware_owner import HardwareOwner
from gemma_direct_client import DirectJointClient, atomic_json
from joycon_teleop import SCOPES
from joycon_teleop_api import TeleopAPI
from wheel_pulse_executor import WHEELS, WHEEL_RADIUS_M, WHEELBASE_M, TICKS_PER_REV


class VirtualBus:
    """Integer register reads/writes with idealized position and wheel response."""
    def __init__(self):
        self.motors = dict.fromkeys(SCOPES['both'] + SCOPES['head'] + list(WHEELS))
        self.r = {n:dict(Torque_Enable=0, Operating_Mode=0, Homing_Offset=0,
                         Min_Position_Limit=100, Max_Position_Limit=4000,
                         Present_Position=2048, Goal_Position=2048, Goal_Velocity=0,
                         Lock=1, Torque_Limit=1000, Acceleration=0, P_Coefficient=16,
                         Goal_Time=0, Status=0, Present_Load=0, Present_Voltage=120,
                         Present_Temperature=25, Present_Velocity=0)
                  for n in self.motors}
        self.wheels = {n:2048.0 for n in WHEELS}
        self.pose = dict(x=0.0, y=0.0, heading=0.0)

    def read(self, field, name, **kwargs):
        return self.r[name].get(field, 0)

    def write(self, field, name, value, **kwargs):
        self.r[name][field] = value

    def disable_torque(self, names, **kwargs):
        for n in names: self.write('Torque_Enable', n, 0)

    def step(self, dt):
        for n, r in self.r.items():
            if n in WHEELS:
                v = r['Goal_Velocity'] if r['Torque_Enable'] and r['Operating_Mode'] == 1 else 0
                r['Present_Velocity'] = v
                self.wheels[n] = (self.wheels[n] + v*dt) % TICKS_PER_REV
                r['Present_Position'] = round(self.wheels[n]) % TICKS_PER_REV
            elif r['Torque_Enable']:
                r['Present_Position'] = r['Goal_Position']
        scale = 2*math.pi*WHEEL_RADIUS_M/TICKS_PER_REV
        left, right = (-self.r[WHEELS[0]]['Present_Velocity']*scale,
                       self.r[WHEELS[1]]['Present_Velocity']*scale)
        speed, turn = (left+right)/2, (right-left)/WHEELBASE_M
        mid = self.pose['heading']+turn*dt/2
        self.pose['x'] += speed*math.cos(mid)*dt
        self.pose['y'] += speed*math.sin(mid)*dt
        self.pose['heading'] = (self.pose['heading']+turn*dt+math.pi)%(2*math.pi)-math.pi


class SimulatedRobot:
    preview = True
    simulation = True

    def __init__(self,bus=None):
        self.temp = tempfile.TemporaryDirectory(prefix='joycon-simulation-')
        self.folder = Path(self.temp.name)
        self.bus = bus if bus is not None else VirtualBus()
        cal = {n:SimpleNamespace(range_min=100, range_max=4000, homing_offset=0)
               for n in self.bus.motors}
        self.owner = HardwareOwner([self.bus], cal, lambda b,n:dict(b.r[n]),
                                   position_scope=SCOPES['both'], paddle_profile=True,
                                   camera_metadata=lambda:dict(received_at=time.time(), seq=1),
                                   wheels=True, teleop=True)
        self.owner.inspect()
        self.persist()
        self.client = DirectJointClient(self.folder, {n:vars(c) for n,c in cal.items()})
        self.api = TeleopAPI(self.client)
        self.stopping = threading.Event()
        self.thread = threading.Thread(target=self.run, name='virtual-servo-owner', daemon=True)
        self.thread.start()

    def persist(self):
        self.owner.state['simulation'] = True
        self.owner.state['virtual_pose'] = dict(self.bus.pose)
        atomic_json(self.folder/'status.json', self.owner.state)

    def run(self):
        last_command = None
        previous = time.monotonic()
        try:
            while not self.stopping.is_set():
                now = time.monotonic()
                dt=min(now-previous,.1);previous=now
                try:
                    self.bus.step(dt)
                    mailbox = self.folder/'teleop-input.json'
                    if self.owner.teleop.active and mailbox.exists():
                        self.owner.teleop.accept(json.loads(mailbox.read_text()))
                    self.owner.poll()
                except (RuntimeError, ValueError, TypeError, KeyError) as e:
                    self.owner.release_all(str(e))
                command = self.folder/'command.json'
                if command.exists():
                    c = json.loads(command.read_text())
                    if c['id'] != last_command:
                        last_command = c['id']
                        try: self.owner.command(c)
                        except ValueError as e:
                            self.owner.state['last_rejected'] = dict(id=c['id'], reason=str(e))
                        except RuntimeError as e:
                            self.owner.state['failed_command_id'] = c['id']
                            self.owner.release_all(str(e))
                self.persist()
                if hasattr(self.bus,'render_frame'):
                    try:self.bus.render_frame()
                    except Exception as e:self.bus.render_error=str(e)
                self.stopping.wait(.02)
        finally:
            self.owner.release_all('Simulation closed')
            self.persist()
            if hasattr(self.bus,'close'):self.bus.close()

    def call(self, path, body=None, timeout=None):
        if self.stopping.is_set(): raise ValueError('Simulation closed')
        result = self.api.status() if path == 'status' else self.api.post(path, body)
        if path == 'status':
            state = self.client.status()
            result.update(preview=True, simulation=True, virtual_pose=state['virtual_pose'])
        if 'status' in result:
            result['status'].update(preview=True, simulation=True,
                                    virtual_pose=self.client.status()['virtual_pose'])
        return result

    def close(self):
        self.stopping.set()
        self.thread.join(timeout=3)
        if self.thread.is_alive(): raise RuntimeError('Virtual owner did not exit')
        self.temp.cleanup()
