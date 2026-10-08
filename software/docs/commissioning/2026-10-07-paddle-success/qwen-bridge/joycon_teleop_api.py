"""Operator-only mTLS routes; streaming mailbox never runs in an LLM tool loop."""
import json
import secrets
import threading
import time
from gemma_direct_client import atomic_json
from joycon_teleop import INPUT_TTL, SCOPES, validate_input


class TeleopAPI:
    def __init__(self, client, clock=time.monotonic):
        self.client = client
        self.clock = clock
        self.lock = threading.Lock()
        self.permit = None
        self.token = None
        self.sequence = 0

    def status(self):
        s = self.client.status()
        return {k:s.get(k) for k in ('started','time','status_age_s','phase','ok','teleop','enabled_motors','last_stop','calibration_mismatches','missing_buses')} | {
            'motors': {n:{k:r.get(k) for k in ('Present_Position','Present_Load','Present_Temperature','Present_Voltage','Torque_Enable','Status')} for n,r in s.get('rows',{}).items()}}

    def renew(self):
        self.permit = secrets.token_urlsafe(24)
        self.expires = self.clock()+INPUT_TTL
        return {'permit':self.permit, 'owner_started':self.started, 'token':self.token, 'input_timeout_s':INPUT_TTL}

    def command(self, op, **args):
        # Same interprocess lock as the normal API. The owner enforces exclusion too.
        with self.client.serialized():
            generation = self.client.cancel_generation
            s = self.client.status()
            if not 0 <= s['status_age_s'] <= 1: raise ValueError('Owner status stale')
            path = self.client.folder/'command.json'
            old = json.loads(path.read_text()) if path.exists() else {}
            if old.get('op') == 'stop' and old.get('session_started') == s['started'] and s.get('completed') != old.get('id'):
                raise ValueError('STOP pending')
            ident = max(time.time_ns(), old.get('id',0)+1)
            latest = self.client.status()
            current = json.loads(path.read_text()) if path.exists() else {}
            if current != old or self.client.cancel_generation != generation or latest.get('stop_count') != s.get('stop_count'):
                raise ValueError('STOP or another writer interrupted manual dispatch')
            atomic_json(path, dict(op=op, id=ident, session_started=s['started'], **args))
            deadline = self.clock()+6
            while self.clock() < deadline:
                latest = self.client.status()
                if latest['started'] != s['started']: raise ValueError('Owner restarted')
                if (latest.get('last_rejected') or {}).get('id') == ident:
                    raise ValueError(latest['last_rejected']['reason'])
                if latest.get('completed') == ident: return latest
                if latest.get('failed_command_id') == ident: raise ValueError('Owner faulted')
                time.sleep(.01)
            raise ValueError('Manual command timed out; no automatic retry')

    def post(self, action, body):
        if not isinstance(body, dict): raise ValueError('Object required')
        if not self.lock.acquire(blocking=False): raise ValueError('Manual request in progress')
        try:
            if action == 'claim':
                if set(body) != {'scope'} or body['scope'] not in SCOPES: raise ValueError('Select a valid scope')
                token = secrets.token_urlsafe(32)
                s = self.command('teleop_claim', token=token, scope=body['scope'])
                self.token, self.scope, self.started, self.sequence = token, body['scope'], s['started'], 0
                return self.renew()
            if action == 'release':
                if set(body) != {'token'}: raise ValueError('Session token required')
                s = self.command('teleop_end', token=body['token'])
                self.permit = self.token = None
                return {'released':not s.get('enabled_motors') and s.get('released') is True}
            if action != 'input': raise ValueError('Unknown manual route')
            if set(body) != {'token','permit','sequence','owner_started','rates','deadman','linear','angular'}:
                raise ValueError('Invalid manual input fields')
            now = self.clock()
            if not self.token or body['token'] != self.token or body['permit'] != self.permit or now > self.expires:
                raise ValueError('Expired or invalid one-use input permit; re-arm required')
            self.permit = None  # consume before validation: never retry a motion packet
            c = {'token':self.token, 'session_started':body['owner_started'], 'sequence':body['sequence'],
                 'received':now, 'expires':now+INPUT_TTL,
                 **{k:body[k] for k in ('rates','deadman','linear','angular')}}
            validate_input(c, self.scope)
            if c['sequence'] <= self.sequence or c['session_started'] != self.started: raise ValueError('Old input or owner')
            s = self.client.status()
            if s['started'] != self.started or not 0 <= s['status_age_s'] <= .5 or (s.get('teleop') or {}).get('token') != self.token or not s['teleop']['active']:
                raise ValueError('Manual ownership lost')
            atomic_json(self.client.folder/'teleop-input.json', c)
            self.sequence = c['sequence']
            if self.sequence == 1:
                # Acknowledge the initial neutral frame only after the owner has
                # consumed it; a later held-trigger frame must not overwrite it.
                deadline = self.clock()+.25
                while self.clock() < deadline:
                    latest = self.client.status()
                    manual = latest.get('teleop') or {}
                    if latest['started'] != self.started or manual.get('token') != self.token or not manual.get('active'):
                        raise ValueError('Manual session stopped before neutral acknowledgement')
                    if manual.get('neutral_seen'):
                        break
                    time.sleep(.005)
                else: raise ValueError('Owner did not acknowledge neutral input')
            return self.renew() | {'accepted_sequence':self.sequence, 'owner_sequence':s['teleop']['sequence'], 'status':self.status()}
        finally:
            self.lock.release()
