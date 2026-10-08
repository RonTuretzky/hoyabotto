"""Strict mapping of the observed macOS combined original Joy-Con profile."""
import math, time
LAYERS = [('shoulder_pan','shoulder_lift'),('wrist_flex','elbow_flex'),('wrist_roll','gripper')]
class Mapping:
    def __init__(self):
        self.session = None
        self.identity = None
        self.sequence = -1
        self.checked = set()
        self.layer = 0
        self.previous_menu = False

    def decode(self, f, now=None):
        now = time.time() if now is None else now
        if f.get('source') != 'live' or f.get('schema_version') != 1 or f.get('event') == 'shutdown':
            raise ValueError('Live native controller input required')
        stamp = f.get('timestamp')
        if type(stamp) not in (float,int) or not math.isfinite(stamp) or not 0 <= now-stamp <= .2:
            raise ValueError('Controller samples stale')
        cs = f.get('controllers',[])
        if not cs:
            self.checked.clear()
            raise ValueError('No Joy-Cons detected. Wake both controllers with a button press.')
        if len(cs) != 1 or not cs[0].get('connected') or cs[0].get('role') != 'pair':
            self.checked.clear()
            raise ValueError('Waiting for macOS to present the combined Joy-Con (L/R) pair.')
        if cs[0].get('remapped'):
            raise ValueError('This controller has an OS remapping; restore its default layout before practice.')
        c=cs[0]
        if self.session != f['session_id'] or self.identity != c['id']:
            self.session,self.identity=f['session_id'],c['id'];self.checked.clear();self.sequence=-1
        if type(f['sequence']) is not int or f['sequence'] < self.sequence:
            raise ValueError('Out-of-order reader frame')
        self.sequence=f['sequence']
        buttons=c['buttons'];pads=c['pads']
        def pressed(key):
            b=buttons[key]
            if type(b['pressed']) is not bool: raise ValueError('Invalid button')
            return b['pressed']
        dead={'left':pressed('Left Trigger'),'right':pressed('Right Trigger')}
        axes={}
        for side in ('Left','Right'):
            for axis in ('x','y'):
                v=pads[side+' Thumbstick'][axis]['filtered']
                if type(v) not in (float,int) or not math.isfinite(v) or abs(v)>1: raise ValueError('Invalid axis')
                axes[side.lower()+axis]=v
        # Each session verifies that both physical triggers produce events; the
        # operator sees which control lights before the Arm button becomes available.
        if c.get('input_event_count',0)>0:
            for side,on in dead.items():
                if on:self.checked.add(side)
        neutral=not any(dead.values()) and not any(axes.values())
        menu=pressed('Button Menu')
        if menu and not self.previous_menu and neutral:self.layer=(self.layer+1)%3
        self.previous_menu=menu
        return dict(deadman=dead,axes=axes,neutral=neutral,stop=pressed('Button Options'),
                    ready=self.checked=={'left','right'},identity=(self.session,self.identity),layer=self.layer)

    def command(self, d, scope):
        out=dict(rates={},deadman=d['deadman'],linear=0.,angular=0.)
        if scope in ('left','right','both'):
            xjoint,yjoint=LAYERS[self.layer]
            for side in ('left','right') if scope=='both' else (scope,):
                for axis,joint in (('x',xjoint),('y',yjoint)):
                    out['rates'][side+'_arm_'+joint]=d['axes'][side+axis]*80 if d['deadman'][side] else 0.
        elif scope=='head':
            for axis,joint in (('x','head_motor_1'),('y','head_motor_2')):
                out['rates'][joint]=d['axes']['left'+axis]*60 if d['deadman']['left'] else 0.
        elif scope=='drive':
            if all(d['deadman'].values()):
                forward,turn=d['axes']['lefty'],d['axes']['rightx']
                scale=max(1,abs(forward)+abs(turn))
                out['linear']=forward*.02/scale;out['angular']=-turn*.16/scale
        else:raise ValueError('Unknown control scope')
        return out
