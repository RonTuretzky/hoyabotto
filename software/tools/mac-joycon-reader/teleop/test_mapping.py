import copy, math, unittest
from mapping import Mapping
from bridge import PreviewRobot, Bridge

def frame():
    return dict(source='live',schema_version=1,event='sample',timestamp=10.,session_id='reader',sequence=1,controllers=[dict(id='pair',connected=True,role='pair',remapped=False,input_event_count=1,
        buttons={k:{'pressed':False} for k in ('Left Trigger','Right Trigger','Button Options','Button Menu')},
        pads={side+' Thumbstick':{axis:{'filtered':0.} for axis in ('x','y')} for side in ('Left','Right')})])

class Tests(unittest.TestCase):
    def test_every_joint_accessible_and_deadman(self):
        m=Mapping();f=frame();d=m.decode(f,10.);self.assertFalse(d['ready'])
        f['controllers'][0]['buttons']['Left Trigger']['pressed']=True
        m.decode(f,10.)
        f['controllers'][0]['buttons']['Right Trigger']['pressed']=True
        d=m.decode(f,10.);self.assertTrue(d['ready'])
        for p in f['controllers'][0]['pads'].values():
            for a in p.values():a['filtered']=1
        d=m.decode(f,10.);names=set()
        for layer in range(3):
            m.layer=layer;c=m.command(d,'both');names.update(c['rates']);self.assertTrue(all(v==80 for v in c['rates'].values()))
        self.assertEqual(len(names),12)
        self.assertEqual(set(m.command(d,'head')['rates']),{'head_motor_1','head_motor_2'})
        with self.assertRaisesRegex(ValueError,'geometry'):m.command(d,'drive')
        m.update_robot_status({'teleop':{'wheelbase_m':.45,'wheel_limit_m_s':.02}})
        c=m.command(d,'drive');self.assertAlmostEqual(abs(c['linear'])+abs(c['angular'])*.225,.02)
        f['controllers'][0]['buttons']['Left Trigger']['pressed']=False
        d=m.decode(f,10.);self.assertFalse(any(m.command(d,'left')['rates'].values()));self.assertEqual(m.command(d,'drive')['linear'],0)
    def test_layer_change_only_neutral(self):
        m=Mapping();f=frame();f['controllers'][0]['buttons']['Button Menu']['pressed']=True
        m.decode(f,10.);self.assertEqual(m.layer,1)
        m.decode(f,10.);self.assertEqual(m.layer,1)
        f['controllers'][0]['buttons']['Button Menu']['pressed']=False;m.decode(f,10.)
        f['controllers'][0]['buttons']['Button Menu']['pressed']=True
        f['controllers'][0]['buttons']['Left Trigger']['pressed']=True
        m.decode(f,10.);self.assertEqual(m.layer,1)
    def test_no_demo_stale_remapped_disconnect(self):
        bad=[]
        for k,v in [('source','demo'),('event','shutdown'),('timestamp',8),('timestamp',11),('timestamp',float('nan'))]:
            f=frame();f[k]=v;bad.append(f)
        for k,v in [('connected',False),('role','unknown'),('remapped',True)]:
            f=frame();f['controllers'][0][k]=v;bad.append(f)
        f=frame();f['controllers'][0]['pads']['Left Thumbstick']['x']['filtered']=float('inf');bad.append(f)
        for f in bad:
            with self.assertRaises(ValueError):Mapping().decode(f,10.)
    def test_reconnect_requires_checks_again(self):
        m=Mapping();f=frame()
        for b in ('Left Trigger','Right Trigger'):f['controllers'][0]['buttons'][b]['pressed']=True
        self.assertTrue(m.decode(f,10.)['ready'])
        f['controllers'][0]['id']='new'
        for b in ('Left Trigger','Right Trigger'):f['controllers'][0]['buttons'][b]['pressed']=False
        self.assertFalse(m.decode(f,10.)['ready'])
    def test_preview_cannot_issue_commands(self):
        r=PreviewRobot();self.assertTrue(r.call('status')['preview'])
        for path in ('claim','input','release'):
            with self.assertRaises(ValueError):r.call(path,{})
if __name__=='__main__':unittest.main()
