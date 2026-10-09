"""Small, observable practice milestones; no physical robot or force claims."""
import math

LESSONS=[
    ('Meet the wrists','Gently tilt each Joy-Con, then return both wrists to where they started.','Bend or roll; no buttons needed in Practice.'),
    ('Approach the box','Open the right gripper and move its tip to the white R marker.','Right stick moves the hand. R raises; click the stick to lower. ZR toggles the jaw.'),
    ('Pick up the box','Close the right jaw while the tip is at R.','Tap ZR. A nearby closed jaw attaches the practice box.'),
    ('Lift clear','Raise the held box at least 4 cm.','Hold R gently. Keep the other hand clear.'),
    ('Place on P','Move the box over P, lower it, then open the right jaw.','Right stick to P; stick-click lowers; tap ZR to release.'),
]


class BoxCourse:
    def __init__(self):self.active=False;self.step=0;self.seen=set();self.baseline={};self.pickup_height=0.;self.complete=False;self.note='Take the short tutorial, then begin.'
    def begin(self,positions,pickup_height):
        self.active=True;self.complete=False;self.step=0;self.seen=set()
        self.baseline={(s,j):positions.get(s+'_arm_'+j,0.) for s in ('left','right') for j in ('wrist_roll','wrist_flex')}
        self.pickup_height=pickup_height;self.note='Slow movements are easier to control.'
    def update(self,positions,distance,held,box_height,placed):
        if not self.active or self.complete:return
        if self.step==0:
            for side in ('left','right'):
                if any(abs(positions.get(side+'_arm_'+j,0.)-self.baseline[(side,j)])>=12 for j in ('wrist_roll','wrist_flex')):self.seen.add(side)
            if len(self.seen)==2 and all(abs(positions.get(s+'_arm_'+j,0.)-self.baseline[(s,j)])<8 for s in ('left','right') for j in ('wrist_roll','wrist_flex')):
                self.step=1;self.note='Both wrists checked. Aim the right hand at R.'
        elif self.step==1 and distance<.028 and positions['right_arm_gripper']>=55:
            self.step=2;self.note='Good approach. Close the right jaw.'
        elif self.step==2 and held:
            self.step=3;self.note='Box attached in practice. Lift slowly.'
        elif self.step==3 and held and box_height-self.pickup_height>=.04:
            self.step=4;self.note='Clear of the table. Move across to P.'
        elif self.step==4 and placed:
            self.step=5;self.complete=True;self.note='Box placed. Repeat for a smoother run, or explore freely.'
    def status(self,distance,held):
        if self.complete:title,instruction,controls='Course complete','You practiced wrists, approach, grip, lift and placement.','Repeat course or continue free practice.'
        else:title,instruction,controls=LESSONS[self.step]
        return dict(active=self.active,step=self.step,total=len(LESSONS),complete=self.complete,
                    title=title,instruction=instruction,controls=controls,note=self.note,
                    wrists_checked=sorted(self.seen),distance_cm=round(distance*100,1),box_held=held)
