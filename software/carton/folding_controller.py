"""Coordinated two-hand folding from tag registration and aligned depth.

This controller has no simulator or hardware import. Its port accepts paired
Cartesian targets and supplies pixel-derived scene estimates and robot feedback.
"""
from __future__ import annotations
import math
import numpy as np

from carton.geometry import Box

_box=Box()
L,W,H,F=_box.length,_box.width,_box.height,_box.flap

def short_flap_points(theta,y=0.):
    r=.095;extra=.012
    return {side:[sign*(L/2-r*math.sin(theta)+extra*math.cos(theta)),y,H+r*math.cos(theta)+extra*math.sin(theta)] for side,sign in [('left',-1),('right',1)]}

def long_flap_point(theta,sign,x,tilt=.65,r=.12,extra=.020):
    point=[x,sign*(W/2-r*math.sin(theta)+extra*math.cos(theta)),H+.0035+r*math.cos(theta)+extra*math.sin(theta)]
    direction=np.array([0,-tilt*(1-theta/(math.pi/2)),1.]);direction/=np.linalg.norm(direction)
    return point,{'direction':direction.tolist()}

class FoldingController:
    flap_assignments={'left':('short_left','long_near'),'right':('short_right','long_far')}
    def __init__(self,port,*,rear_cart=False):
        self.port=port
        self.rear_cart=rear_cart
        self.visual_evidence={}
        self.trace=[]
        self.box=None

    def sense(self,label):
        reading=self.port.observe(label)
        if reading.get('world_from_box') is not None:self.box=np.asarray(reading['world_from_box'])
        if self.box is None:raise ValueError('Carton tag has not supplied a metric pose')
        self.trace.append({'label':label,'angles':reading['angles'],'tags':reading['tags']})
        return reading

    def move(self,points,seconds,label,orientation='down'):
        targets={side:(self.box[:3,:3]@np.asarray(p)+self.box[:3,3]).tolist() for side,p in points.items()}
        if isinstance(orientation,dict):
            def rotate(value):
                if not isinstance(value,dict):return value
                return {k:(self.box[:3,:3]@v).tolist() if k in ('direction','tangent') else v for k,v in value.items()}
            orientation=rotate(orientation) if 'direction' in orientation else {side:rotate(value) for side,value in orientation.items()}
        return self.port.move_arms(targets,seconds,label,orientation)

    def require_folded(self,reading,flaps):
        for flap in flaps:
            row=reading['angles'].get(flap)
            if row is None or not 80<=row['degrees']<=101:
                raise ValueError(f'{flap} has no visual closure confirmation: {row}')
            self.visual_evidence[flap]=row

    def run(self):
        self.sense('Register carton from rendered RGB and aligned depth')
        # Each short flap is contacted by a different real gripper.
        for i,theta in enumerate(np.linspace(0,math.pi/2,19)):
            points=short_flap_points(theta,y=-.06 if self.rear_cart else 0.)
            if i==0:self.move({a:np.array(p)+[0,0,.055] for a,p in points.items()},1.,'Approach short flaps')
            self.move(points,.22,f'Fold both short flaps: {math.degrees(theta):.0f} degrees')
            if i%4==0:self.sense('Observe short-flap progress')
        self.move({'left':[-.24,-.10,.29],'right':[.24,-.10,.29]} if self.rear_cart else
                  {'left':[-.23,0,.31],'right':[.23,0,.31]},1.,'Clear view of short flaps')
        reading=self.sense('Verify both short flaps before covering them')
        self.require_folded(reading,['short_left','short_right'])
        for flap,side,sign,x in [('long_far','right',1,.10),('long_near','left',-1,0.)]:
            reading=self.sense('Locate '+flap)
            measured=reading['angles'].get(flap)
            if measured is None:raise ValueError(f'Depth/marker observation required before contacting {flap}')
            start=math.radians(np.clip(measured['degrees']-5,-30,0))
            for i,theta in enumerate(np.linspace(start,math.pi/2,25)):
                far_reach=self.rear_cart and flap=='long_far'
                progress=max(0.,math.sin(theta))
                point,ori=long_flap_point(theta,sign,x,tilt=5. if far_reach else .65,
                    r=.04+.08*progress if far_reach else .12,
                    extra=.005+.015*progress if far_reach else .020)
                if i==0:self.move({side:np.array(point)+[0,0,.015 if far_reach else .035]},1.,'Approach '+flap,ori)
                self.move({side:point},.25,f'Fold {flap}: {math.degrees(theta):.0f} degrees',ori)
                if i%4==0:self.sense('Observe '+flap+' progress')
            reading=self.sense('Verify '+flap)
            self.require_folded(reading,[flap])
        # Hold both long panels down together; this also retains the short flaps
        # beneath them. The fixed fingertip offset comes from the gripper CAD.
        self.move({'left':[0.,-.0215,H+.0035+.007],
                   'right':[.10,.0215,H+.0035+.007]},.8,'Press and retain both long flaps')
        self.sense('Check paired contact hold')
        self.port.move_arms({},2.,'Verify two-second closure',None)
        self.require_folded(self.sense('Final visible long-flap closure'),['long_far','long_near'])
        return {'visual_sequence_passed':True,'visual_flap_evidence':self.visual_evidence,'trace':self.trace}
