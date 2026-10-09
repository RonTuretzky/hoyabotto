"""Final-table dimensions and full-size carton, in the kit preview's -X-forward frame.

Source: carton-pilot-tags-2026-10-09.md final-table handoff. Placement is model
derived, not a survey. All props are visual; no collision/force claim is made.
"""
import math
import xml.etree.ElementTree as E
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont
from carton.geometry import Box

BOX=Box()
TABLE_TOP=.700
TABLE_DEPTH=.480
TABLE_WIDTH=.500
TABLE_NEAR=.1112  # Model-only nominal setback, not the old unproven 114 mm measurement.
CENTER=(-TABLE_NEAR-.010-BOX.width/2,0.,TABLE_TOP+BOX.height)
# hinge position relative to rim center, inward vector, hinge-line vector, span
FLAPS={
    'right':((0.,BOX.length/2,0.),(0.,-1.,0.),(1.,0.,0.),BOX.width,'R12'),
    'left':((0.,-BOX.length/2,0.),(0.,1.,0.),(1.,0.,0.),BOX.width,'L11'),
    'far':((-BOX.width/2,0.,0.),(1.,0.,0.),(0.,1.,0.),BOX.length,'F13'),
    'near':((BOX.width/2,0.,0.),(-1.,0.,0.),(0.,1.,0.),BOX.length,'N14'),
}


def add_carton_station(root,cache):
    world=root.find('worldbody');asset=root.find('asset')
    for name in ('exercise_box','placement_marker'):
        world.remove(world.find(f"body[@name='{name}']"))
    table=world.find("body[@name='practice_table']")
    table.set('pos',f'{-TABLE_NEAR-TABLE_DEPTH/2} 0 {TABLE_TOP-.015}')
    table.find("geom[@name='practice_table_top']").set('size',f'{TABLE_DEPTH/2} {TABLE_WIDTH/2} .015')
    for i,(x,y) in enumerate(((-.20,-.21),(-.20,.21),(.20,-.21),(.20,.21))):
        leg=table.find(f"geom[@name='practice_table_leg_{i}']")
        leg.set('pos',f'{x} {y} -.35');leg.set('size','.014 .014 .335')
    carton=E.SubElement(world,'body',name='fold_carton',pos=' '.join(map(str,CENTER)))
    def geom(parent,**attrs):
        return E.SubElement(parent,'geom',contype='0',conaffinity='0',material='cardboard',**attrs)
    wall=.0035
    geom(carton,name='carton_floor',type='box',size=f'{BOX.width/2} {BOX.length/2} {wall/2}',pos=f'0 0 {-BOX.height+wall/2}')
    for name,(hinge,inward,along,span,label) in FLAPS.items():
        geom(carton,name='carton_wall_'+name,type='box',size=(f'{BOX.width/2} {wall/2} {BOX.height/2}' if name in ('left','right') else f'{wall/2} {BOX.length/2} {BOX.height/2}'),pos=f'{hinge[0]} {hinge[1]} {-BOX.height/2}')
        panel=E.SubElement(carton,'body',name='fold_'+name,pos=' '.join(map(str,hinge)))
        # z rotated toward inward: axis = up cross inward.
        axis=(-inward[1],inward[0],0.)
        E.SubElement(panel,'joint',name='fold_hinge_'+name,type='hinge',axis=' '.join(map(str,axis)),range=f'{math.radians(-60)} {math.radians(125)}',limited='true')
        geom(panel,name='fold_panel_'+name,type='box',size=(f'{span/2} {wall/2} {BOX.flap/2}' if name in ('left','right') else f'{wall/2} {span/2} {BOX.flap/2}'),pos=f'0 0 {BOX.flap/2}',mass='.012')
        path=Path(cache)/(label+'.png')
        im=Image.new('RGB',(256,256),'#f5eee0');d=ImageDraw.Draw(im)
        d.rectangle((4,4,251,251),outline='#20252a',width=10)
        try:font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Bold.ttf',90)
        except OSError:font=ImageFont.load_default(size=70)
        d.text((128,128),label,font=font,anchor='mm',fill='#20252a');im.save(path)
        E.SubElement(asset,'texture',name='carton_label_'+name,type='2d',file=str(path.resolve()))
        E.SubElement(asset,'material',name='carton_label_'+name,texture='carton_label_'+name,texuniform='false',specular='.03')
        # Readable identifier on the outside, not an AprilTag detection/calibration fixture.
        E.SubElement(panel,'geom',name='fold_label_'+name,type='box',size='.018 .018 .0007',pos=f'{-inward[0]*.0025} {-inward[1]*.0025} .113',quat=('0.7071 0.7071 0 0' if name in ('left','right') else '0.7071 0 0.7071 0'),material='carton_label_'+name,contype='0',conaffinity='0')
        E.SubElement(panel,'geom',name='fold_target_'+name,type='sphere',size='.008',pos='0 0 .120',rgba='1 .98 .88 1',contype='0',conaffinity='0')
        for i,angle in enumerate(range(0,111,10)):
            a=math.radians(angle);point=[CENTER[j]+hinge[j]+inward[j]*.12*math.sin(a)+(j==2)*.12*math.cos(a) for j in range(3)]
            E.SubElement(world,'geom',name=f'fold_arc_{name}_{i}',type='sphere',size='.0028',pos=' '.join(map(str,point)),rgba='1 .96 .78 .65',contype='0',conaffinity='0')
    # Backlit bench lamp, as seen in the latest carton benchmark.
    E.SubElement(world,'geom',type='cylinder',size='.022 .14',pos='-.61 -.16 .84',rgba='.12 .13 .14 1',contype='0',conaffinity='0')
    E.SubElement(world,'geom',type='sphere',size='.032',pos='-.61 -.16 1.01',rgba='1 .97 .84 1',contype='0',conaffinity='0')
