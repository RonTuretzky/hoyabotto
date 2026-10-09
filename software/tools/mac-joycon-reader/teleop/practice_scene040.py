"""Workshop props for local practice; no physical scene registration implied."""
import math
import xml.etree.ElementTree as E
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont


def add_workshop(root,cache):
    cache=Path(cache);cache.mkdir(parents=True,exist_ok=True)
    asset=root.find('asset');world=root.find('worldbody')
    headlight=root.find('./visual/headlight')
    if headlight is not None:headlight.set('ambient','.35 .35 .35');headlight.set('diffuse','.40 .40 .40')
    E.SubElement(asset,'texture',name='workshop_sky',type='skybox',builtin='gradient',rgb1='.28 .34 .41',rgb2='.84 .86 .87',width='256',height='1536')
    E.SubElement(asset,'texture',name='workshop_grid',type='2d',builtin='checker',rgb1='.30 .34 .38',rgb2='.37 .41 .45',width='256',height='256')
    E.SubElement(asset,'material',name='workshop_floor',texture='workshop_grid',texrepeat='12 12',reflectance='.04',shininess='.15')
    floor=world.find("geom[@name='preview_floor']");floor.set('material','workshop_floor');floor.attrib.pop('rgba',None)
    E.SubElement(asset,'material',name='workshop_wood',rgba='.68 .48 .29 1',specular='.1',shininess='.15')
    E.SubElement(asset,'material',name='workshop_steel',rgba='.11 .14 .17 1',specular='.35',shininess='.35')
    E.SubElement(asset,'material',name='cardboard',rgba='.68 .44 .24 1',specular='.04',shininess='.1')
    for label in ('R','P'):
        path=cache/(label+'.png')
        if not path.exists():
            im=Image.new('RGB',(256,256),'#f7f4e9');d=ImageDraw.Draw(im)
            d.rounded_rectangle((5,5,251,251),radius=26,outline='#151b20',width=12)
            try:font=ImageFont.truetype('/System/Library/Fonts/Supplemental/Arial Bold.ttf',160)
            except OSError:font=ImageFont.load_default(size=130)
            box=d.textbbox((0,0),label,font=font);d.text(((256-box[2])/2,(256-box[3])/2-5),label,font=font,fill='#151b20')
            im.save(path)
        E.SubElement(asset,'texture',name='label_'+label,type='2d',file=str(path.resolve()))
        E.SubElement(asset,'material',name='label_'+label,texture='label_'+label,texuniform='false',texrepeat='1 1',rgba='1 1 1 1',specular='.03')
    table=E.SubElement(world,'body',name='practice_table',pos='-.45 0 .85')
    E.SubElement(table,'geom',name='practice_table_top',type='box',size='.28 .34 .025',material='workshop_wood',contype='0',conaffinity='0')
    for i,(x,y) in enumerate(((-.23,-.29),(-.23,.29),(.23,-.29),(.23,.29))):
        E.SubElement(table,'geom',name='practice_table_leg_'+str(i),type='box',size='.015 .015 .42',pos=f'{x} {y} -.445',material='workshop_steel',contype='0',conaffinity='0')
    box=E.SubElement(world,'body',name='exercise_box',pos='-.35 .1 .94')
    E.SubElement(box,'geom',type='box',size='.035 .025 .025',material='cardboard',contype='0',conaffinity='0')
    E.SubElement(box,'geom',type='box',size='.006 .0252 .0252',rgba='.85 .73 .49 1',contype='0',conaffinity='0')
    E.SubElement(box,'geom',type='box',size='.017 .017 .0008',pos='0 0 .0258',material='label_R',contype='0',conaffinity='0')
    E.SubElement(box,'geom',type='cylinder',size='.006 .006',pos='0 0 .031',rgba='.90 .90 .86 1',contype='0',conaffinity='0')
    E.SubElement(box,'site',name='box_grip_target',pos='0 0 .037',size='.003',rgba='1 1 1 1')
    marker=E.SubElement(world,'body',name='placement_marker',pos='-.35 -.1 .90')
    E.SubElement(marker,'geom',type='box',size='.046 .046 .0007',material='label_P',contype='0',conaffinity='0')
    # A small workshop backdrop with readable contrast and soft lighting.
    E.SubElement(world,'geom',type='box',size='.035 1.6 1.2',pos='-1.3 0 1.2',rgba='.66 .71 .74 1',contype='0',conaffinity='0')
    for y in (-.85,.85):
        E.SubElement(world,'geom',type='box',size='.04 .30 .012',pos=f'-1.23 {y} 1.10',material='workshop_steel',contype='0',conaffinity='0')
    E.SubElement(world,'light',name='workshop_key',pos='.4 -.8 2.7',dir='-.25 .1 -1',directional='true',diffuse='.42 .40 .37',ambient='.06 .06 .06',castshadow='true')
    E.SubElement(world,'light',name='workshop_fill',pos='-.9 .9 2',dir='.3 -.3 -1',directional='true',diffuse='.22 .25 .28',castshadow='false')
