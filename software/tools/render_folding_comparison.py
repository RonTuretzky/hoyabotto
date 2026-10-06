"""Render complete recorded physics runs, with explicit stopped-run labels."""
import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path
import textwrap
import xml.etree.ElementTree as E

import mujoco
import numpy as np
from PIL import Image,ImageDraw,ImageFont


def font(size):
    for path in ('/System/Library/Fonts/Supplemental/Arial.ttf','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'):
        if Path(path).exists():return ImageFont.truetype(path,size)
    return ImageFont.load_default(size=size)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--comparison',required=True);p.add_argument('--case',required=True)
    a=p.parse_args();root=Path(a.comparison);items=[]
    for tool in ('claws','paddle'):
        folder=root/(a.case+'-'+tool)
        # Presentation-only sky; no physical geometry or recorded state changes.
        xml=E.parse(folder/'scene.xml');asset=xml.getroot().find('asset')
        E.SubElement(asset,'texture',type='skybox',builtin='gradient',rgb1='.66 .73 .81',rgb2='.94 .95 .96',width='256',height='1536')
        presentation=folder/'presentation-scene.xml';xml.write(presentation,encoding='unicode')
        m=mujoco.MjModel.from_xml_path(str(presentation));d=mujoco.MjData(m)
        states=json.loads((folder/'folding-frames.json').read_text())
        result=json.loads((folder/'result.json').read_text())
        camera=mujoco.MjvCamera();camera.lookat[:]=[0,.22,-.15];camera.distance=2.3;camera.azimuth=130;camera.elevation=-25
        items.append(dict(model=m,data=d,renderer=mujoco.Renderer(m,360,640),camera=camera,
            states=states,times=[s['time'] for s in states],result=result,tool=tool))
    end=max(i['times'][-1] for i in items)
    timeline=sorted(set([round(t,7) for t in np.arange(0,end,.2)]+[end]))
    option=mujoco.MjvOption();option.geomgroup[3]=0
    frames=[];heading=font(22);body=font(17);small=font(15)
    for t in timeline:
        im=Image.new('RGB',(1280,764),'#f1f2f3');draw=ImageDraw.Draw(im)
        draw.text((14,8),a.case.upper().replace('-',' ')+' | OFFLINE SIMULATION | 1x time | whole robot + close view',font=heading,fill='#111')
        for col,item in enumerate(items):
            x=col*640;states=item['states'];result=item['result'];d=item['data'];m=item['model'];r=item['renderer']
            st=states[max(0,bisect_right(item['times'],t+1e-7)-1)]
            d.qpos[:]=st['qpos'];mujoco.mj_forward(m,d)
            title='TWO BARE CLAWS' if col==0 else 'LEFT CLAW + RIGHT PADDLE (FREE GRIP)'
            draw.text((x+14,40),title,font=body,fill='#111')
            r.update_scene(d,camera=item['camera'],scene_option=option)
            im.paste(Image.fromarray(r.render().copy()),(x,64))
            r.update_scene(d,camera='front',scene_option=option)
            im.paste(Image.fromarray(r.render().copy()).resize((400,225),Image.Resampling.LANCZOS),(x,427))
            reached=t>=item['times'][-1]-1e-6
            status=('PASSED' if result['success'] else 'STOPPED') if reached else 'RUNNING'
            draw.text((x+413,438),status,font=heading,fill='#111')
            draw.text((x+413,469),f"t = {min(t,item['times'][-1]):.2f} s",font=body,fill='#111')
            if reached:
                message=('Closed; released; retained for 5 s.' if result['success'] else result['controller'].get('error','Closure failed'))
            else:message=st['label']
            for line_no,line in enumerate(textwrap.wrap(message,width=24)):
                draw.text((x+413,500+line_no*20),line,font=small,fill='#111')
            label=st['label'] if not reached else ('Final verified closure' if result['success'] else 'Final stopped state held on screen')
            for n,line in enumerate(textwrap.wrap(label,width=76)):
                draw.text((x+12,661+n*20),line,font=small,fill='#111')
            if reached:
                draw.text((x+12,704),f"Max box motion (3D): {result['physics']['carton_motion']['max_translation_mm']:.1f} mm",font=small,fill='#111')
        draw.line((639,36,639,728),fill='#555',width=2)
        material=items[0]['result']['assumptions']['material']
        draw.text((14,740),f"UNMEASURED: contents {material['contents_mass_kg']*1000:.0f} g; friction {material['table_friction']}; crease {material['hinge_stiffness']} Nm/rad. Unbolted box. No hardware. Pickup not tested.",font=small,fill='#111')
        frames.append(im)
    durations=[max(10,round((b-a)*1000/10)*10) for a,b in zip(timeline,timeline[1:])]+[2500]
    target=root/(a.case+'-full-comparison.gif')
    frames[0].save(target,save_all=True,append_images=frames[1:],duration=durations,loop=0,optimize=True)
    for name,index in [('start',0),('middle',len(frames)//2),('end',-1)]:frames[index].save(root/(a.case+'-'+name+'.png'))
    check=Image.open(target);actual_duration=0
    for i in range(check.n_frames):check.seek(i);check.load();actual_duration+=check.info.get('duration',0)
    metadata=dict(path=str(target),simulation_end_seconds=end,playback_seconds=actual_duration/1000,
                  frames=check.n_frames,size=check.size,complete_recorded_timeline=True,
                  stopped_runs_held_with_labels=True,source_frames={i['tool']:len(i['states']) for i in items})
    target.with_suffix('.json').write_text(json.dumps(metadata,indent=2))
    for item in items:item['renderer'].close()
    print(json.dumps(metadata),flush=True)


if __name__=='__main__':main()
