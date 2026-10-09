"""Paired MuJoCo perception-only checks through the staged pilot's sense(tags).

No training, physical robot client, motor dispatch or fold controller. This is
an idealized tag visibility fixture, NOT the station-refit candidate preview.
"""
from __future__ import annotations

import argparse
import base64
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
from PIL import Image, ImageDraw

from farm.perception.carton_tags import CartonTagRobot, TOOL_NAME
from simulate_gemma_tags import add_marker, camera_payload, words

VIEWS = {
    'oak35': ((0, -.53, .808 + .53*math.tan(math.radians(35))), (0, 0, .808), 39.2),
    'oak58': ((0, -.53, .808 + .53*math.tan(math.radians(58))), (0, 0, .808), 39.2),
    'right_detail': ((.43, -.05, .96), (.19, .015, .86), 60),
}


def scene(tagged, yaw, flap_angle):
    root = ET.Element('mujoco', model='carton_tag_visibility_only')
    visual = ET.SubElement(root, 'visual')
    ET.SubElement(visual, 'global', offwidth='640', offheight='480')
    ET.SubElement(visual, 'headlight', ambient='.8 .8 .8', diffuse='.2 .2 .2', specular='0 0 0')
    world = ET.SubElement(root, 'worldbody')
    def board(parent, name, pos, size, rgba='.72 .54 .34 1'):
        return ET.SubElement(parent, 'geom', name=name, type='box', pos=words(pos), size=words(size),
                             rgba=rgba, contype='0', conaffinity='0')
    board(world, 'table', (0, 0, .685), (.6, .6, .015), '.45 .45 .45 1')
    half_y, half_x, h, flap = .283/2, .379/2, .108, .140
    box = ET.SubElement(world, 'body', name='carton', pos='0 0 .7', euler=f'0 0 {yaw}')
    board(box, 'floor', (0, 0, .002), (half_x, half_y, .002))
    board(box, 'near', (0, -half_y, h/2), (half_x, .0015, h/2))
    board(box, 'far', (0, half_y, h/2), (half_x, .0015, h/2))
    for side in [-1, 1]: board(box, f'side{side}', (side*half_x, 0, h/2), (.0015, half_y, h/2))
    def marker(parent, tag_id, pos, xyaxes, size=.045):
        if tagged: add_marker(parent, f'tag{tag_id}', tag_id, size, pos, xyaxes)
    for tag, x in [(26, -.12), (10, 0), (27, .12)]:
        marker(box, tag, (x, -half_y-.0018, h/2), (1, 0, 0, 0, 0, 1))
    for tag, y in [(21, .04), (28, -.08)]:
        marker(box, tag, (-half_x-.0018, y, h/2), (0, -1, 0, 0, 0, 1))
    marker(box, 22, (half_x+.0018, .04, h/2), (0, 1, 0, 0, 0, 1))
    for tag, x in [(25, 0), (24, .08)]: marker(box, tag, (x, 0, .0043), (1, 0, 0, 0, 1, 0))
    # Each flap is one rigid panel for this visibility fixture. It does not model creasing/contact.
    for tag, pos, axis, right, offset, width in [
        (11, (-half_x,0,h), (0,1,0), (0,-1,0), -.07, half_y),
        (12, (half_x,0,h), (0,-1,0), (0,1,0), .07, half_y),
        (13, (0,half_y,h+.0035), (1,0,0), (-1,0,0), .08, half_x),
        (14, (0,-half_y,h+.0035), (-1,0,0), (1,0,0), .08, half_x),
    ]:
        angle = math.radians(flap_angle if tag == 12 else 0)/2
        body = ET.SubElement(box, 'body', name=f'flap{tag}', pos=words(pos),
                             quat=words([math.cos(angle), *(np.array(axis)*math.sin(angle))]))
        # Printed +x axis = right, +y = up; the board's thin dimension is its normal.
        panel = ET.SubElement(body, 'body', name=f'panel{tag}', xyaxes=words([*right, 0,0,1]))
        board(panel, f'board{tag}', (0, flap/2, 0), (width-.003, flap/2, .0015))
        marker(panel, tag, (offset, .09, .0018), (1,0,0,0,1,0), .035)
    for name, (pos, target, fovy) in VIEWS.items():
        back = np.array(pos)-target; back /= np.linalg.norm(back)
        right = np.cross([0,0,1], back); right /= np.linalg.norm(right)
        up = np.cross(back, right)
        ET.SubElement(world, 'camera', name=name, pos=words(pos), xyaxes=words([*right,*up]), fovy=str(fovy))
    return ET.tostring(root, encoding='unicode')


class FrameRobot:
    """An image provider with no motor API or credentials."""
    def __init__(self, payload): self.payload, self.calls = payload, []
    def catalog(self):
        return {'tools': [{'type': 'function', 'function': {'name':'robot_get_cameras', 'parameters': {
            'type':'object', 'properties':{'cameras':{'type':'array','items':{'type':'string','enum':list(VIEWS)}},
                                         'revive':{'type':'boolean'}}}}}]}
    def call(self, name, args, request_id=None):
        if name != 'robot_get_cameras' or args.get('revive') is not False:
            raise AssertionError('Perception fixture only permits camera reads without revival')
        self.calls.append({'name':name, 'args':args})
        return self.payload


def run(pilot, out):
    sys.path.insert(0, str(pilot.resolve()))
    from chat_server import Chat, parse_decision
    from model_backend import DECISION_SCHEMA
    assert 'tags' in DECISION_SCHEMA['properties']['what']['items']['enum']
    assert parse_decision(json.dumps({'action':'sense', 'what':['tags']}))['what'] == ['tags']
    out.mkdir(parents=True, exist_ok=True)
    records, tiles = [], []
    for yaw in (-8,0,8):
        for angle in (0,45,100):
            for tagged in (False,True):
                stem=f'yaw{yaw}-flap{angle}-tags{int(tagged)}'
                xml=scene(tagged,yaw,angle)
                (out/f'{stem}.xml').write_text(xml)
                model=mujoco.MjModel.from_xml_string(xml); data=mujoco.MjData(model)
                mujoco.mj_forward(model,data)
                payload={'ok':True,'result':{'cameras':{}},'images':[]}
                with mujoco.Renderer(model, height=360, width=640) as renderer:
                    for view, (_,_,fovy) in VIEWS.items():
                        renderer.update_scene(data,camera=view)
                        rgb=renderer.render().copy()
                        frame=camera_payload(rgb,1,stem,1000,fovy,view)
                        payload['result']['cameras'].update(frame['result']['cameras'])
                        payload['images'] += frame['images']
                        Image.fromarray(rgb).save(out/f'{stem}-{view}.png')
                raw=FrameRobot(payload); robot=CartonTagRobot(raw,clock=lambda:1000.1)
                result=robot.call(TOOL_NAME, {'include_images':True})
                for im in result.pop('images'):
                    (out/f'{stem}-{im["view"]}-detected.jpg').write_bytes(base64.b64decode(im['data_base64']))
                chat=Chat.__new__(Chat); chat.robot=robot
                text=chat.sense(['tags'],lambda event:None)
                (out/f'{stem}-sense.txt').write_text(text)
                (out/f'{stem}-result.json').write_text(json.dumps(result,indent=2))
                ids={view:[t['tag_id'] for t in row['tags'] if t['status']=='DETECTED']
                     for view,row in result['result']['observations'].items()}
                assert tagged or not any(ids.values()), 'False detection on untagged carton'
                assert not result['result']['physical_task_completed']
                records.append({'yaw_deg':yaw,'flap_angle_deg':angle,'tagged':tagged,'ids':ids,'stem':stem,
                                'calls':raw.calls,'observation_id':result['result']['observation_id']})
                if yaw == 0 and angle == 0:
                    for view in VIEWS:
                        tile=Image.new('RGB',(640,406),'white')
                        tile.paste(Image.open(out/f'{stem}-{view}-detected.jpg'),(0,46))
                        ImageDraw.Draw(tile).text((10,8),f'PERCEPTION FIXTURE | {view} | tags {"ON" if tagged else "OFF"}\nAccepted IDs {ids[view]} | synthetic view, not station calibration',fill='black')
                        tiles.append(tile)
    montage=Image.new('RGB',(640*3,406*2),'white')
    for i,tile in enumerate(tiles): montage.paste(tile,((i%3)*640,(i//3)*406))
    montage.save(out/'comparison.png')
    summary={'scope':'Idealized perception-only, no training or fold policy; camera poses are synthetic.',
             'table_top_m':.7,'box_m':[.379,.283,.108],'views':VIEWS,'cases':len(records),
             'rendered_frames':len(records)*len(VIEWS),'motor_commands':0,
             'tagged_cases_with_detection':sum(r['tagged'] and any(r['ids'].values()) for r in records),
             'untagged_false_detections':sum(not r['tagged'] and any(r['ids'].values()) for r in records),
             'records':records}
    (out/'results.json').write_text(json.dumps(summary,indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k not in ('records','views')},indent=2))


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pilot',type=Path,required=True)
    parser.add_argument('--out',type=Path,required=True)
    a=parser.parse_args();run(a.pilot,a.out)
