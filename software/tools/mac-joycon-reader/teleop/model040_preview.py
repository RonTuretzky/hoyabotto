"""0.4 kit-layout preview: two verified SO-101 arms, two drive wheels and OAK.

SO-101 meshes and joint frames come from the pinned maker URDF. The common IKEA
cart shell is retained from the old model; mounting positions and encoder zeroes
remain preview assumptions, never physical calibration or collision evidence.
"""
import copy
import hashlib
import math
import os
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path
import mujoco
from mujoco_simulator import MujocoBus, DEFAULT_MODEL

SOFTWARE=Path(__file__).resolve().parents[3]
if str(SOFTWARE) not in sys.path:sys.path.insert(0,str(SOFTWARE))
from farm.kinematics.assets import verified_model
DEFAULT_SO101=SOFTWARE.parent/'.context/joycon-readiness/so101-model'


def scene040(path, so101_directory):
    _,manifest=verified_model(so101_directory)
    urdf=ET.parse(Path(so101_directory)/manifest['urdf']).getroot()
    for mesh in urdf.iter('mesh'):
        mesh.set('filename',str((Path(so101_directory)/mesh.get('filename')).resolve()))
    ET.SubElement(ET.SubElement(urdf,'mujoco'),'compiler',discardvisual='false')
    with tempfile.TemporaryDirectory() as directory:
        source=Path(directory)/'arm.urdf';converted=Path(directory)/'arm.xml'
        ET.ElementTree(urdf).write(source)
        arm_model=mujoco.MjModel.from_xml_path(str(source))
        mujoco.mj_saveLastXML(str(converted),arm_model)
        arm=ET.parse(converted).getroot()
    root=ET.parse(path).getroot();root.set('model','XLeRobot_040_SO101_kit_preview')
    root.find('compiler').set('meshdir',str(path.parent/'assets'))
    chassis=root.find("./worldbody/body[@name='chassis']")
    for body in list(chassis):
        if body.tag=='freejoint' or body.tag=='body' and body.get('name') in ('Base','Base_2'):
            chassis.remove(body)
    # Remove the old USB head-camera visual and show the current OAK as a schematic.
    camera=chassis.find(".//body[@name='head_camera_link']")
    if camera is not None:
        for child in list(camera):
            if child.tag in ('geom','body','camera','site'):camera.remove(child)
        ET.SubElement(camera,'geom',type='box',size='.010 .045 .012',rgba='.12 .12 .13 1',contype='0',conaffinity='0',group='2')
        for y in (-.031,0,.031):
            ET.SubElement(camera,'geom',type='sphere',size='.005',pos=f'.011 {y} 0',rgba='.25 .28 .30 1',contype='0',conaffinity='0',group='2')
    # Drop every unreferenced SO-100 arm mesh. Keep only the shared cart/head shell.
    used={g.get('mesh') for g in chassis.iter('geom') if g.get('mesh')}
    asset=root.find('asset')
    for mesh in list(asset):
        if mesh.tag=='mesh' and mesh.get('name') not in used:asset.remove(mesh)
    for mesh in arm.find('asset'):asset.append(copy.deepcopy(mesh))
    for side,sign in (('left',1),('right',-1)):
        mount=ET.SubElement(chassis,'body',name=side+'_so101_mount',pos=f'0 {sign*.11} .41',euler='0 0 3.141592653589793')
        for child in arm.find('worldbody'):
            item=copy.deepcopy(child)
            for node in list(item.iter('geom')):
                if node.get('group')!='1':
                    for parent in item.iter():
                        if node in list(parent):parent.remove(node);break
                else:node.set('group','2')
            if item.tag=='geom' and item.get('group')!='2':continue
            for node in item.iter():
                if node.get('name'):node.set('name',side+'_'+node.get('name'))
            mount.append(item)
    # This renderer is kinematic only: no controllers, tendons, sensors or physics.
    for name in ('actuator','tendon','sensor','keyframe','contact'):
        child=root.find(name)
        if child is not None:root.remove(child)
    for child in list(root.find('worldbody')):
        if child is not chassis:root.find('worldbody').remove(child)
    ET.SubElement(root.find('worldbody'),'geom',name='preview_floor',type='plane',size='6 6 .1',rgba='.28 .32 .37 1',contype='0',conaffinity='0')
    visual=ET.SubElement(root,'visual');ET.SubElement(visual,'global',offwidth='960',offheight='640')
    ET.SubElement(visual,'headlight',ambient='.6 .6 .6',diffuse='.7 .7 .7')
    return ET.tostring(root,encoding='unicode'),manifest


class Model040PreviewBus(MujocoBus):
    def __init__(self,model_path=None,*,render=True,forward_arms=True):
        path=Path(model_path or DEFAULT_MODEL).resolve()
        directory=Path(os.environ.get('XLEROBOT_SO101_MODEL',DEFAULT_SO101)).resolve()
        xml,self.arm_manifest=scene040(path,directory)
        self.model=mujoco.MjModel.from_xml_string(xml);self.data=mujoco.MjData(self.model)
        self.model_sha256=hashlib.sha256(xml.encode()).hexdigest();self.map={}
        for side in ('left','right'):
            for name in ('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper'):
                joint=self.model.joint(side+'_'+name);lo,hi=self.model.jnt_range[joint.id]
                self.map[side+'_arm_'+name]=(None,int(joint.qposadr[0]),int(joint.dofadr[0]),float(lo),float(hi))
        for name,joint_name in (('head_motor_1','head_pan_joint'),('head_motor_2','head_tilt_joint')):
            joint=self.model.joint(joint_name);lo,hi=self.model.jnt_range[joint.id]
            self.map[name]=(None,int(joint.qposadr[0]),int(joint.dofadr[0]),float(lo),float(hi))
        self.chassis=self.model.body('chassis').id
        self.render_enabled=render;self.renderer=None;self.latest_jpeg=None
        self.frame_lock=threading.Lock();self.render_error=None;self.frame_sequence=0;self.last_render=0.
        self.view='orbit';self.pending_view=None;self.render_interval=.1
        mujoco.mj_forward(self.model,self.data)

    def pose_ticks(self,reference,name,ticks):
        c=reference.calibration[name]
        if name.endswith('gripper'):
            lo,hi=self.map[name][3:]
            return lo+reference.from_ticks(name,ticks)/100*(hi-lo)
        sign=-1 if c['drive_mode'] else 1
        return sign*(ticks-(c['range_min']+c['range_max'])/2)*2*math.pi/4096
