"""Offline station review. No policy training or hardware clients.

Candidate geometry, never an implicit physical calibration. Render projection uses
an oversized centered pinhole view remapped to the supplied OAK intrinsics. This
matches cv2.undistort(..., newCameraMatrix=K), not the raw distorted ISP image.
"""
from pathlib import Path
import copy
import json
import math
import xml.etree.ElementTree as ET

import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from carton.folding_sim import FoldingSimulation, JOINTS, words
from carton.folding_station import FoldingStation
from carton.folding_station_measured import axes_from_rotation_cv, look_at_axes
from carton.xlerobot_cameras import head_camera_spec, wrist_camera_spec, BASE_PLANE_Z
from carton.folding_diagonal import contact_point


def replace_cart(root, upstream, station, out, tilt=35.145, pan=-5.132):
    """Bake source cart/head at its CAD pose; retain source obstacle geometry."""
    m=mujoco.MjModel.from_xml_path(str(upstream));d=mujoco.MjData(m)
    for name,value in [('head_tilt_joint',tilt),('head_pan_joint',pan)]:
        d.qpos[m.joint(name).qposadr[0]]=math.radians(value)
    mujoco.mj_forward(m,d)
    asset=root.find('asset');world=root.find('worldbody')
    for geom in list(world.findall('geom')):
        if geom.get('name','').startswith('cart_'):world.remove(geom)
    blocked={m.body('Base').id,m.body('Base_2').id}
    def arm(bid):
        while bid:
            if bid in blocked:return True
            bid=int(m.body_parentid[bid])
        return False
    T=np.array([[0.,1,0],[-1,0,0],[0,0,1]])
    # Align the CAD mounting plane with the measured arm mounting plane.
    # Desktop/floor coordinates stay fixed; the old -0.7 assumed +29.1 mm mounts.
    shift=np.array([0.,station.base_y-.09,station.base_height-BASE_PLANE_Z])
    out.mkdir(parents=True,exist_ok=True)
    rows=[]
    for g in range(m.ngeom):
        if arm(int(m.geom_bodyid[g])):continue
        name=f'cart_cad_{g}'
        collides=bool(m.geom_contype[g] or m.geom_conaffinity[g])
        color=m.mat_rgba[m.geom_matid[g]] if m.geom_matid[g]>=0 else m.geom_rgba[g]
        attrs=dict(name=name,group='3' if collides else '1',
                   contype='1' if collides else '0',conaffinity='1' if collides else '0',
                   rgba=words(color),mass='0')
        typ=int(m.geom_type[g])
        R=T@d.geom_xmat[g].reshape(3,3);pos=T@d.geom_xpos[g]+shift
        if typ==int(mujoco.mjtGeom.mjGEOM_MESH):
            mid=int(m.geom_dataid[g]);start=m.mesh_vertadr[mid];count=m.mesh_vertnum[mid]
            vertices=np.einsum('ij,kj->ik',m.mesh_vert[start:start+count],R)+pos
            start=m.mesh_faceadr[mid];count=m.mesh_facenum[mid];faces=m.mesh_face[start:start+count]
            path=out/f'{name}.obj'
            path.write_text(''.join('v %.10g %.10g %.10g\n'%tuple(v) for v in vertices)+
                            ''.join('f %d %d %d\n'%tuple(f+1) for f in faces))
            ET.SubElement(asset,'mesh',name=name,file=str(path.resolve()))
            attrs.update(type='mesh',mesh=name)
        else:
            types={int(mujoco.mjtGeom.mjGEOM_BOX):'box',int(mujoco.mjtGeom.mjGEOM_SPHERE):'sphere',
                   int(mujoco.mjtGeom.mjGEOM_CYLINDER):'cylinder'}
            if typ not in types:raise ValueError(f'Unsupported cart geom {typ}')
            quat=np.zeros(4);mujoco.mju_mat2Quat(quat,R.ravel())
            attrs.update(type=types[typ],size=words(m.geom_size[g]),pos=words(pos),quat=words(quat))
        ET.SubElement(world,'geom',**attrs)
        rows.append(dict(name=name,source_body=m.body(int(m.geom_bodyid[g])).name,collides=collides))
    return rows


def configure_scene(sim,upstream,out, *, controller_anchor=False):
    root=ET.parse(out/'scene.xml').getroot();world=root.find('worldbody');st=sim.station
    ET.SubElement(root.find('asset'),'texture',name='refit_sky',type='skybox',builtin='gradient',
                  rgb1='.20 .24 .28',rgb2='.72 .76 .79',width='512',height='3072')
    cart=replace_cart(root,upstream,st,out/'cart-assets')
    for geom in world.findall('geom'):
        if geom.get('name','').startswith('cart_cad_') and geom.get('group')=='1':
            geom.set('rgba','.10 .11 .12 1')
    ET.SubElement(world,'geom',name='floor',type='plane',size='2 2 .01',pos='0 0 -.7',
                  rgba='.22 .24 .26 1',contype='1',conaffinity='1')
    # Desk legs are schematic: no measured folded-tube geometry is available.
    for x in (-.205,.205):
        for y1,y2 in ((.035,.43),(.43,.035)):
            ET.SubElement(world,'geom',name=f'desk_leg_{x}_{y1}',type='capsule',size='.008',
                          fromto=words([x,st.table_edge_y+y1,-.035,x,st.table_edge_y+y2,-.69]),
                          rgba='.12 .13 .14 1',contype='1',conaffinity='1')
    world.find("geom[@name='table']").set('rgba','.22 .14 .08 1')
    # Synthetic table anchor is not present in the physical capture.
    anchor=world.find("body[@name='table_tag']")
    if controller_anchor:
        # Synthetic teacher-only registration aid. Hide group 4 when rendering
        # policy observations; it is not part of the physical station.
        for geom in anchor.iter('geom'):geom.set('group','4')
    else:
        world.remove(anchor)
    for side in ('left','right'):
        base=world.find(f"body[@name='{side}_base_link']")
        for geom in base.iter('geom'):
            if geom.get('name','').endswith('_visual') or geom.get('contype')!='0':continue
            if 'tag' not in geom.get('name',''):geom.set('rgba','.055 .06 .065 1')
        spec=wrist_camera_spec(side);r,u=axes_from_rotation_cv(spec['rotation_cv'])
        ET.SubElement(base.find(f".//body[@name='{side}_gripper_link']"),'camera',name=side+'_wrist',
                      pos=words(spec['position_m']),xyaxes=words(np.r_[r,u]),fovy=str(spec['fovy_deg']))
    spec=head_camera_spec(35.145,-5.132);r,u=axes_from_rotation_cv(spec['rotation_cv'])
    front=world.find("camera[@name='front']")
    front.set('pos',words(np.array(spec['position_m'])+[0,st.base_y,st.base_height]))
    front.set('xyaxes',words(np.r_[r,u]))
    views={
      'overall':([1.15,st.base_y-1.4,1.0],[0,-.07,-.12],46),
      'profile':([1.45,st.base_y+.08,.12],[0,st.base_y+.08,-.08],48),
      'plan':([0,-.03,1.15],[0,-.03,0],50),
      'work':([.63,st.base_y-.64,.58],[0,-.07,.15],44),
    }
    for name,(pos,look,fovy) in views.items():
        r,u=look_at_axes(pos,look)
        ET.SubElement(world,'camera',name=name,pos=words(pos),xyaxes=words(np.r_[r,u]),fovy=str(fovy))
    ET.indent(root);ET.ElementTree(root).write(out/'scene.xml',encoding='unicode')
    q=sim.data.qpos.copy();ctrl=sim.data.ctrl.copy()
    sim.model=mujoco.MjModel.from_xml_path(str(out/'scene.xml'))
    sim.data=mujoco.MjData(sim.model);sim.kin=mujoco.MjData(sim.model)
    sim.data.qpos[:]=q;sim.data.ctrl[:]=ctrl;mujoco.mj_forward(sim.model,sim.data)
    sim.validate_initial_robot_clearance()
    # Static bodies share a weld and never appear in data.contact.
    for gid in range(sim.model.ngeom):
        if sim.model.geom(gid).name.startswith('cart_') and sim.model.geom_contype[gid]:
            distance=mujoco.mj_geomDistance(sim.model,sim.data,gid,sim.model.geom('table').id,1.,None)
            if distance < -.0001:raise ValueError('Static CAD cart intersects the table')
    return cart,spec


def render(sim,camera,width=1000,height=750):
    with mujoco.Renderer(sim.model,height,width) as renderer:
        renderer.update_scene(sim.data,camera=camera,scene_option=sim.option)
        return renderer.render().copy()


def oak_policy_render(sim, contract=None):
    """Commissioned 320x240 policy preview, using the training pixel convention."""
    from carton.refit_camera_contract import load_contract, render_policy_camera
    contract = load_contract() if contract is None else contract
    width, height = contract['policy']['size_wh']
    with mujoco.Renderer(sim.model, height, width) as renderer:
        return render_policy_camera(renderer, sim.data, 'front', contract=contract)


def oak_render(sim,K):
    # Margin preserves rays needed for the slightly offset principal point.
    w,h=768,480
    cam=sim.model.camera('front').id
    sim.model.cam_fovy[cam]=math.degrees(2*math.atan(h/(2*K[1,1])))
    raw=render(sim,'front',w,h)
    transform=np.array([[K[0,0]/K[1,1],0,K[0,2]-w/2*K[0,0]/K[1,1]],
                        [0,1,K[1,2]-h/2]])
    return cv2.warpAffine(raw,transform,(640,360))


def camera_detail_render(sim):
    """Isolate the right end effector visually; do not change collision flags."""
    m,d=sim.model,sim.data;rgba=m.geom_rgba.copy();cam=m.camera('overall').id
    pos0=m.cam_pos[cam].copy();quat0=m.cam_quat[cam].copy()
    jaw=m.body('right_gripper_link').id
    try:
        for gid in range(m.ngeom):
            bid=int(m.geom_bodyid[gid]);keep=False
            while bid:
                if bid==jaw:keep=True;break
                bid=int(m.body_parentid[bid])
            if not keep or 'tag' in m.geom(gid).name:m.geom_rgba[gid,3]=0.
        R=d.body(jaw).xmat.reshape(3,3);origin=d.body(jaw).xpos
        center=origin+R@np.array([.0035,.035,-.033])
        eye=center+R@np.array([.18,.19,-.06]);right,up=look_at_axes(eye,center)
        rotation=np.column_stack((right,up,np.cross(right,up)))
        m.cam_pos[cam]=eye;mujoco.mju_mat2Quat(m.cam_quat[cam],rotation.ravel())
        mujoco.mj_forward(m,d)
        return render(sim,'overall',900,700)
    finally:
        m.geom_rgba[:]=rgba;m.cam_pos[cam]=pos0;m.cam_quat[cam]=quat0
        mujoco.mj_forward(m,d)


def label(image,title,detail):
    im=Image.fromarray(image);d=ImageDraw.Draw(im)
    d.rectangle((0,0,im.width,48),fill='#14242c')
    font='/System/Library/Fonts/Helvetica.ttc'
    d.text((14,5),title,fill='white',font=ImageFont.truetype(font,18))
    d.text((14,28),detail,fill='#d5dfe5',font=ImageFont.truetype(font,12))
    return im


def clearance(sim):
    m,d=sim.model,sim.data;names=[m.geom(i).name or '' for i in range(m.ngeom)]
    cams=[i for i,n in enumerate(names) if '_wrist_camera_' in n and n.endswith('_collision')]
    carton=[i for i,n in enumerate(names) if n in ('bottom','wall_left','wall_right','wall_far','wall_near') or n.endswith('_cardboard')]
    distances=[(float(mujoco.mj_geomDistance(m,d,a,b,1.,None)),names[a],names[b]) for a in cams for b in carton]
    bad=[]
    for c in d.contact:
        a,b=names[c.geom1],names[c.geom2]
        if c.dist < -.0001 and sim.forbidden_contact(a,b):bad.append([a,b,float(-c.dist*1000)])
    dist,a,b=min(distances)
    return dict(camera_to_carton_min_mm=dist*1000,closest_pair=[a,b],forbidden_penetrations=bad)


def reach_screen(sim):
    """Sparse terminal poses only; explicit orientation and clearance checks."""
    m,d=sim.model,sim.data;park=d.qpos.copy();seeds=copy.deepcopy(sim.seeds);rows=[]
    for side,sign in [('left',-1),('right',1)]:
        for degrees in (0,30,60,85):
            theta=math.radians(degrees)
            target,_=contact_point(theta,0,sign,-.10,.115,0,-.0045)
            target[2]+=.001
            direction=np.array([sign*math.cos(theta),0,math.sin(theta)])
            # Matches outside-face short-panel normal at the jaw's local +X.
            d.qpos[:]=park;sim.seeds=copy.deepcopy(seeds)
            attempts=[]
            starts=[seeds[side]]+[[sign*.5,shoulder,elbow,wrist,0.] for shoulder in (-1.,1.)
                                 for elbow in (-1.,1.) for wrist in (-1.,1.)]
            for seed in starts:
                d.qpos[:]=park;sim.seeds[side]=np.asarray(seed)
                q,error=sim.ik(side,target,dict(direction=direction.tolist(),local_axis=[1,0,0]))
                d.qpos[sim.arm_indices[side][:5]]=q
                d.qpos[m.joint('short_'+side+'_hinge').qposadr[0]]=theta
                mujoco.mj_forward(m,d)
                axis=d.body(side+'_gripper_link').xmat.reshape(3,3)[:,0]
                angle=math.degrees(math.acos(float(np.clip(axis@direction,-1,1))))
                c=clearance(sim)
                attempts.append(dict(position_error_mm=error*1000,orientation_error_deg=angle,**c,
                    joint_radians=q.tolist(),passes=bool(error<=.008 and angle<=10 and
                    not c['forbidden_penetrations'] and c['camera_to_carton_min_mm']>=1)))
            best=min(attempts,key=lambda x:(not x['passes'],x['position_error_mm']+x['orientation_error_deg']/2+
                     sum(p[2] for p in x['forbidden_penetrations'])))
            rows.append(dict(side=side,flap_degrees=degrees,target_m=target.tolist(),**best,
                             start_count=len(starts),attempts=attempts))
    d.qpos[:]=park;sim.seeds=seeds;mujoco.mj_forward(m,d)
    return rows
