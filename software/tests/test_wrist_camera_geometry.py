"""Exercise actual solver contact, including the formerly invisible module."""
import xml.etree.ElementTree as ET
import numpy as np
import pytest

mujoco = pytest.importorskip('mujoco')
from carton.wrist_camera_geometry import ASSETS, add_wrist_camera
from carton.folding_sim import FoldingSimulation


def vertices(path):
    return np.array([list(map(float,line.split()[1:])) for line in path.read_text().splitlines() if line.startswith('v ')])


def test_registered_camera_long_axis_and_side_mount_difference():
    left=np.concatenate([vertices(ASSETS/f'left-{p}.obj') for p in (1,2)])
    right=np.concatenate([vertices(ASSETS/f'right-{p}.obj') for p in (1,2)])
    assert np.ptp(right,axis=0)*1000 == pytest.approx([35.9883,67.2548,42.7833],abs=.03)
    assert right.min(0)*1000 == pytest.approx([-14.7033,22.2428,-29.0968],abs=.03)
    assert np.linalg.norm((left-right).mean(0)) == pytest.approx(.001,abs=1e-7)


def test_camera_shell_applies_force_and_displaces_a_free_object():
    displacement=[]
    for enabled in (True,False):
        root=ET.Element('mujoco')
        ET.SubElement(root,'option',gravity='0 0 0',timestep='.001')
        asset=ET.SubElement(root,'asset');world=ET.SubElement(root,'worldbody')
        jaw=ET.SubElement(world,'body',name='jaw')
        add_wrist_camera(asset,jaw,'right')
        if not enabled:
            for g in jaw.findall('geom'):
                g.set('contype','0');g.set('conaffinity','0')
        probe=ET.SubElement(world,'body',name='probe',pos='.024 .074 0')
        ET.SubElement(probe,'freejoint')
        ET.SubElement(probe,'geom',name='probe',type='sphere',size='.006',mass='.02')
        m=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        d=mujoco.MjData(m);mujoco.mj_forward(m,d)
        start=d.qpos[:3].copy();peak=0.
        for _ in range(50):
            mujoco.mj_step(m,d)
            for i,c in enumerate(d.contact):
                force=np.zeros(6);mujoco.mj_contactForce(m,d,i,force)
                peak=max(peak,float(np.linalg.norm(force[:3])))
        displacement.append(float(np.linalg.norm(d.qpos[:3]-start)))
        assert (peak>0.01) == enabled
    assert displacement[0]>.001
    assert displacement[1]==pytest.approx(0.,abs=1e-12)


def test_camera_is_forbidden_even_against_a_foldable_flap():
    check=lambda a,b:FoldingSimulation.forbidden_contact(None,a,b)
    for obstacle in ('short_right_cardboard','table','wall_right','left_gripper_link_geom_1'):
        assert check('right_wrist_camera_2_collision',obstacle)
        assert check(obstacle,'right_wrist_camera_2_collision')
