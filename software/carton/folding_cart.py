"""Fixed cart collision approximation reused from the local paddle simulator.

The earlier Molmo scene used a 500 x 360 mm cart with three trays. Dimensions,
mount offsets and registration are assumptions, not a scan of the user's cart.
All coordinates below are relative to the arm-base line, facing world +Y.
"""
from __future__ import annotations
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class CartBox:
    name: str
    center: tuple[float, float, float]
    half_size: tuple[float, float, float]


def cart_boxes(station):
    """The same tray/post dimensions as molmoact2-mac-sim/build_scene.py."""
    boxes=[]
    def add(name,local,half):
        boxes.append(CartBox('cart_'+name,
            (local[0],station.base_y+local[1],station.base_height+local[2]),tuple(half)))
    cy=-.065
    for i,level in enumerate((-.13,-.41,-.70)):
        add(f'tray_{i}',(0,cy,level),(.25,.18,.018))
        for sign in (-1,1):
            add(f'rim_front_back_{i}_{sign}',(0,cy+sign*.17,level+.045),(.25,.01,.06))
            add(f'rim_side_{i}_{sign}',(sign*.24,cy,level+.045),(.01,.16,.06))
    for x in (-.23,.23):
        for y in (cy-.16,cy+.16):
            add(f'post_{x}_{y}',(x,y,-.39),(.012,.012,.36))
    for side,sign in (('left',-1),('right',1)):
        x=sign*station.base_spacing/2
        add(f'{side}_mount_plate',(x,.016,-.0125),(.065,.06,.01))
        add(f'{side}_mount_support',(x,-.01,-.06725),(.025,.025,.04475))
    return boxes


def table_overlap(station,boxes=None):
    """Static world bodies share a weld in MuJoCo; check their overlap explicitly."""
    width,depth=station.table_size
    table_lo=np.array([-width/2,station.table_edge_y,-.032])
    table_hi=np.array([width/2,station.table_edge_y+depth,0.])
    overlaps=[]
    for box in cart_boxes(station) if boxes is None else boxes:
        lo=np.asarray(box.center)-box.half_size
        hi=np.asarray(box.center)+box.half_size
        depth=np.minimum(hi,table_hi)-np.maximum(lo,table_lo)
        if np.all(depth>1e-6):
            overlaps.append({'part':box.name,'overlap_xyz_mm':(depth*1000).tolist()})
    return overlaps


def report(station):
    return {'model':'three-tray fixed cart approximation',
            'source':'local molmoact2-mac-sim/build_scene.py tray/post/mount dimensions',
            'dimensions_measured':False,'collision_enabled':True,
            'width_m':.50,'depth_m':.36,'front_edge_ahead_of_base_line_m':.115,
            'front_edge_to_table_edge_m':station.base_to_table_edge-.115,
            'static_table_overlaps':table_overlap(station)}
