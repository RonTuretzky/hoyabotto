"""Collision-checked joint paths for offline folding experiments.

No hardware adapter. Planning snapshots come from the simulator in these
experiments; this is not a calibrated real-world obstacle model. Flaps remain
dynamic during execution, and the normal runtime collision gates still apply.
"""
import time
import mujoco
import numpy as np

from carton.folding_sim import JOINTS


class JointPathPlanner:
    def __init__(self,sim,side,*,allowed_flaps=(),seed=1):
        self.sim=sim;self.side=side;self.model=sim.model
        self.data=mujoco.MjData(self.model);self.data.qpos[:]=sim.data.qpos
        self.ix=sim.arm_indices[side][:5]
        self.limits=self.model.jnt_range[[self.model.joint(side+'_'+j).id for j in JOINTS[:5]]]
        self.allowed=set(allowed_flaps);self.rng=np.random.default_rng(seed)
        self.checks=0;self.last_collision=None
        self.tool_transform=None
        if side=='right' and mujoco.mj_name2id(self.model,mujoco.mjtObj.mjOBJ_JOINT,'paddle_free')>=0:
            # Predict the held tool's sweep in the planning copy only. Using
            # its frozen world pose would miss tool/environment collisions.
            mujoco.mj_kinematics(self.model,self.data)
            grip=self.data.body('right_gripper_link');tool=self.data.body('paddle')
            rotation=grip.xmat.reshape(3,3)
            self.tool_transform=(rotation.T@(tool.xpos-grip.xpos),
                                 rotation.T@tool.xmat.reshape(3,3))

    def valid(self,q):
        q=np.asarray(q,dtype=float)
        if q.shape!=(5,) or not np.isfinite(q).all() or np.any(q<self.limits[:,0]) or np.any(q>self.limits[:,1]):return False
        self.data.qpos[self.ix]=q
        if self.tool_transform is not None:
            mujoco.mj_kinematics(self.model,self.data)
            grip=self.data.body('right_gripper_link');rotation=grip.xmat.reshape(3,3)
            offset,relative_rotation=self.tool_transform
            adr=self.model.joint('paddle_free').qposadr[0]
            self.data.qpos[adr:adr+3]=grip.xpos+rotation@offset
            mujoco.mju_mat2Quat(self.data.qpos[adr+3:adr+7],(rotation@relative_rotation).ravel())
        mujoco.mj_fwdPosition(self.model,self.data)
        self.checks+=1
        for c in self.data.contact:
            a,b=self.model.geom(c.geom1).name,self.model.geom(c.geom2).name
            if not (a.startswith(self.side+'_') or b.startswith(self.side+'_')):continue
            if c.dist>=-.0001:continue
            other=b if a.startswith(self.side+'_') else a
            arm=a if a.startswith(self.side+'_') else b
            if other in self.allowed and any(n in arm for n in ('moving_jaw','wrist_roll_follower','right_paddle_contact')):continue
            # A held paddle is a separate free body, and its intentional jaw
            # contacts are distinguished from environmental collisions.
            if a.startswith('right_paddle_') or b.startswith('right_paddle_'):
                if not self.sim.forbidden_contact(a,b) and other not in ('short_left_cardboard','short_right_cardboard','long_near_cardboard','long_far_cardboard'):continue
            self.last_collision=(a,b,float(c.dist));return False
        return True

    def edge(self,a,b):
        steps=max(1,int(np.ceil(np.max(np.abs(b-a))/.025)))
        return all(self.valid(a+(b-a)*u) for u in np.linspace(0,1,steps+1))

    def plan(self,goal,*,max_iterations=3000,max_seconds=40.):
        start=np.clip(self.sim.data.qpos[self.ix].copy(),self.limits[:,0],self.limits[:,1]);goal=np.asarray(goal)
        if not self.valid(start):raise ValueError(f'Path start collides: {self.last_collision}')
        if not self.valid(goal):raise ValueError(f'Path goal collides: {self.last_collision}')
        if self.edge(start,goal):return [start,goal]
        trees=[([start],[-1]),([goal],[-1])];started=time.monotonic()
        def extend(tree,target):
            nodes,parents=tree
            near=int(np.argmin([np.linalg.norm(n-target) for n in nodes]));a=nodes[near]
            delta=target-a;length=np.linalg.norm(delta)
            b=target if length<=.20 else a+delta*(.20/length)
            if not self.edge(a,b):return None,False
            nodes.append(b);parents.append(near)
            return len(nodes)-1,length<=.20
        def trace(tree,index):
            nodes,parents=tree;path=[]
            while index!=-1:path.append(nodes[index]);index=parents[index]
            return path[::-1]
        for iteration in range(max_iterations):
            if time.monotonic()-started>max_seconds:break
            first=iteration%2;second=1-first
            sample=trees[second][0][0] if self.rng.random()<.15 else self.rng.uniform(self.limits[:,0],self.limits[:,1])
            index,_=extend(trees[first],sample)
            if index is None:continue
            target=trees[first][0][index]
            for _ in range(80):
                connected,reached=extend(trees[second],target)
                if connected is None:break
                if reached:
                    a=trace(trees[first],index);b=trace(trees[second],connected)
                    path=a+b[-2::-1] if first==0 else b+a[-2::-1]
                    # Deterministic shortcuts retain full edge collision checks.
                    for _ in range(60):
                        if len(path)<3:break
                        i,j=sorted(self.rng.choice(len(path),2,replace=False))
                        if j>i+1 and self.edge(path[i],path[j]):path=path[:i+1]+path[j:]
                    return path
        raise ValueError(f'No collision-free joint path in {self.checks} checks; last contact {self.last_collision}')


def execute_path(sim,side,path,label,*,capture=True):
    for i,(a,b) in enumerate(zip(path,path[1:])):
        duration=max(.35,float(np.max(np.abs(b-a)))/.55)
        event=sim.move({},duration,f'{label} waypoint {i+1}/{len(path)-1}',
                       capture=capture,joint_targets={side:b})
        if event['bad_penetration_mm']>1:raise ValueError('Collision during planned path execution')
        if event['max_joint_tracking_error_radians']>.08:
            raise ValueError('Planned joint path tracking error exceeded 0.08 rad')
