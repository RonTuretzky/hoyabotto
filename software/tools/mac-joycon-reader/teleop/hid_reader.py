#!/usr/bin/env python3
"""Original Joy-Con HID input for local simulation, using public protocol notes.

Only Nintendo 057e:2006/2007 are opened. Writes are limited to volatile report
mode/IMU configuration and read-only SPI calibration queries. No rumble,
firmware, persistent writes, Bluetooth settings, network or robot access.
"""
import argparse
import json
import math
import signal
import struct
import threading
import time
import uuid
from collections import deque


def unpack_stick(data):
    return (data[0]|((data[1]&15)<<8),(data[1]>>4)|(data[2]<<4))


def stick_calibration(raw,side):
    if len(raw)!=9 or raw==b'\xff'*9:raise ValueError('Missing stick calibration')
    a,b,c=[unpack_stick(raw[i:i+3]) for i in (0,3,6)]
    center,below,above=(b,c,a) if side=='left' else (a,b,c)
    if any(not 500<x<3600 for x in center) or any(not 200<x<3000 for x in below+above):raise ValueError('Invalid stick calibration')
    return center,below,above


def normalize_stick(raw,cal,deadzone):
    center,below,above=cal;values=[]
    for i,v in enumerate(raw):
        x=max(-1.,min(1.,(v-center[i])/(above[i] if v>=center[i] else below[i])))
        values.append({'raw':x,'filtered':math.copysign((abs(x)-deadzone)/(1-deadzone),x) if abs(x)>deadzone else 0.})
    return dict(zip(('x','y'),values))


def quaternion_product(a,b):
    w,x,y,z=a;v,i,j,k=b
    return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]


class Sensor:
    """Each controller has its own calibration, bias and orientation state."""
    def __init__(self,raw):
        if len(raw)!=24 or raw==b'\xff'*24:raise ValueError('Missing IMU calibration')
        values=struct.unpack('<12h',raw)
        self.accel_scale=[4/(values[i+3]-values[i]) for i in range(3)]
        self.offset=list(values[6:9]);self.gyro_scale=[math.radians(936/(values[i+9]-values[i+6])) for i in range(3)]
        if any(not .0001<v<.001 for v in self.accel_scale) or any(not .0005<v<.003 for v in self.gyro_scale):raise ValueError('Invalid IMU calibration')
        self.quaternion=[1.,0.,0.,0.];self.samples=deque(maxlen=200);self.accel_samples=deque(maxlen=200);self.ready=False
        self.gyro=[0.,0.,0.];self.accel=[0.,0.,0.];self.bias=[0.,0.,0.]
        self.raw=[0]*6;self.angular=[0.,0.,0.];self.calibration_status='waiting_for_samples'
        self.session=uuid.uuid4().hex

    def reset(self,status='waiting_for_samples'):
        self.ready=False;self.samples.clear();self.accel_samples.clear()
        self.quaternion=[1.,0.,0.,0.];self.gyro=[0.,0.,0.];self.bias=[0.,0.,0.]
        self.calibration_status=status;self.session=uuid.uuid4().hex

    def update(self,raw):
        vals=struct.unpack('<6h',raw)
        self.raw=list(vals)
        self.accel=[vals[i]*self.accel_scale[i] for i in range(3)]
        angular=[(vals[i+3]-self.offset[i])*self.gyro_scale[i] for i in range(3)]
        self.angular=angular
        if not any(vals):
            # Fresh button reports can still contain a disabled/invalid IMU.
            if self.ready or self.calibration_status!='no_imu_data':self.reset('no_imu_data')
            return
        if not self.ready:
            # Estimate the resting gyro offset from one stable second. Testing
            # absolute angular rate before removing that offset prevented the
            # observed stationary controllers (~0.21 rad/s bias) ever calibrating.
            # Require plausible gravity/rate AND small variation of every axis.
            gravity=math.sqrt(sum(v*v for v in self.accel))
            status='calibrating'
            if not .8<=gravity<=1.2:status='acceleration_out_of_range'
            elif max(abs(v) for v in angular)>.35:status='rotation_detected'
            if status!='calibrating':
                self.samples.clear();self.accel_samples.clear();self.calibration_status=status;return
            self.samples.append(angular);self.accel_samples.append(self.accel)
            unstable=any(max(s[i] for s in window)-min(s[i] for s in window)>limit
                         for window,limit in ((self.samples,.035),(self.accel_samples,.04)) for i in range(3))
            if unstable:
                self.samples.clear();self.accel_samples.clear();self.calibration_status='movement_detected';return
            self.calibration_status='calibrating'
            if len(self.samples)==200:
                self.bias=[sum(s[i] for s in self.samples)/200 for i in range(3)];self.ready=True;self.calibration_status='ready'
            return
        self.gyro=[angular[i]-self.bias[i] for i in range(3)]
        # Relative clutch orientation avoids an absolute yaw promise; Home reanchors.
        rate=math.sqrt(sum(v*v for v in self.gyro));angle=rate*.005
        if rate>1e-6:
            delta=[math.cos(angle/2)]+[v/rate*math.sin(angle/2) for v in self.gyro]
            self.quaternion=quaternion_product(self.quaternion,delta)
            norm=math.sqrt(sum(v*v for v in self.quaternion));self.quaternion=[v/norm for v in self.quaternion]


class JoyCon:
    def __init__(self,row,deadzone):
        import hid
        self.side='left' if row['product_id']==0x2006 else 'right'
        self.device=hid.device();self.device.open_path(row['path']);self.device.set_nonblocking(1)
        self.count=0;self.deadzone=deadzone;self.last_packet=None;self.last_received=0.;self.state=None;self.sequence=0
        try:
            self.subcommand(0x03,b'\x30');self.subcommand(0x40,b'\x01')
            # Select the sensitivity whose factory scale is documented below.
            self.subcommand(0x41,b'\x03\x00\x01\x01')
            useraddr=0x8010 if self.side=='left' else 0x801b
            user=self.spi(useraddr,11)
            stick=user[2:] if user[:2]==b'\xb2\xa1' else self.spi(0x603d if self.side=='left' else 0x6046,9)
            self.cal=stick_calibration(stick,self.side)
            imu=self.spi(0x8026,26)
            self.sensor=Sensor(imu[2:] if imu[:2]==b'\xb2\xa1' else self.spi(0x6020,24))
        except Exception:self.device.close();raise

    def subcommand(self,command,data):
        if command not in (0x03,0x40,0x41,0x10):raise ValueError('Input-only command allowlist')
        packet=bytes([1,self.count&15,0,1,0x40,0x40,0,1,0x40,0x40,command])+data
        self.count+=1;self.device.write(packet+bytes(64-len(packet)))
        deadline=time.monotonic()+.6
        while time.monotonic()<deadline:
            reply=bytes(self.device.read(64))
            if len(reply)>=15 and reply[0]==0x21 and reply[14]==command:
                if not reply[13]&0x80:raise OSError('Joy-Con rejected configuration')
                return reply
            time.sleep(.002)
        raise OSError('Joy-Con configuration acknowledgement timed out')

    def spi(self,address,length):
        reply=self.subcommand(0x10,struct.pack('<IB',address,length))
        if len(reply)<20+length or reply[15:20]!=struct.pack('<IB',address,length):raise OSError('Calibration reply mismatch')
        return reply[20:20+length]

    def read(self):
        for _ in range(512):
            raw=bytes(self.device.read(64))
            if not raw:break
            if len(raw)<49 or raw[0]!=0x30 or raw[1]==self.last_packet:continue
            now=time.monotonic()
            if self.last_received and now-self.last_received>.2:
                # Never integrate old orientation across a Bluetooth outage.
                self.sensor.reset('stream_interrupted')
            self.last_packet=raw[1];self.last_received=now;self.sequence+=1
            for i in (13,25,37):self.sensor.update(raw[i:i+12])
            self.state=raw
        else:raise OSError('Joy-Con report backlog; reconnect before practice')

    def close(self):
        # Leave streaming mode intact: macOS may also be using the controller.
        self.device.close()


class HIDReader:
    def __init__(self,deadzone=.12):
        self.deadzone=deadzone;self.devices={};self.session=uuid.uuid4().hex;self.sequence=0;self.last_scan=-10.;self.errors={}

    def scan(self):
        import hid
        rows=[r for r in hid.enumerate(0x057e,0) if r['product_id'] in (0x2006,0x2007)]
        for pid,side in ((0x2006,'left'),(0x2007,'right')):
            matching={r['path']:r for r in rows if r['product_id']==pid}
            if side in self.devices:continue
            if len(matching)!=1:
                self.errors[side]='Connect exactly one '+side+' original Joy-Con';continue
            try:self.devices[side]=JoyCon(next(iter(matching.values())),self.deadzone);self.errors.pop(side,None);self.session=uuid.uuid4().hex
            except (OSError,ValueError,ZeroDivisionError) as e:self.errors[side]=str(e)

    def sample(self):
        now=time.monotonic()
        if now-self.last_scan>3:self.last_scan=now;self.scan()
        for side,device in list(self.devices.items()):
            try:
                device.read()
                if device.last_received and now-device.last_received>1:raise OSError('Controller packet timeout')
            except (OSError,ValueError):device.close();del self.devices[side];self.session=uuid.uuid4().hex
        self.sequence+=1
        frame=dict(source='live',backend='Nintendo HID',input_only=True,schema_version=1,event='sample',timestamp=time.time(),session_id=self.session,sequence=self.sequence,controllers=[],diagnostics=dict(self.errors))
        if set(self.devices)!={'left','right'} or any(d.state is None or time.monotonic()-d.last_received>.12 for d in self.devices.values()):return frame
        left,right=self.devices['left'],self.devices['right'];l,r=left.state,right.state
        values={'Left Shoulder':bool(l[5]&64),'Right Shoulder':bool(r[3]&64),'Left Trigger':bool(l[5]&128),'Right Trigger':bool(r[3]&128),
                'Left Thumbstick Button':bool(l[4]&8),'Right Thumbstick Button':bool(r[4]&4),'Button Options':bool(l[4]&1),
                'Button Menu':bool(r[4]&2),'Button Home':bool(r[4]&16),'Button A':bool(r[3]&8),'Button B':bool(r[3]&4),
                'Button X':bool(r[3]&2),'Button Y':bool(r[3]&1)}
        pads={'Left Thumbstick':normalize_stick(unpack_stick(l[6:9]),left.cal,self.deadzone),'Right Thumbstick':normalize_stick(unpack_stick(r[9:12]),right.cal,self.deadzone)}
        dx=float(bool(l[5]&4))-float(bool(l[5]&8));dy=float(bool(l[5]&2))-float(bool(l[5]&1))
        pads['Direction Pad']={a:dict(raw=v,filtered=v) for a,v in (('x',dx),('y',dy))}
        motion={side:dict(ready=d.sensor.ready,age_s=max(0,time.monotonic()-d.last_received),quaternion=list(d.sensor.quaternion),rotation_rate=list(d.sensor.gyro),session=d.sensor.session,
                         calibration=dict(status=d.sensor.calibration_status,samples=len(d.sensor.samples),required=200,acceleration_g=list(d.sensor.accel),angular_rate=list(d.sensor.angular),raw=list(d.sensor.raw))) for side,d in self.devices.items()}
        frame['controllers']=[dict(id=self.session,connected=True,role='pair',name='Original Joy-Con pair (raw HID)',remapped=False,input_event_count=left.sequence+right.sequence,
                                   buttons={n:dict(pressed=v,value=int(v),physical_names=[n]) for n,v in values.items()},pads=pads,independent_motion=motion)]
        return frame

    def close(self):
        for d in self.devices.values():d.close()


def main():
    ap=argparse.ArgumentParser();ap.add_argument('--json',action='store_true');ap.add_argument('--hz',type=float,default=30);ap.add_argument('--deadzone',type=float,default=.12);ap.add_argument('--seconds',type=float,default=0)
    a=ap.parse_args()
    if not 1<=a.hz<=120 or not 0<=a.deadzone<1:ap.error('Invalid sample rate/deadzone')
    reader=HIDReader(a.deadzone);stop=threading.Event()
    for s in (signal.SIGTERM,signal.SIGINT):signal.signal(s,lambda *_:stop.set())
    started=time.monotonic()
    try:
        while not stop.is_set() and (not a.seconds or time.monotonic()-started<a.seconds):
            print(json.dumps(reader.sample(),allow_nan=False),flush=True);stop.wait(1/a.hz)
    except BrokenPipeError:pass
    finally:reader.close()

if __name__=='__main__':main()
