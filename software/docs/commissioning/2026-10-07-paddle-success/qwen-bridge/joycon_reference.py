"""Pure, offline physical-unit reference for the pinned upstream Joy-Con tool.

Never imports a motor driver, opens a port, or writes calibration. Simulator
zeros and range midpoints are not accepted as physical reference measurements.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

SOURCE_BINDING='b65b1cdd1b6c99016c87ecb79db8823182d1a8d816492e302af6de2e56601352'
JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
POSITION_NAMES=tuple(f'{s}_arm_{j}' for s in ('left','right') for j in JOINTS)+('head_motor_1','head_motor_2')
CAL_FIELDS=('range_min','range_max','homing_offset')


def digest(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def calibration_rows(calibration):
    rows={}
    for name in POSITION_NAMES:
        c=calibration[name]
        rows[name]={key:c[key] if isinstance(c,dict) else getattr(c,key) for key in CAL_FIELDS}
        if any(type(v) is not int for v in rows[name].values()):raise ValueError(name+': invalid saved calibration')
        if not 0<=rows[name]['range_min']<rows[name]['range_max']<=4095:raise ValueError(name+': invalid saved range')
    return rows


def number(value,label):
    if type(value) not in (int,float) or not math.isfinite(value):raise ValueError(label+': measured finite value required')
    return float(value)


def template(calibration,robot_id):
    rows=calibration_rows(calibration)
    return dict(schema=1,kind='upstream-joycon-physical',source_binding=SOURCE_BINDING,
        robot_id=robot_id,verified=False,evidence='',calibration=rows,
        joints={n:(dict(closed_tick=None,open_tick=None,evidence='') if n.endswith('gripper') else
                   dict(reference_tick=None,reference_degrees=None,model_sign=None,evidence='')) for n in POSITION_NAMES})


class PhysicalReference:
    def __init__(self,record):
        if (record.get('schema')!=1 or record.get('kind')!='upstream-joycon-physical'
                or record.get('source_binding')!=SOURCE_BINDING):raise ValueError('Reference does not match the pinned upstream controller')
        if record.get('verified') is not True:raise ValueError('Physical joint references are not verified')
        for key in ('robot_id','evidence'):
            if not isinstance(record.get(key),str) or not record[key].strip():raise ValueError('Measured '+key+' required')
        if set(record.get('joints',{}))!=set(POSITION_NAMES):raise ValueError('All 14 position-joint references required')
        self.calibration=calibration_rows(record['calibration']);self.lines={};self.record=record
        self.reference_id=digest(record)
        for name,row in record['joints'].items():
            if not isinstance(row.get('evidence'),str) or not row['evidence'].strip():raise ValueError(name+': measured evidence required')
            lo,hi=self.raw_limits(name)
            if name.endswith('gripper'):
                closed=number(row.get('closed_tick'),name+' closed tick');opened=number(row.get('open_tick'),name+' open tick')
                if not lo<=closed<=hi or not lo<=opened<=hi or abs(opened-closed)<20:raise ValueError(name+': distinct measured gripper endpoints inside saved limits required')
                self.lines[name]=(closed,(opened-closed)/90.)
            else:
                tick=number(row.get('reference_tick'),name+' reference tick')
                degrees=number(row.get('reference_degrees'),name+' reference angle');sign=row.get('model_sign')
                if type(sign) is not int or sign not in (-1,1):raise ValueError(name+': measured joint direction required')
                if not lo<=tick<=hi:raise ValueError(name+': reference tick outside saved limits')
                scale=sign*4096/360;self.lines[name]=(tick-degrees*scale,scale)

    @classmethod
    def load(cls,path):return cls(json.loads(Path(path).read_text()))

    def validate_calibration(self,calibration):
        if calibration_rows(calibration)!=self.calibration:raise ValueError('Upstream reference differs from current motor calibration')

    def raw_limits(self,name):
        c=self.calibration[name];return c['range_min']+4,c['range_max']-4

    def limits(self,name):
        if name.endswith('gripper'):return (0.,90.)
        return tuple(sorted(self.from_ticks(name,v) for v in self.raw_limits(name)))

    def from_ticks(self,name,ticks):
        ticks=number(ticks,name+' telemetry');lo,hi=self.raw_limits(name)
        # Readback may be at the endpoint, but never extrapolate outside the
        # saved physical calibration. Commands retain the 4-tick margin.
        c=self.calibration[name]
        if not c['range_min']<=ticks<=c['range_max']:raise ValueError(name+': telemetry outside saved calibration')
        zero,scale=self.lines[name];return (ticks-zero)/scale

    def to_ticks(self,name,value):
        value=number(value,name+' target');zero,scale=self.lines[name]
        if name.endswith('gripper') and not 0<=value<=90:raise ValueError(name+': gripper target outside measured open/closed span')
        ticks=zero+scale*value;lo,hi=self.raw_limits(name)
        if not lo<=ticks<=hi:raise ValueError(name+': target outside saved limits')
        return ticks


def main(argv=None):
    p=argparse.ArgumentParser(description='Offline physical Joy-Con reference; no motor or network I/O')
    sub=p.add_subparsers(dest='op',required=True)
    t=sub.add_parser('template');t.add_argument('--calibration',required=True);t.add_argument('--robot-id',required=True);t.add_argument('--out',required=True)
    v=sub.add_parser('validate');v.add_argument('--reference',required=True);v.add_argument('--calibration',required=True)
    a=p.parse_args(argv)
    try:
        calibration=json.loads(Path(a.calibration).read_text())
        if a.op=='template':
            record=template(calibration,a.robot_id)
            with Path(a.out).open('x') as f:json.dump(record,f,indent=2);f.write('\n')
            print('Unverified template written. Measure joint zero/directions and gripper endpoints before use.')
        else:
            ref=PhysicalReference.load(a.reference);ref.validate_calibration(calibration)
            print(json.dumps(dict(reference_id=ref.reference_id,robot_id=ref.record['robot_id'],position_joints=len(ref.lines),motor_writes=0)))
        return 0
    except (OSError,ValueError,KeyError,TypeError) as e:
        print('Refused: '+str(e));return 2

if __name__=='__main__':raise SystemExit(main())
