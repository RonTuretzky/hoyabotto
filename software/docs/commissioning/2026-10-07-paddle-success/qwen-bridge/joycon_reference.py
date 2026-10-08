"""Pure, offline joint-unit references for the pinned upstream Joy-Con tool.

Never imports a motor driver, opens a port, or writes calibration. Simulator
zeros and range midpoints are not accepted as physical reference measurements.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace
from joycon_native_units import SerialMotorsBus,MotorNormMode

SOURCE_BINDING='b65b1cdd1b6c99016c87ecb79db8823182d1a8d816492e302af6de2e56601352'
JOINTS=('shoulder_pan','shoulder_lift','elbow_flex','wrist_flex','wrist_roll','gripper')
POSITION_NAMES=tuple(f'{s}_arm_{j}' for s in ('left','right') for j in JOINTS)+('head_motor_1','head_motor_2')
CAL_FIELDS=('range_min','range_max','homing_offset')
POSITION_MARGIN=40  # Keep the deployed pickup profile's commandable travel margin.
UNITS_BINDING='15d0d85ecbed5ce87dd6cacabc948eef352863c35f6e9ecb0d5da1edfabc1859'


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
    if type(value) not in (int,float) or not math.isfinite(value):raise ValueError(label+': finite value required')
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
    def load(cls,path):
        record=json.loads(Path(path).read_text())
        return NativeReference(record) if record.get('kind')=='upstream-joycon-native' else cls(record)

    def validate_calibration(self,calibration):
        if calibration_rows(calibration)!=self.calibration:raise ValueError('Upstream reference differs from current motor calibration')

    def raw_limits(self,name):
        c=self.calibration[name];return c['range_min']+POSITION_MARGIN,c['range_max']-POSITION_MARGIN

    def limits(self,name):
        if name.endswith('gripper'):return (0.,90.)
        return tuple(sorted(self.from_ticks(name,v) for v in self.raw_limits(name)))

    def from_ticks(self,name,ticks):
        ticks=number(ticks,name+' telemetry');lo,hi=self.raw_limits(name)
        # Readback may be at the endpoint, but never extrapolate outside the
        # saved physical calibration. Commands retain the 40-tick pickup margin.
        c=self.calibration[name]
        if not c['range_min']<=ticks<=c['range_max']:raise ValueError(name+': telemetry outside saved calibration')
        zero,scale=self.lines[name];return (ticks-zero)/scale

    def to_ticks(self,name,value):
        value=number(value,name+' target');zero,scale=self.lines[name]
        if name.endswith('gripper') and not 0<=value<=90:raise ValueError(name+': gripper target outside measured open/closed span')
        ticks=zero+scale*value;lo,hi=self.raw_limits(name)
        if not lo<=ticks<=hi:raise ValueError(name+': target outside saved limits')
        return ticks


def native_calibration_rows(calibration):
    rows=calibration_rows(calibration)
    for name,row in rows.items():
        c=calibration[name]
        mode=c['drive_mode'] if isinstance(c,dict) else c.drive_mode
        if type(mode) is not int or mode not in (0,1):raise ValueError(name+': saved drive_mode required')
        row['drive_mode']=mode
    return rows


def native_record(calibration,robot_id,evidence):
    """Bind existing calibration to the original driver's default units.

    This records no measured geometric zero and makes no physical-pose claim.
    The owner must still compare the exact calibration with hardware at startup.
    """
    return dict(schema=1,kind='upstream-joycon-native',source_binding=SOURCE_BINDING,
                units_binding=UNITS_BINDING,robot_id=robot_id,evidence=evidence,
                calibration=native_calibration_rows(calibration))


class NativeReference(PhysicalReference):
    """Original XLeRobot default RANGE units, using unmodified LeRobot methods."""
    quantization_ticks=2  # Two integer conversions in bound -> readback -> encode.
    def __init__(self,record):
        if (record.get('schema')!=1 or record.get('kind')!='upstream-joycon-native'
                or record.get('source_binding')!=SOURCE_BINDING or record.get('units_binding')!=UNITS_BINDING):
            raise ValueError('Native reference does not match the pinned upstream units')
        for key in ('robot_id','evidence'):
            if not isinstance(record.get(key),str) or not record[key].strip():raise ValueError('Calibration '+key+' required')
        self.calibration=native_calibration_rows(record['calibration']);self.record=record
        self.reference_id=digest(record)
        # This class contains only two pure conversion methods. It has no
        # device constructor, port handler, serial access, or motor writes.
        self.units=SerialMotorsBus()
        self.units.calibration={n:SimpleNamespace(**c) for n,c in self.calibration.items()}
        self.units.apply_drive_mode=True  # Feetech 0.6.1's original setting.
        self.units.motors={n:SimpleNamespace(norm_mode=MotorNormMode.RANGE_0_100 if n.endswith('gripper') else MotorNormMode.RANGE_M100_100) for n in POSITION_NAMES}
        self.units._id_to_name=lambda n:n  # Pure conversion keys are names, never bus addresses.

    def validate_calibration(self,calibration):
        if native_calibration_rows(calibration)!=self.calibration:raise ValueError('Native reference differs from current motor calibration')

    def from_ticks(self,name,ticks):
        ticks=number(ticks,name+' telemetry');c=self.calibration[name]
        if not c['range_min']<=ticks<=c['range_max']:raise ValueError(name+': telemetry outside saved calibration')
        return self.units._normalize({name:ticks})[name]

    def to_ticks(self,name,value):
        value=number(value,name+' target')
        ticks=self.units._unnormalize({name:value})[name];lo,hi=self.raw_limits(name)
        if not lo<=ticks<=hi:raise ValueError(name+': target outside saved command margin')
        return ticks

    def limits(self,name):
        lo,hi=self.raw_limits(name)
        # Leave one quantization tick inside the safety bound: the original
        # unnormalizer intentionally truncates floats to integer ticks.
        limits=sorted(self.from_ticks(name,v) for v in (lo+1,hi-1))
        if name.endswith('gripper'):limits=[max(0.,limits[0]),min(90.,limits[1])]
        return tuple(limits)


def main(argv=None):
    p=argparse.ArgumentParser(description='Offline physical Joy-Con reference; no motor or network I/O')
    sub=p.add_subparsers(dest='op',required=True)
    t=sub.add_parser('template');t.add_argument('--calibration',required=True);t.add_argument('--robot-id',required=True);t.add_argument('--out',required=True)
    n=sub.add_parser('native');n.add_argument('--calibration',required=True);n.add_argument('--robot-id',required=True);n.add_argument('--evidence',required=True);n.add_argument('--out',required=True)
    v=sub.add_parser('validate');v.add_argument('--reference',required=True);v.add_argument('--calibration',required=True)
    a=p.parse_args(argv)
    try:
        calibration=json.loads(Path(a.calibration).read_text())
        if a.op in ('template','native'):
            record=template(calibration,a.robot_id) if a.op=='template' else native_record(calibration,a.robot_id,a.evidence)
            if a.op=='native':NativeReference(record)
            with Path(a.out).open('x') as f:json.dump(record,f,indent=2);f.write('\n')
            print('Unverified geometric template written.' if a.op=='template' else 'Original native-unit calibration binding written; no hardware connected or validated.')
        else:
            ref=PhysicalReference.load(a.reference);ref.validate_calibration(calibration)
            print(json.dumps(dict(reference_id=ref.reference_id,robot_id=ref.record['robot_id'],position_joints=len(ref.calibration),motor_writes=0)))
        return 0
    except (OSError,ValueError,KeyError,TypeError) as e:
        print('Refused: '+str(e));return 2

if __name__=='__main__':raise SystemExit(main())
