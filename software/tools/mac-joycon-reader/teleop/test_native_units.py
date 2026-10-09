"""Original driver's units, independent of simulator geometry; no device access."""
import ast
import copy
import hashlib
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import test_upstream_hardware as hardware_tests
from joycon_reference import NativeReference,native_record,POSITION_NAMES,UNITS_BINDING,digest
import joycon_native_units
from upstream_hardware import UpstreamHardware

VENDOR=Path(__file__).with_name('vendor')


def native_test_record():
    calibration=hardware_tests.reference_record()['calibration']
    for c in calibration.values():c['drive_mode']=0
    return native_record(calibration,'SYNTHETIC_TEST_ONLY','Synthetic native-unit test calibration')


class NativeUnitTests(unittest.TestCase):
    def test_original_methods_and_driver_configuration_are_pinned(self):
        rows=json.loads((VENDOR/'native-units-manifest.json').read_text())
        self.assertEqual(UNITS_BINDING,digest({r['source']:r['sha256'] for r in rows}))
        for row in rows:
            self.assertEqual(hashlib.sha256((VENDOR/row['source']).read_bytes()).hexdigest(),row['sha256'])
        source=ast.parse((VENDOR/'sources/lerobot_motors_bus.py.txt').read_text())
        old=next(n for n in source.body if isinstance(n,ast.ClassDef) and n.name=='SerialMotorsBus')
        new=ast.parse(Path(joycon_native_units.__file__).read_text())
        new=next(n for n in new.body if isinstance(n,ast.ClassDef) and n.name=='SerialMotorsBus')
        for method in new.body:
            original=next(n for n in old.body if isinstance(n,ast.FunctionDef) and n.name==method.name)
            self.assertEqual(ast.dump(original),ast.dump(method))
        config=ast.parse((VENDOR/'sources/native_config.py.txt').read_text())
        cls=next(n for n in config.body if isinstance(n,ast.ClassDef) and n.name=='XLerobot2WheelsConfig')
        mode=next(n for n in cls.body if isinstance(n,ast.AnnAssign) and n.target.id=='use_degrees')
        self.assertIs(ast.literal_eval(mode.value),False)
        driver=ast.parse((VENDOR/'sources/native_driver.py.txt').read_text())
        motor_modes={n.args[0].value:n.args[2] for n in ast.walk(driver) if isinstance(n,ast.Call) and isinstance(n.func,ast.Name) and n.func.id=='Motor'}
        # Original gripper motor 6 uses 0..100; arm/head use the shared norm_mode_body.
        self.assertEqual(ast.unparse(motor_modes[6]),'MotorNormMode.RANGE_0_100')
        self.assertEqual(ast.unparse(motor_modes[7]),'norm_mode_body')
        feetech=ast.parse((VENDOR/'sources/lerobot_feetech.py.txt').read_text())
        modes=[n.value for n in ast.walk(feetech) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='apply_drive_mode' for t in n.targets)]
        self.assertEqual([ast.literal_eval(x) for x in modes],[True])

    def test_normalized_targets_are_not_geometric_degrees(self):
        with patch('socket.socket',side_effect=AssertionError('No network')):
            record=native_test_record();ref=NativeReference(record)
            # Saved span 100..4000: body +50 means 75% of travel, not 50 degrees.
            self.assertEqual(ref.to_ticks('left_arm_shoulder_pan',50),3025)
            # Original tool's gripper target 90 means 90% of saved travel.
            self.assertEqual(ref.to_ticks('left_arm_gripper',90),3610)
            self.assertEqual(ref.from_ticks('head_motor_1',2050),0)
            for name in POSITION_NAMES:
                for tick in (400,1200,2050,3000,3600):
                    value=ref.from_ticks(name,tick)
                    self.assertLessEqual(abs(ref.to_ticks(name,value)-tick),1)
            record['calibration']['left_arm_shoulder_pan']['drive_mode']=1
            record['calibration']['left_arm_gripper']['drive_mode']=1
            ref=NativeReference(record)
            self.assertEqual(ref.to_ticks('left_arm_shoulder_pan',50),1075)
            self.assertEqual(ref.to_ticks('left_arm_gripper',90),490)
            for name in POSITION_NAMES:
                for value in ref.limits(name):
                    self.assertTrue(140<=ref.to_ticks(name,value)<=3960)
            with self.assertRaisesRegex(ValueError,'margin'):ref.to_ticks('left_arm_shoulder_pan',100)

    def test_calibration_direction_and_source_changes_refuse(self):
        record=native_test_record();ref=NativeReference(record)
        for key,value in (('range_min',101),('homing_offset',1),('drive_mode',1)):
            cal=copy.deepcopy(record['calibration']);cal['left_arm_elbow_flex'][key]=value
            with self.assertRaisesRegex(ValueError,'calibration'):ref.validate_calibration(cal)
        for key in ('source_binding','units_binding'):
            bad=copy.deepcopy(record);bad[key]='changed'
            with self.assertRaises(ValueError):NativeReference(bad)
        bad=copy.deepcopy(record);del bad['calibration']['head_motor_1']['drive_mode']
        with self.assertRaises(KeyError):NativeReference(bad)

    def test_captured_neutral_outside_command_span_is_not_pulled_to_limit(self):
        ref=NativeReference(native_test_record());robot=UpstreamHardware(None,ref,clock=lambda:0)
        robot.state=dict(ok=True,status_age_s=0,teleop=dict(upstream_reference_id=ref.reference_id,wheelbase_m=.45,wheel_limit_m_s=.02),
                         motors={n:dict(Present_Position=2048) for n in POSITION_NAMES},
                         goals={n:2048 for n in POSITION_NAMES})
        robot.received=0;robot.dead=dict(left=True,right=True)
        for name,ticks in [('left_arm_gripper',3900),('head_motor_1',110)]:
            robot.state['motors'][name]['Present_Position']=ticks
            robot.state['goals'][name]=ticks
            value=ref.from_ticks(name,ticks)
            bounded,_=robot.bound_upstream_positions({name:value})
            command=robot.encode_upstream_command(dict(positions=bounded,linear=0,angular=0),None)
            self.assertEqual(command['rates'][name],0)


class NativeHardwareAdapterTests(hardware_tests.HardwareAdapterTests):
    """Run the full actual owner/API fake-hardware scenarios in native units."""
    def make_reference(self):return NativeReference(native_test_record())

    def test_native_rounding_cannot_accumulate_drift_while_holding_rails(self):
        self.start();before={n:r['Present_Position'] for n,r in self.plant.bus.r.items()}
        self.rails(True);self.tick(30)
        self.assertEqual(before,{n:r['Present_Position'] for n,r in self.plant.bus.r.items()})


if __name__=='__main__':unittest.main()
