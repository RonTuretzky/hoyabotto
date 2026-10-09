"""Commissioning refuses unsafe state before writes; admin bootstrap never stops an owner."""
import contextlib
import copy
import hashlib
import io
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import joycon_commissioning as C
import bootstrap_joycon_admin as B
import remote_admin as A


def status():
    return dict(started=3, hardware_server=True, ok=True, phase='idle', age_s=.01,
                rows={n:dict(Torque_Enable=0, Present_Position=2000,
                             firmware_position_limits=[100, 4000]) for n in C.MOTOR_NAMES},
                enabled_motors=[], missing_buses=[])


class CommissioningTests(unittest.TestCase):
    def test_bad_state_never_creates_a_reference(self):
        cases=[dict(age_s=2),dict(age_s=-1),dict(age_s=float('nan')),dict(ok=False),
               dict(phase='holding'),dict(hardware_server=False),dict(enabled_motors=['head_motor_1']),
               dict(missing_buses=['left']),dict(calibration_mismatches={'head_motor_1':{}}),
               dict(teleop={'active':True})]
        for fields in cases:
            with tempfile.TemporaryDirectory() as d:
                s=status()|fields
                with self.assertRaises(ValueError):C.prepare_native_reference('must-not-read',s,Path(d)/'refs')
                self.assertFalse((Path(d)/'refs').exists())
        for mutate in ('torque','missing'):
            s=status()
            if mutate=='torque':s['rows']['head_motor_1']['Torque_Enable']=1
            else:del s['rows']['head_motor_1']
            with self.assertRaises(ValueError):C.require_released(s)

    def test_native_reference_binds_current_limits_and_dry_run_writes_nothing(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);cal={n:dict(range_min=100,range_max=4000,homing_offset=0,drive_mode=0) for n in C.MOTOR_NAMES}
            path=root/'cal.json';path.write_text(json.dumps(cal));s=status()
            reference,out=C.prepare_native_reference(path,s,root/'refs',dry_run=True)
            self.assertFalse(out.exists())
            actual,out=C.prepare_native_reference(path,s,root/'refs')
            self.assertEqual(actual.reference_id,reference.reference_id)
            self.assertEqual(json.loads(out.read_text()),reference.record)
            s['rows']['head_motor_1']['firmware_position_limits']=[101,4000]
            with self.assertRaisesRegex(ValueError,'live limits'):C.prepare_native_reference(path,s,root/'other')
            self.assertFalse((root/'other').exists())

    def test_modes_are_fixed_and_only_opt_in(self):
        self.assertEqual(A.MODES['restart'],[])
        self.assertEqual(A.MODES['joycon-ready'],['--joycon-teleop','--native-joycon-reference','--no-wrist-cams'])
        self.assertEqual(A.MODES['joycon-ready-dry-run'],A.MODES['joycon-ready']+['--dry-run'])

    def test_pending_command_cannot_race_a_released_restart(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'command.json';s=status()
            path.write_text(json.dumps({'id':12,'session_started':s['started'],'op':'enable_motors'}))
            with self.assertRaisesRegex(ValueError,'Pending owner command'):C.require_no_pending_command(s,path)
            for result in ({'completed':12},{'failed_command_id':12},{'last_rejected':{'id':12}}):
                C.require_no_pending_command(s|result,path)
            C.require_no_pending_command(s|{'started':4},path)

    def bootstrap(self, *, dry_run=False, fail_start=False, unexpected=False):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'src';work=root/'work';source.mkdir();work.mkdir()
            old=b'old admin\n';new=b'new admin\n'
            (source/'remote_admin.py').write_bytes(new);(work/'remote_admin.py').write_bytes(old)
            stop_calls=[];starts=[]
            def start():
                starts.append(1)
                if fail_start and len(starts)==1:raise SystemExit('injected API startup failure')
                return SimpleNamespace(pid=22)
            def stop(pids,label,timeout):stop_calls.append((pids,label));return []
            def processes(script):return [11] if script=='gemma_hardware_owner.py' else ([] if stop_calls else [21])
            with patch.multiple(B.deploy,BRIDGE=source,WORK=work,SESSION=work/'session'),patch.object(B.deploy,'read_status',side_effect=status),patch.object(B.deploy,'processes',side_effect=processes),patch.object(B.deploy,'stop',side_effect=stop),patch.object(B.deploy,'start_api',side_effect=start),patch.object(B.deploy,'record_deploy'),patch.object(B,'BASELINE_ADMIN_SHA256',hashlib.sha256(b'other' if unexpected else old).hexdigest()),patch('sys.argv',['bootstrap']+(['--dry-run'] if dry_run else [])),contextlib.redirect_stdout(io.StringIO()):
                if fail_start:
                    with self.assertRaises(SystemExit):B.main()
                elif unexpected:
                    with self.assertRaisesRegex(ValueError,'reviewed baseline'):B.main()
                else:B.main()
            self.assertFalse(any(11 in pids for pids,_ in stop_calls))
            self.assertEqual((work/'remote_admin.py').read_bytes(),old if dry_run or fail_start or unexpected else new)
            if dry_run or unexpected:self.assertFalse(stop_calls);self.assertFalse(starts)
            if fail_start:self.assertEqual(len(starts),2)

    def test_bootstrap_preserves_owner_and_rolls_back_api_failure(self):
        for kwargs in ({},{'dry_run':True},{'fail_start':True},{'unexpected':True}):
            self.bootstrap(**kwargs)


if __name__=='__main__':unittest.main()
