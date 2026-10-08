from test_gemma_hardware_owner import B,telemetry,C
from gemma_hardware_owner import HardwareOwner
b=B();cal={n:C(range_min=100,range_max=1000,homing_offset=0) for n in b.motors if not n.startswith('base')};b.r['left_arm_test']['Homing_Offset']=17
o=HardwareOwner([b],cal,telemetry)
try:o.inspect()
except RuntimeError as e:assert 'saved calibration differs' in str(e)
else:raise AssertionError('Normal owner accepted calibration mismatch')
assert not b.writes
o=HardwareOwner([b],cal,telemetry,read_only=True);o.inspect()
assert o.state['operator_armed'] is False and o.state['read_only'] is True
assert o.state['calibration_mismatches']['left_arm_test']['actual']['Homing_Offset']==17
assert len(o.rows)==3 and all(r['Torque_Enable']==0 for r in o.rows.values())
for command in [{'op':'enable_motors','names':['left_arm_test'],'enabled':True},{'op':'direct_joint','positions':{'left_arm_test':500},'duration_s':1}]:
 try:o.command(dict(command,id=1,session_started=o.started))
 except ValueError as e:assert 'READ_ONLY_OWNER' in str(e)
 else:raise AssertionError('Read-only owner accepted activation or motion')
assert not b.writes
print({'readonly_owner_checks':5,'normal_calibration_guard_preserved':True,'motor_writes':0,'hardware_access':False})

b=B();b.r['right_arm_test']['Present_Position']=500;b.r['left_arm_test']['Homing_Offset']=17
o=HardwareOwner([b],cal,telemetry,position_scope=['right_arm_test']);o.inspect()
assert o.state['operator_armed'] and o.state['supportsselectedjoints']==['right_arm_test']
assert o.state['commandable_motors']==['right_arm_test'] and not b.writes
try:o.enable(['left_arm_test'],True)
except ValueError as e:assert 'UNSUPPORTED_OWNER_SCOPE' in str(e)
else:raise AssertionError('Unselected mismatched motor enabled')
assert not b.writes
o.enable(['right_arm_test'],True);assert o.enabled=={'right_arm_test'}
o.release_all('fake test')
b.r['right_arm_test']['Homing_Offset']=18
o=HardwareOwner([b],cal,telemetry,position_scope=['right_arm_test'])
try:o.inspect()
except RuntimeError as e:assert 'saved calibration differs' in str(e)
else:raise AssertionError('Selected-arm calibration guard weakened')
print({'right_scope_checks':5,'selected_calibration_guard_retained':True,'hardware_access':False})
