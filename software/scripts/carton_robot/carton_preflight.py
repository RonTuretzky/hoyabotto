"""Read-only carton preflight: no torque, mode, goal, or calibration writes."""
import json
import sys
import time
from pathlib import Path

from carton_runtime import ROOT, PROFILE
from farm.config import load_profile
from farm.adapters.robot_lerobot import LeRobotXLeRobot
from strict_servo_replies import guard_replies

def run():
    robot = LeRobotXLeRobot(load_profile(PROFILE).robot).robot
    result = {'time': time.time(), 'writes': False, 'motors': []}
    try:
        for bus in (robot.bus1, robot.bus2):
            guard_replies(bus)
            bus.connect(handshake=False)
            for name in bus.motors:
                row = {'name': name}
                for field in ('Torque_Enable', 'Status', 'Present_Position', 'Present_Load', 'Present_Voltage', 'Operating_Mode'):
                    row[field] = int(bus.read(field, name, normalize=False, num_retry=3))
                if name in robot.calibration and not name.startswith('base_'):
                    cal = robot.calibration[name]
                    row['calibration_matches'] = all(int(bus.read(field, name, normalize=False, num_retry=3)) == value for field, value in [('Homing_Offset', cal.homing_offset), ('Min_Position_Limit', cal.range_min), ('Max_Position_Limit', cal.range_max)])
                    row['range'] = [cal.range_min, cal.range_max]
                    row['in_range'] = cal.range_min <= row['Present_Position'] <= cal.range_max
                    row['normalized'] = (row['Present_Position'] - cal.range_min) / (cal.range_max - cal.range_min) * (100 if name.endswith('gripper') else 200) - (0 if name.endswith('gripper') else 100)
                result['motors'].append(row)
    except Exception as exc:
        result['error'] = str(exc)
    finally:
        for bus in (robot.bus1, robot.bus2):
            if bus.is_connected:
                bus.disconnect(disable_torque=False)
    result['all_16_released'] = len(result['motors']) == 16 and all(x['Torque_Enable'] == 0 for x in result['motors'])
    (ROOT / 'work/carton-preflight.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    return result

if __name__ == '__main__':
    run()
