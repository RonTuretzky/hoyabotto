"""Protocol fixtures, calibration and independent gyro state; no devices opened."""
import math
import struct
import time
import unittest
from types import SimpleNamespace
from hid_reader import Sensor,JoyCon,HIDReader,stick_calibration,normalize_stick


def pack(x,y):return bytes((x&255,((x>>8)&15)|((y&15)<<4),(y>>4)&255))
def imu_cal():return struct.pack('<12h',0,0,0,16384,16384,16384,0,0,0,13371,13371,13371)

class HIDTests(unittest.TestCase):
    def test_asymmetric_stick_calibration_both_sides(self):
        left=pack(1200,1100)+pack(2000,2100)+pack(1000,900)
        right=pack(2000,2100)+pack(1000,900)+pack(1200,1100)
        for side,raw in (('left',left),('right',right)):
            cal=stick_calibration(raw,side)
            self.assertEqual(normalize_stick((2000,2100),cal,.12)['x']['filtered'],0)
            self.assertEqual(normalize_stick((3200,1200),cal,.12)['x']['filtered'],1)
            self.assertEqual(normalize_stick((3200,1200),cal,.12)['y']['filtered'],-1)
        with self.assertRaises(ValueError):stick_calibration(b'\xff'*9,'left')

    def test_gyro_stationary_bias_then_independent_rotation(self):
        left,right=Sensor(imu_cal()),Sensor(imu_cal())
        still=struct.pack('<6h',0,0,4096,10,-5,8)
        for _ in range(200):left.update(still);right.update(still)
        self.assertTrue(left.ready and right.ready)
        for _ in range(200):left.update(struct.pack('<6h',0,0,4096,1010,-5,8));right.update(still)
        self.assertGreater(abs(left.quaternion[1]),.4)
        self.assertLess(abs(right.quaternion[1]),1e-8)
        self.assertAlmostEqual(sum(v*v for v in left.quaternion),1)
        sensor=Sensor(imu_cal())
        for _ in range(220):sensor.update(struct.pack('<6h',0,0,4096,3000,0,0))
        self.assertFalse(sensor.ready)

    def test_raw_pair_buttons_freshness_and_axes(self):
        reader=HIDReader();reader.last_scan=time.monotonic()
        for side in ('left','right'):
            raw=bytearray(49);raw[0]=0x30;raw[6:9]=pack(2048,2048);raw[9:12]=pack(2048,2048)
            if side=='left':raw[5]=64|128|2|4;raw[4]=8;raw[6:9]=pack(1000,3000)
            else:raw[3]=64|128|2|8;raw[4]=4|2|16;raw[9:12]=pack(3000,1000)
            reader.devices[side]=SimpleNamespace(state=raw,last_received=time.monotonic(),sequence=2,cal=((2048,2048),(1200,1200),(1200,1200)),sensor=Sensor(imu_cal()),read=lambda:None,close=lambda:None)
        f=reader.sample();c=f['controllers'][0]
        for name in ('Left Shoulder','Right Shoulder','Left Trigger','Right Trigger','Button X','Button A','Button Menu','Button Home','Left Thumbstick Button','Right Thumbstick Button'):
            self.assertTrue(c['buttons'][name]['pressed'],name)
        self.assertLess(c['pads']['Left Thumbstick']['x']['filtered'],0)
        self.assertGreater(c['pads']['Right Thumbstick']['x']['filtered'],0)
        self.assertEqual(c['pads']['Direction Pad']['y']['filtered'],1)
        reader.devices['right'].last_received-=.2
        self.assertFalse(reader.sample()['controllers'])

    def test_persistent_and_rumble_commands_rejected(self):
        device=JoyCon.__new__(JoyCon)
        for cmd in (0x11,0x12,0x01,0x06,0x07,0x30,0x48):
            with self.assertRaises(ValueError):device.subcommand(cmd,b'')

if __name__=='__main__':unittest.main()
