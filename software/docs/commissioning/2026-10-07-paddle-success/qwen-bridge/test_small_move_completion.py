"""Offline reproduction of the 3216 -> 3200 request that stopped at 3214."""
import unittest

from paddle_joint_executor import PaddleJointExecutor, small_move_tolerances


class SmallMoveCompletionTests(unittest.TestCase):
    joint = 'right_arm_shoulder_lift'

    def simulate(self, origin, target, measured, *, moving=False):
        now, writes = [0.], []
        n = self.joint
        engine = PaddleJointExecutor([n], {n: [824, 3270]}, writes.append,
                                     clock=lambda: now[0], wall=lambda: now[0])
        engine.start({'id': 1, 'session_started': 7, 'positions': {n: target},
                      'duration_s': .4}, {n: origin}, session_started=7)
        result = None
        for i in range(100):
            now[0] = i / 10
            result = engine.tick({n: origin if i == 0 else measured}, now[0],
                rows={n: {'Present_Velocity': 10 if moving else 0,
                          'Moving': int(moving), 'Present_Load': 0}})
            if not engine.active:
                break
        return result, engine, writes

    def test_recorded_shortfall_does_not_claim_endpoint_or_add_corrections(self):
        result, engine, writes = self.simulate(3216, 3200, 3214)
        self.assertFalse(result['endpoint_reached'])
        self.assertEqual(result['closure_outcome'], 'settled_short')
        self.assertEqual(result['settle_residual_ticks'], {self.joint: 14})
        self.assertEqual(writes, [{self.joint: 3200}])
        self.assertEqual(engine.corrections[self.joint], 0)
        self.assertEqual(engine.bias[self.joint], 0)

    def test_short_move_boundaries_both_directions(self):
        for direction in (-1, 1):
            for distance in (3, 16, 32, 40, 57):
                origin, target = 2000, 2000 + direction * distance
                tol = max(1, min(5, distance // 10))
                for error in (0, tol, tol + 1, distance):
                    with self.subTest(direction=direction, distance=distance, error=error):
                        result, engine, writes = self.simulate(origin, target, target - direction * error)
                        self.assertEqual(result['endpoint_reached'], error <= tol)
                        self.assertEqual(engine.corrections[self.joint], 0)
                        self.assertTrue(all(min(origin, target) <= w[self.joint] <= max(origin, target)
                                            for w in writes))

    def test_moving_joint_cannot_qualify(self):
        with self.assertRaisesRegex(RuntimeError, 'deadline'):
            self.simulate(2000, 2016, 2016, moving=True)

    def test_other_motion_contracts_unchanged(self):
        n = self.joint
        for positions, origins, path in [
            ({n: 2058}, {n: 2000}, False),
            ({n: 2016}, {n: 2000}, True),
            ({'right_arm_gripper': 2016}, {'right_arm_gripper': 2000}, False),
            ({'head_motor_1': 2016}, {'head_motor_1': 2000}, False),
            ({n: 2016, 'right_arm_elbow_flex': 2016},
             {n: 2000, 'right_arm_elbow_flex': 2000}, False),
        ]:
            self.assertEqual(small_move_tolerances(positions, origins, waypoints=path), {})


if __name__ == '__main__':
    unittest.main()
