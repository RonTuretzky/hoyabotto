"""Continuous joint trajectories with measured speed/acceleration constraints.

The whole path is one action. Encoder updates are small samples of that path,
not independent moves that require settling or an LLM decision. No collision
or workspace calibration is inferred here; the owner must supply a separately
commissioned joint corridor and continue its health/camera/watchdog checks.
"""
from __future__ import annotations

import math

import numpy as np

from .common import Refused, finite, vector


class JointTrajectory:
    def __init__(self, waypoints, joints, corridor, velocity, acceleration):
        self.joints = list(joints)
        if not self.joints or len(set(self.joints)) != len(self.joints):
            raise Refused("Trajectory needs unique joints")
        if not isinstance(waypoints, list) or not 2 <= len(waypoints) <= 100:
            raise Refused("Trajectory needs 2–100 timed waypoints")
        self.times = np.asarray([finite(p["time_s"]) for p in waypoints])
        if self.times[0] != 0 or np.any(np.diff(self.times) <= 0):
            raise Refused("Trajectory times must start at zero and increase strictly")
        if any(set(p["positions"]) != set(self.joints) for p in waypoints):
            raise Refused("Every waypoint must specify the same joint set")
        self.q = np.asarray([[finite(p["positions"][n]) for n in self.joints] for p in waypoints])
        vmax = vector([finite(velocity[n]) for n in self.joints], len(self.joints))
        amax = vector([finite(acceleration[n]) for n in self.joints], len(self.joints))
        if min(vmax) <= 0 or min(amax) <= 0:
            raise Refused("Measured velocity and acceleration limits must be positive")
        for i, name in enumerate(self.joints):
            lo, hi = vector(corridor[name], 2)
            if not 0 <= lo < hi <= 4095 or np.min(self.q[:, i]) < lo or np.max(self.q[:, i]) > hi:
                raise Refused("Trajectory leaves the commissioned encoder corridor")
        # Monotone cubic Hermite interpolation: no position overshoot, no
        # intermediate rest on a same-direction path, rest at genuine reversal.
        dt = np.diff(self.times)
        slopes = np.diff(self.q, axis=0)/dt[:, None]
        self.v = np.zeros_like(self.q)
        for i in range(1, len(self.q)-1):
            same = slopes[i-1]*slopes[i] > 0
            w1, w2 = 2*dt[i]+dt[i-1], dt[i]+2*dt[i-1]
            self.v[i, same] = (w1+w2)/(w1/slopes[i-1, same]+w2/slopes[i, same])
        # Analytically bound the entire polynomial, including extrema between
        # waypoints. Stretch time globally; never clip only sampled velocities.
        peak_v = np.zeros(len(self.joints))
        peak_a = np.zeros(len(self.joints))
        for i, h in enumerate(dt):
            a, b, c, _ = self._coefficients(i)
            peak_a = np.maximum(peak_a, np.maximum(abs(2*b), abs(6*a+2*b))/h**2)
            peak_v = np.maximum(peak_v, np.maximum(abs(c), abs(3*a+2*b+c))/h)
            for k in range(len(self.joints)):
                if abs(a[k]) > 1e-12:
                    u = -b[k]/(3*a[k])
                    if 0 < u < 1:
                        peak_v[k] = max(peak_v[k], abs(3*a[k]*u*u+2*b[k]*u+c[k])/h)
        stretch = max(1.0, float(np.max(peak_v/vmax)), math.sqrt(float(np.max(peak_a/amax))))
        self.times *= stretch
        self.v /= stretch
        self.duration = float(self.times[-1])
        if self.duration > 30:
            raise Refused("Rate-limited trajectory exceeds the 30-second local supervision horizon")
        self.travel_ticks = float(np.sum(abs(np.diff(self.q, axis=0))))

    def _coefficients(self, i):
        h = self.times[i+1]-self.times[i]
        d = self.q[i]
        c = h*self.v[i]
        b = 3*(self.q[i+1]-self.q[i])-h*(2*self.v[i]+self.v[i+1])
        a = 2*(self.q[i]-self.q[i+1])+h*(self.v[i]+self.v[i+1])
        return a, b, c, d

    def sample(self, elapsed):
        t = min(max(finite(elapsed), 0), self.duration)
        i = min(int(np.searchsorted(self.times, t, side="right"))-1, len(self.times)-2)
        h = self.times[i+1]-self.times[i]
        u = (t-self.times[i])/h
        a, b, c, d = self._coefficients(i)
        q = ((a*u+b)*u+c)*u+d
        velocity = (3*a*u*u+2*b*u+c)/h
        acceleration = (6*a*u+2*b)/h**2
        return tuple(dict(zip(self.joints, values.tolist())) for values in (q, velocity, acceleration))

    def assert_start(self, actual, tolerance=5):
        if any(abs(finite(actual[n])-self.q[0, i]) > tolerance for i, n in enumerate(self.joints)):
            raise Refused("Trajectory start differs from live encoders")

    def check_following(self, actual, elapsed, tolerance):
        expected = self.sample(elapsed)[0]
        if any(abs(finite(actual[n])-expected[n]) > tolerance for n in self.joints):
            raise Refused("Trajectory tracking error exceeded the commissioned tolerance")
