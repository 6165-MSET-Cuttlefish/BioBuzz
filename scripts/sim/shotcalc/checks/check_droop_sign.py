"""Independent analytic (vacuum) check of Study 3's hood-vs-deficit slope sign at the maxP shots: for each D take the
T1 maxP (rpm, theta) from study1.json, drop the exit speed by 100 rpm worth, and find the launch angle (nearest the
original) whose analytic vacuum trajectory crosses the opening plane at the same w, descending."""
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc

s, c = math.sin(sc.TILT), math.cos(sc.TILT)


def cross(v, th_deg, D):
    th = math.radians(th_deg)
    vx, vz = v * math.cos(th), v * math.sin(th)
    xo = sc.opening_origin_x(D)
    a = -0.5 * sc.G * s
    b = -vx * c + vz * s
    cc = xo * c + (sc.EXIT_Z - sc.Z_BOTTOM) * s
    disc = b * b - 4 * a * cc
    if disc < 0:
        return None
    for t in sorted([(-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)]):
        if t > 0:
            x, z = vx * t, sc.EXIT_Z + vz * t - 0.5 * sc.G * t * t
            return (x - xo) * s + (z - sc.Z_BOTTOM) * c, vz - sc.G * t
    return None


res = json.load(open(os.path.join(os.path.dirname(HERE), "results", "study1.json")))["T1_vac"]["policies"]["maxP"]
rows = []
for p in res:
    D, rpm, h, th = p["D"], p["rpm"], p["hood"], p["theta"]
    v0 = float(sc.rpm_to_v(rpm, h))
    w0, vz0 = cross(v0, th, D)
    v1 = float(sc.rpm_to_v(rpm - 100, h))
    best = None
    for dth in np.arange(-15, 15.001, 0.01):
        r = cross(v1, th + dth, D)
        if r is None or r[1] >= 0:
            continue
        err = abs(r[0] - w0)
        if err < 0.02 and (best is None or abs(dth) < abs(best)):
            best = dth
    # also: max over angles of w at the reduced speed (can any angle reach w0?)
    wmax = max((cross(v1, a, D) or (-99, 1))[0] for a in np.arange(30, 80.01, 0.05) if (cross(v1, a, D) or (0, 1))[1] < 0)
    rows.append(f"D={D:4d} theta0={th:5.1f} w0={w0:5.2f} vz0={vz0:+5.0f}  -100rpm: dtheta to keep w = {('none' if best is None else f'{best:+.2f}')} deg; "
                f"max w reachable descending at -100rpm = {wmax:5.2f} (band lo {sc.W_LO:.2f})")
txt = "\n".join(rows)
print(txt)
open(os.path.join(HERE, "check_droop_sign.txt"), "w").write(txt)
