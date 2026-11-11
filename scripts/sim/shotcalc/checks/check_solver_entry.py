"""Vacuum: if tofS in the table is the measured time to the opening PLANE (docs/sotm-tuning.md step 5) but ShotSolver
.checkEntry flies a trajectory that reaches the AIM point (centroid, 3 in deep) at t = tofS, what entry w / vz does
the solver report for Study 1's maxP and minDistSens shots? (kDragPerS = 0, straight-on.)"""
import json, math, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc
s, c, G = math.sin(sc.TILT), math.cos(sc.TILT), sc.G
res = json.load(open(os.path.join(os.path.dirname(HERE), "results", "study1.json")))["T1_vac"]["policies"]
L = []
for pol in ("maxP", "minDistSens"):
    for p in res[pol]:
        D, t = p["D"], p["tof"]
        vx = D / t
        vz0 = (sc.Z_AIM - sc.EXIT_Z + 0.5 * G * t * t) / t
        xo = sc.opening_origin_x(D)
        a, b, cc = -0.5 * G * s, -vx * c + vz0 * s, xo * c + (sc.EXIT_Z - sc.Z_BOTTOM) * s
        tc = min(x for x in np.roots([a, b, cc]).real if x > 0)
        x, z = vx * tc, sc.EXIT_Z + vz0 * tc - 0.5 * G * tc * tc
        w = (x - xo) * s + (z - sc.Z_BOTTOM) * c
        vz = vz0 - G * tc
        spd = math.hypot(vx, vz)
        inc = (vx * c - vz * s) / spd
        margin = sc.BALL_D / 2 / max(inc, 0.33) + sc.RIM_CLEAR
        L.append(f"{pol:11s} D={D:3d} true: w={p['w']:.2f} vz={p['vz_entry']:+.0f} tof={t:.3f} | solver model: w={w:.2f} vz={vz:+.0f} "
                 f"launch {math.degrees(math.atan2(vz0, vx)):.1f} deg (true {p['theta']:.1f}) -> {'RISING' if vz >= 0 else 'ok'} {'MISSES' if w < margin else ''}")
txt = "\n".join(L); print(txt); open(os.path.join(HERE, "check_solver_entry.txt"), "w").write(txt)
