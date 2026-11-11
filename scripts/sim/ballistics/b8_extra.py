import math
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geom import *  # noqa
from cross import *  # noqa

h = AIM[2] - EXIT_Z


def vmin(d):
    r = math.hypot(d, h)
    return math.sqrt(G * (h + r)), math.atan((h + r) / d)


def for_descent(d, beta_deg):
    tb = math.tan(math.radians(beta_deg))
    tth = 2 * h / d + tb
    vx2 = G * d / (2 * h / d + 2 * tb)
    return math.sqrt(vx2 * (1 + tth * tth)), math.atan(tth)


def edges(u, w, margin):
    half = W_OPEN / 2
    rise = H_OPEN - SHOULDER
    slant = math.hypot(half, rise)
    below = ((H_OPEN - w) * half - abs(u) * rise) / slant
    return {"bottom": w - margin, "side": half - abs(u) - margin, "roof": below - margin}


# launch-side check boundary: N.(p0 - O) > 0
print("BEHIND_OPENING boundary: launch at z=14 is on the opening's inner side when d*cos(phi) <= X")
X = (math.sin(TILT) * (BOTTOM_Z - EXIT_Z)) / math.cos(TILT) + AIM[1]
print(f"  X = {X:.2f} in (aim-plan distance along the normal); max phi at d: " +
      ", ".join(f"{d}:{math.degrees(math.acos(min(1, X/d))):.1f}deg" for d in (30, 36, 48, 60, 84, 132)))

print("\nPhysical entry for the BEHIND cases (ignore the launch-side check): inward crossing after the outward one")
for d in (36,):
    for fam in ("vmin", "b45"):
        v, th = vmin(d) if fam == "vmin" else for_descent(d, 45)
        for phi in (40, 60):
            p0 = launch_point(d, phi); v0 = launch_vel(v, th, phi)
            res, before = crossing_nodrag(p0, v0)
            t, p, vv = res
            u, w, inc, margin, slack, desc = entry_eval(p, vv)
            print(f"  {fam} d={d} phi={phi}: before={before:.2f}, inward crossing u={u:+.2f} w={w:.2f} inc={inc:.3f} slack={slack:+.2f} desc={desc} edges={ {k: round(x,2) for k,x in edges(u,w,margin).items()} }")

print("\nBinding edge at nominal (no error), vmin family")
for d in (60, 84, 132):
    v, th = vmin(d)
    for phi in (0, 20, 40, 60):
        p0 = launch_point(d, phi); v0 = launch_vel(v, th, phi)
        res, before = crossing_nodrag(p0, v0)
        t, p, vv = res
        u, w, inc, margin, slack, desc = entry_eval(p, vv)
        e = edges(u, w, margin)
        print(f"  d={d} phi={phi}: u={u:+.2f} w={w:.2f} margin={margin:.2f} " + " ".join(f"{k}={x:+.2f}" for k, x in e.items()))

print("\nCentre of the speed window and of the yaw window (the 'best' choice) vs phi, vmin family: worked out by hand from the windows_nodrag.py output")

print("\nAim-depth variant: aimDepth 0 (aim at the centroid on the face) at phi=60, vmin")
import geom
for depth in (3.0, 1.5, 0.0):
    aim = O + CW * Wv - depth * N
    for d in (60, 84, 132):
        hh = aim[2] - EXIT_Z
        r = math.hypot(d, hh)
        v = math.sqrt(G * (hh + r)); th = math.atan((hh + r) / d)
        p = math.radians(60)
        p0 = np.array([aim[0] - d * math.sin(p), aim[1] - d * math.cos(p), EXIT_Z])
        v0 = launch_vel(v, th, 60)
        res, before = crossing_nodrag(p0, v0)
        t, pp, vv = res
        u, w, inc, margin, slack, desc = entry_eval(pp, vv)
        print(f"  depth={depth} d={d}: u={u:+.2f} w={w:.2f} slack={slack:+.2f}")
