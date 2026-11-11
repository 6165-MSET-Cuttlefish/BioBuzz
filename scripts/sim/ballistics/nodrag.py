import math
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geom import *  # noqa

DS = [36, 60, 84, 108, 132]
h = AIM[2] - EXIT_Z
print(f"centroid w = {CW:.4f} in; AIM local = {AIM.round(4)}; aim z = {AIM[2]:.4f}; rise h = {h:.4f}")
print(f"aim offset from opening bottom centre: into cell {AIM[1]:.3f} in, up {AIM[2]-BOTTOM_Z:.3f} in")


def vmin(d):
    r = math.hypot(d, h)
    v = math.sqrt(G * (h + r))
    th = math.atan((h + r) / d)
    return v, th


def descent(d, v, th):
    vx = v * math.cos(th)
    T = d / vx
    vz = v * math.sin(th) - G * T
    return math.degrees(math.atan2(-vz, vx)), T


def for_descent(d, beta_deg):
    tb = math.tan(math.radians(beta_deg))
    tth = 2 * h / d + tb
    vx2 = G * d / (2 * h / d + 2 * tb)
    v = math.sqrt(vx2 * (1 + tth * tth))
    return v, math.atan(tth)


def solve_theta(d, v, high=True):
    # d tan - g d^2 (1+tan^2)/(2v^2) = h
    a = G * d * d / (2 * v * v)
    disc = d * d - 4 * a * (a + h)
    if disc < 0:
        return None
    t = (d + (1 if high else -1) * math.sqrt(disc)) / (2 * a)
    return math.atan(t)


print("\nmin-speed line")
for d in DS:
    v, th = vmin(d)
    b, T = descent(d, v, th)
    print(f" d={d:4d}  vmin={v:7.2f} in/s  theta={math.degrees(th):6.2f}  descent={b:6.2f}  tof={T*1000:6.1f} ms")

print("\nhood-absorbable speed loss: 1 - vmin/v at fixed descent angle")
for beta in (30, 40, 45, 50, 55, 60):
    row = []
    for d in DS:
        v, th = for_descent(d, beta)
        vm, _ = vmin(d)
        row.append(f"{d}:{100*(1-vm/v):5.2f}%(v={v:6.1f},th={math.degrees(th):5.1f})")
    print(f" beta={beta}: " + "  ".join(row))

print("\nvertical shift at fixed horizontal position per 1% speed, fixed angle: exact 0.02(h + d tan beta) and numeric")
for beta in (20, 30, 45):
    row = []
    for d in DS:
        v, th = for_descent(d, beta)
        lin = 0.02 * (h + d * math.tan(math.radians(beta)))
        v1 = v * 0.99
        z1 = d * math.tan(th) - G * d * d / (2 * v1 * v1 * math.cos(th) ** 2)
        v2 = v * 1.01
        z2 = d * math.tan(th) - G * d * d / (2 * v2 * v2 * math.cos(th) ** 2)
        k = math.cos(TILT) - 0.5 * math.tan(math.radians(beta) - TILT)
        row.append(f"{d}:lin {lin:4.2f} (-1%:{z1-h:+5.2f},+1%:{z2-h:+5.2f}) inplane~{lin*k:4.2f}")
    print(f" beta={beta}: " + " | ".join(row))
print(" on min-speed line:")
for d in DS:
    v, th = vmin(d)
    b, _ = descent(d, v, th)
    lin = 0.02 * (h + d * math.tan(math.radians(b)))
    print(f"  d={d} beta={b:5.2f}  dz/1% = {lin:4.2f} in")
