import math
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geom import *  # noqa
from cross import *  # noqa

h = AIM[2] - EXIT_Z
DS = [36, 60, 84, 108, 132]


def vmin(d):
    r = math.hypot(d, h)
    return math.sqrt(G * (h + r)), math.atan((h + r) / d)


def for_descent(d, beta_deg):
    tb = math.tan(math.radians(beta_deg))
    tth = 2 * h / d + tb
    vx2 = G * d / (2 * h / d + 2 * tb)
    return math.sqrt(vx2 * (1 + tth * tth)), math.atan(tth)


def evaluate(d, phi, v, th, dyaw=0.0, rim=RIM):
    p0 = launch_point(d, phi)
    v0 = launch_vel(v, th, phi + dyaw)
    res, before = crossing_nodrag(p0, v0)
    if before <= 0:
        return None, "BEHIND"
    if res is None:
        return None, "NO_ENTRY"
    t, p, vv = res
    u, w, inc, margin, slack, desc = entry_eval(p, vv, rim=rim)
    ok = slack >= 0 and desc
    return (u, w, inc, margin, slack, desc, t), ("OK" if ok else ("RISING" if not desc else "MISS"))


def window(f, lo=-0.25, hi=0.25, n=2001):
    xs = np.linspace(lo, hi, n)
    good = np.array([f(x) for x in xs])
    i0 = n // 2
    if not good[i0]:
        return None
    a = i0
    while a > 0 and good[a - 1]:
        a -= 1
    b = i0
    while b < n - 1 and good[b + 1]:
        b += 1

    def refine(x_in, x_out):
        for _ in range(40):
            m = 0.5 * (x_in + x_out)
            if f(m):
                x_in = m
            else:
                x_out = m
        return x_in
    la = refine(xs[a], xs[a - 1]) if a > 0 else xs[a]
    lb = refine(xs[b], xs[b + 1]) if b < n - 1 else xs[b]
    return la, lb


def fams(d):
    out = {"vmin": vmin(d)}
    for b in (30, 45, 55):
        out[f"b{b}"] = for_descent(d, b)
    return out


print("drag-free windows. speed window = exit-speed scale error (%) at fixed hood angle; angle window = hood error (deg) at fixed speed; yaw = turret error (deg)")
print("dz = equivalent vertical shift at the aim's horizontal position (in), low/high")
for fam in ("vmin", "b30", "b45", "b55"):
    print(f"\n== family {fam}")
    for d in DS:
        v, th = fams(d)[fam]
        for phi in (0, 20, 40, 60):
            nom, st = evaluate(d, phi, v, th)
            if nom is None or st != "OK":
                print(f" d={d:3d} phi={phi:2d}: nominal {st}" + (f" (u={nom[0]:.2f}, w={nom[1]:.2f}, inc={nom[2]:.3f}, margin={nom[3]:.2f}, slack={nom[4]:.2f})" if nom else ""))
                continue
            u, w, inc, margin, slack, desc, t = nom
            sw = window(lambda x: evaluate(d, phi, v * (1 + x), th)[1] == "OK")
            aw = window(lambda x: evaluate(d, phi, v, th + math.radians(x))[1] == "OK", -15, 15)
            yw = window(lambda x: evaluate(d, phi, v, th, dyaw=x)[1] == "OK", -30, 30)
            dz = [d * math.tan(th) - G * d * d / (2 * (v * (1 + s)) ** 2 * math.cos(th) ** 2) - h for s in sw]
            print(f" d={d:3d} phi={phi:2d}: cross u={u:+5.2f} w={w:5.2f} inc={inc:.3f} margin={margin:.2f} slack={slack:.2f} | "
                  f"speed {100*sw[0]:+5.2f}%/{100*sw[1]:+5.2f}% (dz {dz[0]:+5.2f}/{dz[1]:+5.2f}) | hood {aw[0]:+6.2f}/{aw[1]:+6.2f} deg | yaw {yw[0]:+6.2f}/{yw[1]:+6.2f} deg")

print("\nrim=0 (geometric ball fit only), family b30 and vmin, phi=0")
for fam in ("b30", "vmin"):
    for d in DS:
        v, th = fams(d)[fam]
        sw = window(lambda x: evaluate(d, 0, v * (1 + x), th, rim=0)[1] == "OK")
        print(f" {fam} d={d}: speed {100*sw[0]:+5.2f}%/{100*sw[1]:+5.2f}%")

print("\nmin-speed line, phi=0: entry w on the opening plane and z at the aim distance vs hood offset")
for d in DS:
    v, th = vmin(d)
    row = []
    for dth in (-2, -1, -0.5, 0, 0.5, 1, 2):
        r, st = evaluate(d, 0, v, th + math.radians(dth))
        thp = th + math.radians(dth)
        z = d * math.tan(thp) - G * d * d / (2 * v * v * math.cos(thp) ** 2) - h
        row.append(f"{dth:+.1f}:w={r[1]:.3f},dz={z:+.3f}")
    print(f" d={d}: " + "  ".join(row))
    # first derivative of entry w wrt hood at vmin
    e = 1e-4
    wp = evaluate(d, 0, v, th + e)[0][1]; wm = evaluate(d, 0, v, th - e)[0][1]
    print(f"   dw/dtheta at vmin = {(wp-wm)/(2*e)*math.pi/180:+.4f} in/deg (opening plane, 3 in before aim)")
    # speed loss on min-speed line: best possible miss
    for loss in (0.01, 0.02):
        vv = v * (1 - loss)
        best = max(d * math.tan(t) - G * d * d / (2 * vv * vv * math.cos(t) ** 2) for t in np.linspace(th - 0.3, th + 0.3, 20001)) - h
        print(f"   {100*loss:.0f}% loss: best reachable dz at aim distance over all hood angles = {best:+.3f} in; at nominal hood = "
              f"{d*math.tan(th) - G*d*d/(2*vv*vv*math.cos(th)**2) - h:+.3f} in")

print("\nTOF at the aim point when the hood compensates speed loss (high branch), 55 deg descent")
for d in (36, 60, 84, 108, 132):
    v, th = for_descent(d, 55)
    T0 = d / (v * math.cos(th))
    row = []
    for loss in (0.01, 0.02, 0.03, 0.04):
        vv = v * (1 - loss)
        a = G * d * d / (2 * vv * vv)
        disc = d * d - 4 * a * (a + h)
        if disc < 0:
            row.append(f"{loss*100:.0f}%:none")
            continue
        t1 = math.atan((d + math.sqrt(disc)) / (2 * a))
        T1 = d / (vv * math.cos(t1))
        row.append(f"{loss*100:.0f}%:dth={math.degrees(t1-th):+.2f}deg dT={1000*(T1-T0):+6.1f}ms")
    print(f" d={d} T0={1000*T0:.0f}ms: " + " ".join(row))
