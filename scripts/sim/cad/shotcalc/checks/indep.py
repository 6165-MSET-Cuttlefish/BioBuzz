"""Independent re-implementation (no import from cadcore/windows): scipy solve_ivp with events at rtol 1e-10,
geometry re-derived from BioBuzzField.java constants. Window edges by scan + brentq on the slack / vz functions."""
import math, os, sys, json
import numpy as np
from scipy.integrate import solve_ivp
from scipy.optimize import brentq

G = 386.09
T = math.radians(30)
S, C = math.sin(T), math.cos(T)
ZB, W, SH, H = 53.5, 20.0, 7.61, 14.0
rect, tri = W * SH, W * (H - SH) / 2
WC = (rect * SH / 2 + tri * (SH + (H - SH) / 3)) / (rect + tri)
DEPTH, RIM = 3.0, 1.0
AIMDX = WC * S + DEPTH * C
ZAIM = ZB + WC * C - DEPTH * S
SLANT = math.hypot(W / 2, H - SH)

BALL = {"pollen": (2.8, 24.9), "nectar": (3.6, 41.3)}
EXIT = {"pollen": ((0, 1.56, 12.46), (30, 1.99, 13.83)), "nectar": ((0, 1.23, 12.53), (20, 1.60, 13.57))}
BASE = {"pollen": 57.7, "nectar": 60.8}


def kq(dia, mass_g, cd, rho=1.204):
    A = math.pi * (dia * 0.0254 / 2) ** 2
    return rho * cd * A / (2 * mass_g / 1000) * 0.0254


def exitp(sh, m):
    (m0, x0, z0), (m1, x1, z1) = EXIT[sh]
    f = (m - m0) / (m1 - m0)
    return x0 + f * (x1 - x0), z0 + f * (z1 - z0)


def fly(v, th, x0, z0, D, k, kl=0.0, rim=RIM, dia=2.8, dshift=0.0):
    """Returns plane crossing (w, vx, vz, t) for the target at D+dshift and aim-x crossing (z, t) at D."""
    t = math.radians(th)
    xo = D + dshift - AIMDX

    def rhs(_, y):
        x, z, vx, vz = y
        sp = math.hypot(vx, vz)
        return [vx, vz, -k * sp * vx - kl * sp * vz, -G - k * sp * vz + kl * sp * vx]

    def plane(_, y):
        return -C * (y[0] - xo) + S * (y[1] - ZB)
    plane.terminal = False
    plane.direction = -1

    def aimx(_, y):
        return y[0] - D
    aimx.direction = 1

    def floor(_, y):
        return y[1] + 5
    floor.terminal = True
    sol = solve_ivp(rhs, (0, 5), [x0, z0, v * math.cos(t), v * math.sin(t)], events=(plane, aimx, floor),
                    rtol=1e-10, atol=1e-10, method="DOP853")
    out = {}
    if len(sol.t_events[0]):
        y = sol.y_events[0][0]
        out["w"] = (y[0] - xo) * S + (y[1] - ZB) * C
        out["vx"], out["vz"], out["tp"] = y[2], y[3], sol.t_events[0][0]
    if len(sol.t_events[1]):
        out["za"], out["ta"] = sol.y_events[1][0][1], sol.t_events[1][0]
        out["vza"], out["vxa"] = sol.y_events[1][0][3], sol.y_events[1][0][2]
    return out


def margin_fn(o, dia, rim=RIM):
    """Signed score margin: min(slack, -vz (in/s scaled)); >= 0 scores."""
    if "w" not in o:
        return -99.0
    sp = math.hypot(o["vx"], o["vz"])
    inc = (o["vx"] * C - o["vz"] * S) / sp
    mg = dia / 2 / max(inc, 0.33) + rim
    sl = min(o["w"] - mg, (H - o["w"]) * (W / 2) / SLANT - mg)
    return min(sl, -o["vz"] / 50.0)


def vaim(th, x0, z0, D, k, kl=0.0):
    f = lambda v: fly(v, th, x0, z0, D, k, kl).get("za", -500) - ZAIM
    return brentq(f, 150, 1500, xtol=1e-9)


def edge(fun, step, lim):
    """Walk from 0 by step until fun < 0, then brentq. fun(0) must be >= 0."""
    a = 0.0
    while abs(a) < lim:
        b = a + step
        if fun(b) < 0:
            return brentq(fun, a, b, xtol=1e-6)
        a = b
    return float("nan")


def shot(sh, m, D, case="nom", kl_ratio=0.0, rim=RIM, base=None):
    dia, mass = BALL[sh]
    base = BASE[sh] if base is None else base
    k = 0.0 if case == "none" else kq(dia, mass, 0.5)
    kl = k * kl_ratio / 0.5
    th = base + m
    x0, z0 = exitp(sh, m)
    v = vaim(th, x0, z0, D, k, kl)
    o = fly(v, th, x0, z0, D, k, kl)
    res = dict(sh=sh, m=m, D=D, v=v, tof_ms=1000 * o["ta"], desc_plane=math.degrees(math.atan2(-o["vz"], o["vx"])),
               w=o["w"], margin0=margin_fn(o, dia, rim))
    if res["margin0"] < 0:
        return res
    fs = lambda s: margin_fn(fly(v * (1 + s), th, x0, z0, D, k, kl), dia, rim)
    res["s_lo"] = 100 * edge(fs, -0.002, 0.3)
    res["s_hi"] = 100 * edge(fs, 0.002, 0.3)

    def fh(dm):
        xe, ze = exitp(sh, m + dm)
        return margin_fn(fly(v, th + dm, xe, ze, D, k, kl), dia, rim)
    res["h_lo"] = edge(fh, -0.1, 15)
    res["h_hi"] = edge(fh, 0.1, 15)
    fd = lambda d: margin_fn(fly(v, th, x0, z0, D, k, kl, dshift=d), dia, rim)
    res["d_lo"] = edge(fd, -0.1, 40)
    res["d_hi"] = edge(fd, 0.1, 40)
    return res


if __name__ == "__main__":
    cases = [("pollen", 15, 36), ("pollen", 8, 48), ("pollen", 2, 60), ("pollen", 0, 84), ("pollen", 0, 132),
             ("nectar", 11.5, 36), ("nectar", 0, 132), ("pollen", 8, 132)]
    rows = []
    for sh, m, D in cases:
        r = shot(sh, m, D)
        rows.append(r)
        print(f"{sh:6s} D{D:4d} m{m:5.1f}: v {r['v']:7.2f} tof {r['tof_ms']:6.1f} desc@plane {r['desc_plane']:5.1f} | "
              f"spd {r.get('s_lo', float('nan')):+.2f}/{r.get('s_hi', float('nan')):+.2f} (w {r.get('s_hi', 0) - r.get('s_lo', 0):.2f}) "
              f"hood {r.get('h_lo', float('nan')):+.2f}/{r.get('h_hi', float('nan')):+.2f} dist {r.get('d_lo', float('nan')):+.1f}/{r.get('d_hi', float('nan')):+.1f}", flush=True)
    json.dump(rows, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "indep_out.json"), "w"), indent=1)
