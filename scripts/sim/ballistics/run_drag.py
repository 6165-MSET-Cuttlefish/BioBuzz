import math
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geom import *  # noqa
from drag2 import *  # noqa

which = sys.argv[1:] or [c[0] for c in CASES]


def slack_vec(P, V, rim=RIM):
    dp = P[:3] - O[:, None]
    u = U @ dp; w = Wv @ dp
    sp = np.sqrt((V * V).sum(0))
    inc = -(N @ V) / sp
    margin = POLLEN_D / 2 / np.maximum(inc, 0.33) + rim
    half = W_OPEN / 2; rise = H_OPEN - SHOULDER; slant = math.hypot(half, rise)
    below = ((H_OPEN - w) * half - np.abs(u) * rise) / slant
    sl = np.minimum(np.minimum(w - margin, half - np.abs(u) - margin), below - margin)
    ok = (sl >= 0) & (V[2] < 0)
    return u, w, ok


def entry_batch(d, phi, v, th, k, scale=None, yaw_deg=None):
    p = math.radians(phi)
    p0 = np.array([AIM[0] - d * math.sin(p), AIM[1] - d * math.cos(p), EXIT_Z])
    before = N @ (p0 - O)
    if scale is not None:
        vs = v * (1 + scale); ths = np.full_like(scale, th); yaws = np.zeros_like(scale)
    else:
        vs = np.full_like(yaw_deg, v); ths = np.full_like(yaw_deg, th); yaws = np.radians(yaw_deg)
    az = p + yaws
    S0 = np.vstack([np.full_like(vs, p0[0]), np.full_like(vs, p0[1]), np.full_like(vs, p0[2]),
                    vs * np.cos(ths) * np.sin(az), vs * np.cos(ths) * np.cos(az), vs * np.sin(ths)])
    T, OUT = fly(S0, k, lambda S: N @ (S[:3] - O[:, None]))
    u, w, ok = slack_vec(OUT[:3], OUT[3:])
    ok &= ~np.isnan(T)
    return before, u, w, ok


def window(xs, ok):
    i0 = len(xs) // 2
    if not ok[i0]:
        return None
    a = i0
    while a > 0 and ok[a - 1]:
        a -= 1
    b = i0
    while b < len(xs) - 1 and ok[b + 1]:
        b += 1
    return xs[a], xs[b]


for name, k in CASES:
    if name not in which:
        continue
    print(f"\n######## {name}: k = {k:.6f} 1/in" + (f", terminal v = {math.sqrt(G/k):.0f} in/s, drag/g at 250 in/s = {k*250*250/G:.2f}" if k else ""), flush=True)
    VM = {}
    print(" min-speed line:")
    for d in DS:
        v, th = vmin(d, k)
        VM[d] = (v, th)
        b, T = descent(np.array([v]), np.array([th]), d, k)
        print(f"  d={d:3d} vmin={v:7.2f} theta={math.degrees(th):6.2f} descent={b[0]:6.2f} tof={1000*T[0]:6.1f}ms", flush=True)
    print(" dz at aim distance vs hood offset at vmin; 1% loss nominal vs best hood:")
    for d in DS:
        v, th = VM[d]
        offs = np.radians(np.array([-2, -1, 1, 2.0]))
        T, Oo = at_d(np.full(4, v), th + offs, d, k)
        grid = th + np.radians(np.linspace(-6, 6, 241))
        T2, O2 = at_d(np.full(grid.shape, 0.99 * v), grid, d, k)
        z2 = O2[2] - AIM[2]
        T3, O3 = at_d(np.array([0.99 * v]), np.array([th]), d, k)
        j = np.nanargmax(z2)
        print(f"  d={d}: " + " ".join(f"{o:+.0f}:{z:+.3f}" for o, z in zip((-2, -1, 1, 2), Oo[2] - AIM[2])) +
              f" | 1% loss nominal {O3[2][0]-AIM[2]:+.3f} best {z2[j]:+.3f} at {math.degrees(grid[j]-th):+.2f}deg", flush=True)
    print(" fixed descent angle beta: speed, angle, hood-absorbable loss, dz at -1%/+1% speed:")
    FD = {}
    for beta in (30, 45, 50, 55):
        row = []
        for d in DS:
            r = for_descent(d, beta, k)
            if r is None:
                row.append(f"{d}:n/a"); continue
            v, th = r
            FD[(d, beta)] = r
            T, Oo = at_d(np.array([0.99 * v, 1.01 * v]), np.array([th, th]), d, k)
            dz = Oo[2] - AIM[2]
            row.append(f"{d}:v={v:6.1f} th={math.degrees(th):5.1f} absorb={100*(1-VM[d][0]/v):5.2f}% dz-1%={dz[0]:+5.2f} dz+1%={dz[1]:+5.2f}")
        print(f"  beta={beta}: " + " ; ".join(row), flush=True)
    print(" 84 in, 55 deg descent: TOF change when hood compensates loss (high branch)")
    for d in (84,):
        v, th = FD[(d, 55)]
        _, T0 = descent(np.array([v]), np.array([th]), d, k)
        row = []
        for loss in (0.01, 0.02, 0.03, 0.04):
            vv = v * (1 - loss)
            lo, hi = VM[d][1], th + math.radians(1)
            T, Oo = at_d(np.array([vv]), np.array([lo]), d, k)
            if Oo[2][0] < AIM[2]:
                row.append(f"{loss*100:.0f}%:none"); continue
            for _ in range(30):
                m = 0.5 * (lo + hi)
                T, Oo = at_d(np.array([vv]), np.array([m]), d, k)
                if Oo[2][0] > AIM[2]:
                    lo = m
                else:
                    hi = m
            row.append(f"{loss*100:.0f}%:dth={math.degrees(lo-th):+.2f} dT={1000*(T[0]-T0[0]):+6.1f}ms")
        print(f"  T0={1000*T0[0]:.0f}ms " + " ".join(row), flush=True)
    print(" windows (speed %, yaw deg) at phi 0/20/40/60:")
    scales = np.linspace(-0.08, 0.08, 1601)
    yaws = np.linspace(-15, 15, 1201)
    for fam in ("vmin", "b30", "b45"):
        for d in (36, 84, 132):
            if fam == "vmin":
                v, th = VM[d]
            else:
                r = FD.get((d, int(fam[1:])))
                if r is None:
                    continue
                v, th = r
            row = []
            for phi in (0, 20, 40, 60):
                before, u, w, ok = entry_batch(d, phi, v, th, k, scale=scales)
                if before <= 0:
                    row.append(f"phi{phi}:BEHIND"); continue
                sw = window(scales, ok)
                _, _, _, oky = entry_batch(d, phi, v, th, k, yaw_deg=yaws)
                yw = window(yaws, oky)
                i0 = len(scales) // 2
                row.append(f"phi{phi}: u={u[i0]:+.2f} w={w[i0]:.2f} spd {100*sw[0]:+.2f}/{100*sw[1]:+.2f}% yaw {yw[0]:+.2f}/{yw[1]:+.2f}" if sw and yw else f"phi{phi}: nominal fails")
            print(f"  {fam} d={d}: " + " | ".join(row), flush=True)
    print(" speed interpolation error on the min-speed line:")
    dd = np.arange(36, 133, 4.0)
    vv = np.array([vmin(d, k)[0] for d in dd])
    for nn, nodes in (("5", [36, 60, 84, 108, 132]), ("4", [36, 68, 100, 132])):
        nv = np.interp(nodes, dd, vv)
        err = (np.interp(dd, nodes, nv) - vv) / vv
        print(f"  {nn} nodes: max |err| {100*np.max(np.abs(err)):.3f}% (at {dd[np.argmax(np.abs(err))]:.0f} in)")
    A = np.vstack([dd, np.ones_like(dd)]).T
    c, *_ = np.linalg.lstsq(A, vv, rcond=None)
    print(f"  straight line max dev {100*np.max(np.abs(vv - A @ c)/vv):.2f}%", flush=True)
