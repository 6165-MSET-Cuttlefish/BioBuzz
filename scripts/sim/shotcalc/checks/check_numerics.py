"""Independent numerical checks of shotcore: vacuum analytic crossing, RK4 step convergence, energy, linear-drag
closed form vs RK4, field interpolation error vs direct integration, Magnus does no work."""
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import shotcore as sc
import physmodel as pm

rng = np.random.default_rng(2024)
out = []


def analytic_vac_crossing(v, th_deg, D):
    th = math.radians(th_deg)
    vx, vz = v * math.cos(th), v * math.sin(th)
    s, c = math.sin(sc.TILT), math.cos(sc.TILT)
    xo = sc.opening_origin_x(D)
    # sd(t) = -(vx t - xo) c + (z0 + vz t - g t^2/2 - Zb) s = 0 ; first root where sd goes +->-
    a = -0.5 * sc.G * s
    b = -vx * c + vz * s
    cc = xo * c + (sc.EXIT_Z - sc.Z_BOTTOM) * s
    disc = b * b - 4 * a * cc
    if disc < 0:
        return None
    r = sorted([(-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)])
    for t in r:
        if t > 0:
            x = vx * t
            z = sc.EXIT_Z + vz * t - 0.5 * sc.G * t * t
            if z < 0:
                return None
            w = (x - xo) * s + (z - sc.Z_BOTTOM) * c
            return t, w, vz - sc.G * t
    return None


# 1. vacuum: direct integrate+crossings vs analytic
vs = rng.uniform(200, 550, 300)
ths = rng.uniform(30, 80, 300)
Ds = np.array([36.0, 60.0, 84.0, 108.0, 132.0])
t, X, Z, VX, VZ = sc.integrate(vs, np.radians(ths), 0.0, 0.0)
cr = sc.crossings(t, X, Z, VX, VZ, Ds, 0.0)
errs_w, errs_t, nmis = [], [], 0
for di, D in enumerate(Ds):
    for j in range(len(vs)):
        a = analytic_vac_crossing(vs[j], ths[j], D)
        ok = cr["ok"][di, j]
        if a is None or a[0] > 1.7:
            nmis += int(ok)
            continue
        if not ok:
            nmis += 1
            continue
        errs_w.append(abs(cr["w"][di, j] - a[1]))
        errs_t.append(abs(cr["tof"][di, j] - a[0]))
out.append(f"[1] vacuum crossings vs analytic: n={len(errs_w)} max|dw|={max(errs_w):.2e} in, max|dtof|={max(errs_t)*1e3:.3f} ms, ok-flag mismatches={nmis}")

# 2. RK4 step convergence (strongest drag + Magnus truth)
for name in ("T2_10g_cd.6", "T3_10g_cd.6_cl.30"):
    kq, kl = sc.truth_consts(name)
    v = rng.uniform(250, 600, 200)
    th = rng.uniform(30, 80, 200)
    r2 = sc.crossings(*sc.integrate(v, np.radians(th), kq, kl, dt=0.002), Ds, kq)
    r0 = sc.crossings(*sc.integrate(v, np.radians(th), kq, kl, dt=0.00025), Ds, kq)
    m = r2["ok"] & r0["ok"]
    out.append(f"[2] {name}: dt 2ms vs 0.25ms max|dw|={np.nanmax(np.abs(r2['w'][m]-r0['w'][m])):.2e} in, "
               f"max|dtof|={1e3*np.nanmax(np.abs(r2['tof'][m]-r0['tof'][m])):.3f} ms, max|dlat|={1e3*np.nanmax(np.abs(r2['lat'][m]-r0['lat'][m])):.3f} ms, ok mismatch={int((r2['ok']!=r0['ok']).sum())}")

# 3. energy: drag only -> mechanical energy strictly decreasing; Magnus adds no work (compare T3 vs drag-only energy loss rate)
for name in ("T2_10g_cd.6", "T3_10g_cd.6_cl.30"):
    kq, kl = sc.truth_consts(name)
    t, X, Z, VX, VZ = sc.integrate(np.array([450.0]), np.radians(np.array([50.0])), kq, kl, dt=0.001, tmax=1.0)
    E = 0.5 * (VX ** 2 + VZ ** 2) + sc.G * Z
    dE = np.diff(E[:, 0])
    # expected power of drag per unit mass: -kq |v|^3
    sp = np.sqrt(VX ** 2 + VZ ** 2)[:, 0]
    Pd = -kq * sp ** 3
    pred = 0.5 * (Pd[1:] + Pd[:-1]) * 0.001
    out.append(f"[3] {name}: energy monotone decreasing={bool(np.all(dE < 0))}; max |dE - drag work|/|dE| = {np.max(np.abs(dE-pred)/np.abs(dE)):.2e}")

# 4. closed-form linear drag (physmodel.pos) vs numerical integration of a = -k v - g
k = 0.8
v0, th0 = 400.0, math.radians(55)
dt = 1e-5
x, z, vx, vz = 0.0, sc.EXIT_Z, v0 * math.cos(th0), v0 * math.sin(th0)
T = 0.6
for _ in range(int(T / dt)):
    ax, az = -k * vx, -sc.G - k * vz
    x += vx * dt + 0.5 * ax * dt * dt
    z += vz * dt + 0.5 * az * dt * dt
    vx += ax * dt
    vz += az * dt
xp, zp, vzp = pm.pos(np.array(T), np.array(v0), np.array(55.0), np.array(k))
out.append(f"[4] linear-drag closed form vs Euler(1e-5): dx={float(xp)-x:.3e} dz={float(zp)-z:.3e} dvz={float(vzp)-vz:.3e}")
# z_at_x consistency
zx = pm.z_at_x(np.array(float(xp)), np.array(v0), np.array(55.0), np.array(k))
out.append(f"    z_at_x(x(T)) - z(T) = {float(zx)-float(zp):.2e}")

# 5. field interpolation error: trilinear lookup of cached field vs direct integration at random off-grid points
for name in ("T1_vac", "T2_10g_cd.6", "T3_10g_cd.6_cl.30"):
    F = sc.load_field(name)
    kq, kl = sc.truth_consts(name)
    n = 400
    v = np.exp(rng.uniform(math.log(200), math.log(560), n))
    th = rng.uniform(30, 80, n)
    D = rng.uniform(36, 132, n)
    t, X, Z, VX, VZ = sc.integrate(v, np.radians(th), kq, kl)
    dw, dt_, dvz, flag = [], [], [], 0
    for i in range(n):
        c1 = sc.crossings(t, X[:, i:i + 1], Z[:, i:i + 1], VX[:, i:i + 1], VZ[:, i:i + 1], [D[i]], kq)
        wi = sc.trilinear(F["w"], D[i:i + 1], v[i:i + 1], th[i:i + 1])
        if c1["ok"][0, 0] and np.isfinite(wi[0]):
            dw.append(abs(wi[0] - c1["w"][0, 0]))
            dvz.append(abs(sc.trilinear(F["vz"], D[i:i + 1], v[i:i + 1], th[i:i + 1])[0] - c1["vz"][0, 0]))
            dt_.append(abs(sc.trilinear(F["tof"], D[i:i + 1], v[i:i + 1], th[i:i + 1])[0] - c1["tof"][0, 0]))
        elif c1["ok"][0, 0] != np.isfinite(wi[0]):
            flag += 1
    dw = np.array(dw)
    out.append(f"[5] {name}: trilinear vs direct, n={len(dw)}: |dw| p50={np.median(dw):.3f} p99={np.percentile(dw,99):.3f} max={dw.max():.3f} in; "
               f"|dvz| max={max(dvz):.2f} in/s; |dtof| max={1e3*max(dt_):.2f} ms; finite-flag mismatches={flag}")
    del F

txt = "\n".join(out)
print(txt)
open(os.path.join(HERE, "check_numerics.txt"), "w").write(txt)
