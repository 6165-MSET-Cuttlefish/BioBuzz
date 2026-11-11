"""Shot model for the CAD shooter geometry.

2-D vertical plane through the turret axis and the aim point (azimuth 0, no sideways exit offset).
x: horizontal from the turret axis toward the CELL; z: height above the TILES (in).

Opening geometry, centroid, inside() rule and margin as in ../../ballistics/geom.py and cross.py
(= BioBuzzField.java CellOpening, ShotSolver.checkEntry); RK4 with quadratic drag + Magnus as in
../../shotcalc/shotcore.py.
"""
import math
import os
import numpy as np

G = 386.09                                  # ShotSolver.GRAVITY_IN_PER_S2
TILT = math.radians(30.0)                   # BioBuzzField CELL_TILT_RAD
Z_BOTTOM = 53.5                             # OPENING_BOTTOM_HEIGHT_IN
W_OPEN, SHOULDER, H_OPEN = 20.0, 7.61, 14.0
AIM_DEPTH = 3.0                             # ShotTuning.aimDepthIn
RIM = float(os.environ.get("RIM_CLEARANCE_IN", "1.0"))   # ShotTuning.rimClearanceIn (TODO placeholder in the code)
SIN, COS = math.sin(TILT), math.cos(TILT)
SLANT = math.hypot(W_OPEN / 2, H_OPEN - SHOULDER)


def centroid_w():
    rect = W_OPEN * SHOULDER
    tri = W_OPEN * (H_OPEN - SHOULDER) / 2
    tric = SHOULDER + (H_OPEN - SHOULDER) / 3
    return (rect * SHOULDER / 2 + tri * tric) / (rect + tri)


WC = centroid_w()
# (x, z): w axis up the face leaning into the CELL = (SIN, COS); n out of the opening toward the shooter = (-COS, SIN)
AIM_DX = WC * SIN + AIM_DEPTH * COS         # aim point minus opening bottom-centre, horizontal (5.378 in)
Z_AIM = Z_BOTTOM + WC * COS - AIM_DEPTH * SIN   # 56.815 in

RHO = 1.204                                 # kg/m^3, 20 C sea level [assumed]
IN = 0.0254
RPM_TO_INPS = math.pi * 72e-3 / IN / 60.0   # wheel surface speed per rpm for the 72 mm Rhino wheel (in/s per rpm)

SHOOTERS = {
    "pollen": dict(dia=2.8, mass=(23.5, 24.9, 27.2), mmax=30.0, base={"geo": 57.7, "cal": 55.9},
                   exit=((0.0, 1.56, 12.46), (30.0, 1.99, 13.83))),
    "nectar": dict(dia=3.6, mass=(39.0, 41.3, 45.0), mmax=20.0, base={"geo": 60.8, "cal": 57.4},
                   exit=((0.0, 1.23, 12.53), (20.0, 1.60, 13.57))),
}
DRAG_CASES = ("none", "low", "nom", "high")


def kq_per_in(mass_g, cd, dia_in):
    area = math.pi * (dia_in * IN / 2) ** 2
    return RHO * cd * area / (2 * mass_g / 1000.0) * IN


def drag_consts(shooter, case, cl=0.0):
    """(kq, kl) in 1/in. low = heaviest ball, Cd 0.4; nom = nominal mass, Cd 0.5; high = lightest, Cd 0.6.
    cl > 0 is backspin (lift up), cl < 0 topspin (lift down); kl = kq * cl / cd."""
    s = SHOOTERS[shooter]
    if case == "none":
        return 0.0, 0.0
    mass, cd = {"low": (s["mass"][2], 0.4), "nom": (s["mass"][1], 0.5), "high": (s["mass"][0], 0.6)}[case]
    kq = kq_per_in(mass, cd, s["dia"])
    return kq, kq * cl / cd


def exit_point(shooter, m):
    """Ball-centre exit (forward of the turret axis, height) for hood offset m (deg); linear between the CAD end points,
    extrapolated outside the travel."""
    (m0, x0, z0), (m1, x1, z1) = SHOOTERS[shooter]["exit"]
    f = (np.asarray(m, float) - m0) / (m1 - m0)
    return x0 + f * (x1 - x0), z0 + f * (z1 - z0)


def margin(inc, dia, rim=RIM):
    return dia / 2 / np.maximum(inc, 0.33) + rim


def slack(w, inc, dia, rim=RIM):
    """Signed slack (in) of the ball centre inside the outline less the margin, at u = 0."""
    mg = margin(inc, dia, rim)
    return np.minimum(w - mg, (H_OPEN - w) * (W_OPEN / 2) / SLANT - mg)


def _acc(vx, vz, kq, kl):
    sp = np.sqrt(vx * vx + vz * vz)
    return -kq * sp * vx - kl * sp * vz, -G - kq * sp * vz + kl * sp * vx


def simulate(v, th_deg, x0, z0, D, kq=0.0, kl=0.0, dt=0.002, tmax=4.0, keep=False):
    """Fly balls from (x0, z0) at speed v, angle th; target aim point at horizontal D.
    Returns dict: plane crossing (first inward crossing of the opening plane): tp, w, vxp, vzp, cross;
    aim-x crossing (x = D): ta, za, vza.  keep=True also returns the trajectory arrays."""
    v, th, x0, z0, D = np.broadcast_arrays(*(np.asarray(a, float) for a in (v, th_deg, x0, z0, D)))
    n = v.size
    v, th, x0, z0, D = (a.ravel().copy() for a in (v, th, x0, z0, D))
    thr = np.radians(th)
    x, z = x0.copy(), z0.copy()
    vx, vz = v * np.cos(thr), v * np.sin(thr)
    xo = D - AIM_DX
    side = -COS * (x - xo) + SIN * (z - Z_BOTTOM)
    out = {k: np.full(n, np.nan) for k in ("tp", "w", "xp", "zp", "vxp", "vzp", "ta", "za", "vza", "vxa")}
    pdone = np.zeros(n, bool)
    adone = np.zeros(n, bool)
    t = 0.0
    if keep:
        T, X, Z, VX, VZ = [0.0], [x.copy()], [z.copy()], [vx.copy()], [vz.copy()]
    drag = (kq != 0.0) or (kl != 0.0)
    while t < tmax:
        if drag:
            a1x, a1z = _acc(vx, vz, kq, kl)
            v2x, v2z = vx + 0.5 * dt * a1x, vz + 0.5 * dt * a1z
            a2x, a2z = _acc(v2x, v2z, kq, kl)
            v3x, v3z = vx + 0.5 * dt * a2x, vz + 0.5 * dt * a2z
            a3x, a3z = _acc(v3x, v3z, kq, kl)
            v4x, v4z = vx + dt * a3x, vz + dt * a3z
            a4x, a4z = _acc(v4x, v4z, kq, kl)
            xn = x + dt / 6 * (vx + 2 * v2x + 2 * v3x + v4x)
            zn = z + dt / 6 * (vz + 2 * v2z + 2 * v3z + v4z)
            vxn = vx + dt / 6 * (a1x + 2 * a2x + 2 * a3x + a4x)
            vzn = vz + dt / 6 * (a1z + 2 * a2z + 2 * a3z + a4z)
        else:
            xn = x + vx * dt
            zn = z + vz * dt - 0.5 * G * dt * dt
            vxn = vx
            vzn = vz - G * dt
        sn = -COS * (xn - xo) + SIN * (zn - Z_BOTTOM)
        hit = (~pdone) & (side > 0) & (sn <= 0)
        if hit.any():
            f = side[hit] / (side[hit] - sn[hit])
            px = x[hit] + f * (xn[hit] - x[hit])
            pz = z[hit] + f * (zn[hit] - z[hit])
            out["tp"][hit] = t + f * dt
            out["xp"][hit] = px
            out["zp"][hit] = pz
            out["w"][hit] = (px - xo[hit]) * SIN + (pz - Z_BOTTOM) * COS
            out["vxp"][hit] = vx[hit] + f * (vxn[hit] - vx[hit])
            out["vzp"][hit] = vz[hit] + f * (vzn[hit] - vz[hit])
            pdone |= hit
        ahit = (~adone) & (x < D) & (xn >= D)
        if ahit.any():
            f = (D[ahit] - x[ahit]) / (xn[ahit] - x[ahit])
            out["ta"][ahit] = t + f * dt
            out["za"][ahit] = z[ahit] + f * (zn[ahit] - z[ahit])
            out["vza"][ahit] = vz[ahit] + f * (vzn[ahit] - vz[ahit])
            out["vxa"][ahit] = vx[ahit] + f * (vxn[ahit] - vx[ahit])
            adone |= ahit
        x, z, vx, vz, side = xn, zn, vxn, vzn, sn
        t += dt
        if keep:
            T.append(t); X.append(x.copy()); Z.append(z.copy()); VX.append(vx.copy()); VZ.append(vz.copy())
        fallen = zn < -5
        pdone |= fallen
        adone |= fallen
        if pdone.all() and adone.all() and not keep:
            break
        if keep and (zn < -5).all():
            break
    out["cross"] = ~np.isnan(out["tp"])
    if keep:
        out["traj"] = (np.array(T), np.array(X), np.array(Z), np.array(VX), np.array(VZ))
    return out


def score_of(res, dia, rim=RIM):
    """ShotSolver.checkEntry rule: inward crossing, descending, centre inside outline less r/max(cos inc,.33) + rim."""
    vxp, vzp = res["vxp"], res["vzp"]
    sp = np.hypot(vxp, vzp)
    inc = (vxp * COS - vzp * SIN) / sp          # -(v . n)/|v|, n = (-COS, SIN)
    sl = slack(res["w"], inc, dia, rim)
    ok = res["cross"] & (vzp < 0) & (sl >= 0)
    return np.where(np.isnan(sl), False, ok), sl, inc


def v_aim_nodrag(th_deg, x0, z0, D):
    th = np.radians(th_deg)
    d = D - x0
    h = Z_AIM - z0
    den = 2 * np.cos(th) ** 2 * (d * np.tan(th) - h)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(den > 0, np.sqrt(G * d * d / np.where(den > 0, den, 1)), np.nan)


def v_aim(th_deg, x0, z0, D, kq=0.0, kl=0.0, iters=40, tol=1e-5, vcap=1200.0):
    """Exit speed that passes the aim point (x = D, z = Z_AIM) at launch angle th from (x0, z0); NaN if none <= vcap."""
    th_deg, x0, z0, D = np.broadcast_arrays(*(np.asarray(a, float) for a in (th_deg, x0, z0, D)))
    shape = th_deg.shape
    th_deg, x0, z0, D = (a.ravel() for a in (th_deg, x0, z0, D))
    v0 = v_aim_nodrag(th_deg, x0, z0, D)
    if kq == 0 and kl == 0:
        return v0.reshape(shape)
    res = np.full(v0.shape, np.nan)
    act = np.where(~np.isnan(v0) & (v0 < vcap) if kl <= 0 else (th_deg > 1))[0]
    if act.size == 0:
        return res.reshape(shape)

    def f(v, ix):
        r = simulate(v, th_deg[ix], x0[ix], z0[ix], D[ix], kq, kl)
        return np.where(np.isnan(r["za"]), -500.0, r["za"] - Z_AIM)
    vv = np.where(np.isnan(v0[act]), 300.0, v0[act])
    lo = vv * (0.95 if kl <= 0 else 0.7)
    hi = np.minimum(vv * 1.25, vcap)
    flo = f(lo, act)
    fhi = f(hi, act)
    for _ in range(4):
        bad = fhi < 0
        if not bad.any():
            break
        hi = np.where(bad, np.minimum(hi * 1.4, vcap), hi)
        fhi[bad] = f(hi[bad], act[bad])
    for _ in range(4):
        bad = flo > 0
        if not bad.any():
            break
        lo = np.where(bad, lo * 0.8, lo)
        flo[bad] = f(lo[bad], act[bad])
    keep = (flo < 0) & (fhi > 0)
    act, lo, hi, flo, fhi = act[keep], lo[keep], hi[keep], flo[keep], fhi[keep]
    prev = np.zeros(act.size, int)
    for _ in range(iters):
        if act.size == 0:
            break
        den = fhi - flo
        m = np.where(den != 0, hi - fhi * (hi - lo) / np.where(den != 0, den, 1), 0.5 * (lo + hi))
        m = np.where((m <= lo) | (m >= hi), 0.5 * (lo + hi), m)
        fm = f(m, act)
        done = np.abs(fm) < tol
        res[act[done]] = m[done]
        left = fm < 0
        fhi = np.where(left & (prev == -1), fhi * 0.5, fhi)
        flo = np.where(~left & (prev == 1), flo * 0.5, flo)
        lo = np.where(left, m, lo)
        flo = np.where(left, fm, flo)
        hi = np.where(left, hi, m)
        fhi = np.where(left, fhi, fm)
        prev = np.where(left, -1, 1)
        nd = ~done
        act, lo, hi, flo, fhi, prev = act[nd], lo[nd], hi[nd], flo[nd], fhi[nd], prev[nd]
    return res.reshape(shape)


def rpm_for(v_inps, eta):
    return v_inps / (eta * RPM_TO_INPS)


def vmin_search(exit_fn, D, kq=0.0, kl=0.0, lo=35.0, hi=89.5):
    """Minimum aim-point speed over launch angle (unconstrained). exit_fn(th) -> (x0, z0). Returns (v, th)."""
    th = np.arange(lo, hi + 1e-9, 0.5)
    x0, z0 = exit_fn(th)
    v = v_aim(th, np.broadcast_to(x0, th.shape), np.broadcast_to(z0, th.shape), D, kq, kl)
    i = int(np.nanargmin(v))
    th2 = np.arange(th[i] - 1.0, th[i] + 1.0 + 1e-9, 0.02)
    x0, z0 = exit_fn(th2)
    v2 = v_aim(th2, np.broadcast_to(x0, th2.shape), np.broadcast_to(z0, th2.shape), D, kq, kl)
    j = int(np.nanargmin(v2))
    if 0 < j < len(th2) - 1:
        a, b, c = v2[j - 1], v2[j], v2[j + 1]
        den = a - 2 * b + c
        dx = 0.5 * (a - c) / den if den != 0 else 0.0
        thm = th2[j] + dx * 0.02
        x0, z0 = exit_fn(np.array([thm]))
        vm = v_aim(np.array([thm]), np.broadcast_to(x0, (1,)), np.broadcast_to(z0, (1,)), D, kq, kl)[0]
        return float(vm), float(thm)
    return float(v2[j]), float(th2[j])
