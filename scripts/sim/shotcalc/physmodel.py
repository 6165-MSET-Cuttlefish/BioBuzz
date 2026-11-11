"""The robot-side physics model: closed-form linear drag (kd = 0 is vacuum), as ShotSolver already implements.

Parameters p = dict(k, eta, th0, th1, th2, kd, kdv):
  v  = k * (1 - eta*h) * rpm
  th = th0 + th1*h + th2*h*(1-h)            (deg)
  linear drag rate = kd + kdv * v           (1/s)  (kdv>0 approximates quadratic drag per shot)
Everything vectorised over numpy arrays.
"""
import math

import numpy as np

import shotcore as sc

G = sc.G
S30, C30 = math.sin(sc.TILT), math.cos(sc.TILT)

NOMINAL = dict(k=sc.K_GAIN, eta=0.0, th0=sc.TH_LO, th1=sc.TH_HI - sc.TH_LO, th2=0.0, kd=0.0, kdv=0.0)


def theta(p, h):
    h = np.asarray(h, float)
    return p["th0"] + p["th1"] * h + p["th2"] * h * (1 - h)


def speed(p, rpm, h):
    return p["k"] * (1 - p["eta"] * np.asarray(h, float)) * np.asarray(rpm, float)


def drag(p, v):
    return p["kd"] + p["kdv"] * np.asarray(v, float)


def z_at_x(D, v, th_deg, kd):
    """Height when the ball reaches horizontal distance D (nan if never)."""
    D, v, th_deg, kd = np.broadcast_arrays(*(np.asarray(a, float) for a in (D, v, th_deg, kd)))
    th = np.radians(th_deg)
    c = np.cos(th)
    vx = v * c
    with np.errstate(invalid="ignore", divide="ignore"):
        a = kd * D / vx
        small = kd < 1e-7
        # a + ln(1-a) = -(a^2/2 + a^3/3 + ...)
        kd_safe = np.where(small, 1.0, kd)
        lin = sc.EXIT_Z + D * np.tan(th) + G / kd_safe ** 2 * (a + np.log1p(-np.minimum(a, 1 - 1e-12)))
        vac = sc.EXIT_Z + D * np.tan(th) - G * D ** 2 / (2 * vx ** 2)
        z = np.where(small, vac, lin)
        z = np.where((a >= 1) & ~small, np.nan, z)
    return z


def pos(t, v, th_deg, kd):
    th = np.radians(th_deg)
    vx, vz = v * np.cos(th), v * np.sin(th)
    with np.errstate(invalid="ignore", divide="ignore"):
        small = kd < 1e-7
        kds = np.where(small, 1.0, kd)
        reach = np.where(small, t, (1 - np.exp(-kds * t)) / kds)
        z_lin = sc.EXIT_Z + (vz + G / kds) * reach - G / kds * t
        z_vac = sc.EXIT_Z + vz * t - 0.5 * G * t * t
        z = np.where(small, z_vac, z_lin)
        vzt = np.where(small, vz - G * t, (vz + G / kds) * np.exp(-kds * t) - G / kds)
    return vx * reach, z, vzt


def plane_crossing(D, v, th_deg, kd, n=240, tmax=2.5):
    """First inward crossing of the opening plane: (tof, w, vz). Vectorised over the first axis."""
    D, v, th_deg, kd = np.broadcast_arrays(*(np.asarray(a, float) for a in (D, v, th_deg, kd)))
    shape = D.shape
    D, v, th_deg, kd = (a.ravel() for a in (D, v, th_deg, kd))
    ts = np.linspace(0, tmax, n)[:, None]
    x, z, _ = pos(ts, v[None], th_deg[None], kd[None])
    xo = sc.opening_origin_x(D)[None]
    s = -(x - xo) * C30 + (z - sc.Z_BOTTOM) * S30
    alive = np.minimum.accumulate(z >= 0, axis=0)
    hit = (s <= 0) & alive
    any_hit = hit.any(0)
    i1 = np.where(any_hit, np.argmax(hit, axis=0), 1)
    i1 = np.maximum(i1, 1)
    lo = ts[i1 - 1, 0]
    hi = ts[i1, 0]
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        xm, zm, _ = pos(mid, v, th_deg, kd)
        sm = -(xm - sc.opening_origin_x(D)) * C30 + (zm - sc.Z_BOTTOM) * S30
        out = sm > 0
        lo = np.where(out, mid, lo)
        hi = np.where(out, hi, mid)
    xm, zm, vzm = pos(hi, v, th_deg, kd)
    w = (xm - sc.opening_origin_x(D)) * S30 + (zm - sc.Z_BOTTOM) * C30
    nanm = ~any_hit
    t = np.where(nanm, np.nan, hi)
    w = np.where(nanm, np.nan, w)
    vzm = np.where(nanm, np.nan, vzm)
    return t.reshape(shape), w.reshape(shape), vzm.reshape(shape)


AIM = {"mode": "band"}


def aim_xz(D):
    """Target point. 'solver': ShotSolver's aim (centroid, aimDepthIn=3 inside), at horizontal distance D.
    'band': on the opening plane at the centre of the allowed band (w = W_MID)."""
    D = np.asarray(D, float)
    if AIM["mode"] == "solver":
        return D, np.full(D.shape, sc.Z_AIM)
    return D - sc.AIM_DX + sc.W_MID * S30, np.full(D.shape, sc.Z_BOTTOM + sc.W_MID * C30)


def aim_residual(p, D, rpm, h):
    """Model height at the target's horizontal distance minus the target height (in)."""
    v = speed(p, rpm, h)
    X, Zt = aim_xz(D)
    return z_at_x(X, v, theta(p, h), drag(p, v)) - Zt


def solve_rpm(p, D, h, lo=500.0, hi=12000.0, iters=48):
    """rpm that puts the model trajectory through the target at hood h (bisection; z rises with rpm)."""
    D, h = np.broadcast_arrays(np.asarray(D, float), np.asarray(h, float))
    X, z_target = aim_xz(D)
    a = np.full(D.shape, lo)
    b = np.full(D.shape, hi)
    th = theta(p, h)
    for _ in range(iters):
        m = 0.5 * (a + b)
        v = speed(p, m, h)
        z = z_at_x(X, v, th, drag(p, v))
        up = ~(z >= z_target)       # too low (or unreachable) -> need more rpm
        a = np.where(up, m, a)
        b = np.where(up, b, m)
    return 0.5 * (a + b)


def solve_hood(p, D, rpm, h_guess, span=0.35, n=281):
    """Hood whose model trajectory at this rpm passes through the aim point, the root nearest h_guess."""
    D, rpm, h_guess = np.broadcast_arrays(*(np.asarray(a, float) for a in (D, rpm, h_guess)))
    shape = D.shape
    D, rpm, h_guess = D.ravel(), rpm.ravel(), h_guess.ravel()
    X, Zt = aim_xz(D)
    off = np.linspace(-span, span, n)[:, None]
    hs = h_guess[None] + off
    v = speed(p, rpm[None], hs)
    f = z_at_x(X[None], v, theta(p, hs), drag(p, v)) - Zt[None]
    sign_change = (np.sign(f[1:]) != np.sign(f[:-1])) & np.isfinite(f[1:]) & np.isfinite(f[:-1])
    dist = np.where(sign_change, np.abs(0.5 * (off[1:] + off[:-1])), np.inf)
    k = np.argmin(dist, axis=0)
    found = np.isfinite(dist[k, np.arange(len(D))])
    cols = np.arange(len(D))
    a = hs[k, cols]
    b = hs[k + 1, cols]
    fa = f[k, cols]
    for _ in range(40):
        m = 0.5 * (a + b)
        vm = speed(p, rpm, m)
        fm = z_at_x(X, vm, theta(p, m), drag(p, vm)) - Zt
        same = np.sign(fm) == np.sign(fa)
        a = np.where(same, m, a)
        fa = np.where(same, fm, fa)
        b = np.where(same, b, m)
    out = np.where(found, 0.5 * (a + b), np.nan)
    return out.reshape(shape)


def model_tof(p, D, rpm, h):
    v = speed(p, rpm, h)
    t, w, vz = plane_crossing(D, v, theta(p, h), drag(p, v))
    return t


def lead_time(p, D, rpm, h):
    """Effective lead time for an inherited lateral velocity: (1 - e^{-kd t}) / kd."""
    v = speed(p, rpm, h)
    kd = drag(p, v)
    t = model_tof(p, D, rpm, h)
    with np.errstate(invalid="ignore", divide="ignore"):
        return np.where(kd < 1e-7, t, (1 - np.exp(-kd * t)) / np.where(kd < 1e-7, 1, kd))


def fit(p0, free, D, rpm, h, tof=None, tof_weight=0.0, iters=40, prior=None):
    """Levenberg-Marquardt on aim residuals (in) [+ tof residuals (s) * tof_weight]. Returns (p, residuals)."""
    p = dict(p0)
    x = np.array([p[k] for k in free], float)
    scale = np.array([max(abs(v), 1e-3) for v in x])
    if "kd" in free:
        scale[free.index("kd")] = 0.3
    D, rpm, h = (np.asarray(a, float) for a in (D, rpm, h))

    def resid(xx):
        q = dict(p)
        for k, v in zip(free, xx):
            q[k] = v
        r = aim_residual(q, D, rpm, h)
        r = np.where(np.isfinite(r), r, 50.0)
        if tof is not None and tof_weight > 0:
            rt = model_tof(q, D, rpm, h) - tof
            r = np.concatenate([r, tof_weight * np.where(np.isfinite(rt), rt, 1.0)])
        if prior is not None:
            r = np.concatenate([r, [(xx[free.index(k)] - m) / s for k, (m, s) in prior.items() if k in free]])
        return r

    lam = 1e-2
    r = resid(x)
    cost = r @ r
    for _ in range(iters):
        J = np.empty((len(r), len(x)))
        for j in range(len(x)):
            dx = np.zeros_like(x)
            dx[j] = 1e-5 * scale[j]
            J[:, j] = (resid(x + dx) - r) / dx[j]
        A = J.T @ J
        g = J.T @ r
        improved = False
        for _ in range(8):
            step = np.linalg.solve(A + lam * np.diag(np.diag(A) + 1e-12), -g)
            xn = x + step
            if "kd" in free:
                xn[free.index("kd")] = max(xn[free.index("kd")], 0.0)
            rn = resid(xn)
            cn = rn @ rn
            if cn < cost:
                x, r, cost = xn, rn, cn
                lam = max(lam / 3, 1e-7)
                improved = True
                break
            lam *= 4
        if not improved or np.max(np.abs(step) / scale) < 1e-7:
            break
    for k, v in zip(free, x):
        p[k] = v
    return p, aim_residual(p, D, rpm, h)
