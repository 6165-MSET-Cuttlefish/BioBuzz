"""Calibration simulation (a tuner who stops when shots go in) and the shot representations A-F."""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import shotcore as sc
import physmodel as pm
from evalcore import run_extent

R = sc.R_GRID
H = sc.H_GRID

SPACINGS = {
    "even4": [36, 68, 100, 132],
    "even5": [36, 60, 84, 108, 132],
    "even6": [36, 55.2, 74.4, 93.6, 112.8, 132],
    "even8": list(np.round(np.linspace(36, 132, 8), 2)),
    "uneven4": [36, 50, 80, 132],
    "uneven5": [36, 46, 62, 90, 132],
    "uneven6": [36, 44, 56, 74, 100, 132],
    "uneven8": [36, 42, 50, 60, 72, 88, 108, 132],
}


def slice_S(T, D, gain=sc.K_GAIN):
    rr, hh = np.meshgrid(R, H, indexing="ij")
    return T.scored(np.full(rr.shape, float(D)), rr, hh, gain)


def run_of(vec, idx):
    """[lo, hi] of the True run in a 1-D boolean vector containing idx, or the nearest run."""
    if not vec.any():
        return None
    if not vec[idx]:
        t = np.where(vec)[0]
        idx = int(t[np.argmin(np.abs(t - idx))])
    lo = idx
    while lo > 0 and vec[lo - 1]:
        lo -= 1
    hi = idx
    while hi < len(vec) - 1 and vec[hi + 1]:
        hi += 1
    return lo, hi


class CalSite:
    """Everything about one calibration distance under one truth."""

    def __init__(self, T, D, sdist_cache=None):
        self.D = D
        self.S = slice_S(T, D)
        rr, hh = np.meshgrid(R, H, indexing="ij")
        P = np.where(self.S, T.P(np.full(rr.shape, float(D)), rr, hh), -1)
        i, j = np.unravel_index(int(np.argmax(P)), P.shape)
        self.ref = {"maxP": (int(i), int(j))}
        cols = np.where(self.S.any(0))[0]
        self.cmin, self.cmax = int(cols.min()), int(cols.max())
        if sdist_cache is not None:
            self.ref["minDist"] = sdist_cache

    def ridge_row(self, j):
        col = self.S[:, j]
        d, u = run_extent(col[:, None], 0)
        width = (d + u + 1)[:, 0] * col
        i = int(np.argmax(width))
        return (i - d[i, 0] + i + u[i, 0]) // 2

    def draw(self, schedule, rng, frac=0.6):
        if schedule == "wander":
            j = int(round(self.cmin + (0.5 - frac / 2 + frac * rng.random()) * (self.cmax - self.cmin)))
            while not self.S[:, j].any():
                j += 1
            i = self.ridge_row(j)
        else:
            i, j = self.ref[schedule]
        lo, hi = run_of(self.S[:, j], i)
        f = 0.5 - frac / 2 + frac * rng.random()
        rpm = R[lo] + f * (R[hi] - R[lo])
        ii = int(round((rpm - R[0]) / (R[1] - R[0])))
        span = run_of(self.S[ii], j)
        f = 0.5 - frac / 2 + frac * rng.random()
        hood = H[span[0]] + f * (H[span[1]] - H[span[0]])
        return float(rpm), float(hood)


def min_dist_ref(T, D):
    """The minDistSens point (as Study 1) at an arbitrary distance."""
    import study1
    S = slice_S(T, D)
    rows = np.where(S.any(1))[0]
    sub = slice(max(rows.min() - 2, 0), min(rows.max() + 3, len(R)))
    dt = study1.dist_tolerance(T, D, S[sub], np.arange(len(R))[sub])
    rr, hh = np.meshgrid(R[sub], H, indexing="ij")
    P = T.P(np.full(rr.shape, float(D)), rr, hh)
    score = np.where(S[sub], dt + 1e-3 * P, -1)
    i, j = np.unravel_index(int(np.argmax(score)), score.shape)
    return int(i + sub.start), int(j)


# ----------------------------------------------------------------------------- representations
def pchip_slopes(x, y):
    h = np.diff(x)
    d = np.diff(y) / h
    n = len(x)
    m = np.zeros(n)
    for k in range(1, n - 1):
        if d[k - 1] * d[k] <= 0:
            m[k] = 0
        else:
            w1 = 2 * h[k] + h[k - 1]
            w2 = h[k] + 2 * h[k - 1]
            m[k] = (w1 + w2) / (w1 / d[k - 1] + w2 / d[k])

    def end(h0, h1, d0, d1):
        e = ((2 * h0 + h1) * d0 - h0 * d1) / (h0 + h1)
        if np.sign(e) != np.sign(d0):
            e = 0.0
        elif np.sign(d0) != np.sign(d1) and abs(e) > abs(3 * d0):
            e = 3 * d0
        return e

    if n == 2:
        m[:] = d[0]
    else:
        m[0] = end(h[0], h[1], d[0], d[1])
        m[-1] = end(h[-1], h[-2], d[-1], d[-2])
    return m


def pchip(x, y, xq):
    x = np.asarray(x, float)
    y = np.asarray(y, float)
    m = pchip_slopes(x, y)
    xq = np.clip(np.asarray(xq, float), x[0], x[-1])
    k = np.clip(np.searchsorted(x, xq) - 1, 0, len(x) - 2)
    h = x[k + 1] - x[k]
    t = (xq - x[k]) / h
    h00 = 2 * t ** 3 - 3 * t ** 2 + 1
    h10 = t ** 3 - 2 * t ** 2 + t
    h01 = -2 * t ** 3 + 3 * t ** 2
    h11 = t ** 3 - t ** 2
    return h00 * y[k] + h10 * h * m[k] + h01 * y[k + 1] + h11 * h * m[k + 1]


def polyrep(x, y, deg, xq):
    deg = min(deg, len(x) - 1)
    c = np.polyfit((np.asarray(x) - 84) / 48, y, deg)
    return np.polyval(c, (np.clip(xq, min(x), max(x)) - 84) / 48)


FITS = {
    "D2vac": dict(free=["k", "th0"], base=dict()),
    "D3lin": dict(free=["k", "th0", "kd"], base=dict(kd=0.3)),
    "D4lin": dict(free=["k", "th0", "th1", "kd"], base=dict(kd=0.3)),
}


def fit_physics(kind, Dc, rc, hc, tofc=None, tof_weight=0.0):
    p0 = dict(pm.NOMINAL)
    p0.update(FITS[kind]["base"])
    p0["k"] = sc.K_GAIN * 0.97
    p, res = pm.fit(p0, FITS[kind]["free"], Dc, rc, hc, tof=tofc, tof_weight=tof_weight)
    return p, res


def independent_params(truth_name, rng):
    """Constants measured without shooting at the goal: exit speed by video at mid hood (sigma 3%), hood angle by
    protractor at h = 0, 0.5, 1 (sigma 1 deg each), drag from measured mass with Cd assumed 0.5, no Magnus."""
    t = sc.TRUTHS[truth_name]
    p = dict(pm.NOMINAL)
    p["k"] = sc.K_GAIN * float(sc.eta(0.5)) * (1 + 0.03 * rng.standard_normal())
    p["eta"] = 0.0
    a0, a5, a1 = (float(sc.hood_to_theta(h)) + rng.standard_normal() for h in (0.0, 0.5, 1.0))
    p["th0"] = a0
    p["th1"] = a1 - a0
    p["th2"] = 4 * (a5 - 0.5 * (a0 + a1))
    if t["mass"] is None:
        p["kd"], p["kdv"] = 0.0, 0.0
    else:
        kq_assumed = sc.quad_k_per_in(t["mass"], 0.5)
        p["kd"], p["kdv"] = 0.0, 0.8 * kq_assumed
    return p


class Reps:
    """Builds every representation from one calibration set; .settings(name, D) -> (rpm, hood)."""

    NAMES = ["A_lin", "B_pchip", "C2_quad", "C3_cubic", "D2vac", "D3lin", "D4lin", "E_indep", "F_hybrid"]

    def __init__(self, Dc, rc, hc, truth_name, rng, tofc=None, eparams=None):
        self.Dc = np.asarray(Dc, float)
        self.rc = np.asarray(rc, float)
        self.hc = np.asarray(hc, float)
        self.p = {}
        self.res = {}
        for k in FITS:
            self.p[k], self.res[k] = fit_physics(k, self.Dc, self.rc, self.hc)
        if tofc is not None:
            self.p["D3lin+tof"], self.res["D3lin+tof"] = fit_physics("D3lin", self.Dc, self.rc, self.hc, tofc, 200.0)
        self.p["E_indep"] = eparams if eparams is not None else independent_params(truth_name, rng)
        self.resid_F = self.rc - pm.solve_rpm(self.p["D3lin"], self.Dc, self.hc)

    def hood(self, D):
        return np.interp(D, self.Dc, self.hc)

    def settings(self, name, D, rpm_scale=1.0, rpm_add=0.0):
        D = np.asarray(D, float)
        if name == "A_lin":
            r, h = np.interp(D, self.Dc, self.rc), self.hood(D)
        elif name == "B_pchip":
            r, h = pchip(self.Dc, self.rc, D), pchip(self.Dc, self.hc, D)
        elif name == "C2_quad":
            r, h = polyrep(self.Dc, self.rc, 2, D), polyrep(self.Dc, self.hc, 2, D)
        elif name == "C3_cubic":
            r, h = polyrep(self.Dc, self.rc, 3, D), polyrep(self.Dc, self.hc, 3, D)
        elif name in self.p:
            h = self.hood(D)
            r = pm.solve_rpm(self.p[name], D, h)
        elif name == "F_hybrid":
            h = self.hood(D)
            r = pm.solve_rpm(self.p["D3lin"], D, h) + np.interp(D, self.Dc, self.resid_F)
        else:
            raise KeyError(name)
        return r * rpm_scale + rpm_add, h
