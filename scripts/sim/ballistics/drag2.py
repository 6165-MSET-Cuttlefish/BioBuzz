"""Vectorised quadratic-drag repeat. a = -g z - k |v| v, k = rho Cd A / (2 m), in 1/in."""
import math
import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from geom import *  # noqa

RHO = 1.204  # kg/m^3, 20 C sea level (assumption)
AREA = math.pi * (POLLEN_D * 0.0254) ** 2 / 4
h = AIM[2] - EXIT_Z
DT = 1e-3
DS = [36, 60, 84, 108, 132]


def kq(mass_g, cd):
    return RHO * cd * AREA / (2 * mass_g / 1000) * 0.0254


def rk4(S, k, dt):
    def f(S):
        V = S[3:]
        sp = np.sqrt((V * V).sum(0))
        A = -k * sp * V
        A[2] -= G
        return np.concatenate([V, A])
    k1 = f(S); k2 = f(S + 0.5 * dt * k1); k3 = f(S + 0.5 * dt * k2); k4 = f(S + dt * k3)
    return S + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def fly(S0, k, side, tmax=3.0):
    """S0: (6, n). side(S) -> (n,) ; returns time and state at the first + to - crossing (NaN if none)."""
    n = S0.shape[1]
    S = S0.copy()
    sv = side(S)
    T = np.full(n, np.nan); OUT = np.full((6, n), np.nan)
    done = np.zeros(n, bool)
    t = 0.0
    while t < tmax and not done.all():
        Sn = rk4(S, k, DT)
        sn = side(Sn)
        hit = (~done) & (sv > 0) & (sn <= 0)
        if hit.any():
            a = sv[hit] / (sv[hit] - sn[hit])
            T[hit] = t + a * DT
            OUT[:, hit] = S[:, hit] + a * (Sn[:, hit] - S[:, hit])
            done |= hit
        done |= Sn[2] < -20
        S, sv, t = Sn, sn, t + DT
    return T, OUT


def at_d(v, th, d, k):
    v = np.atleast_1d(np.asarray(v, float)); th = np.broadcast_to(np.atleast_1d(th), v.shape).astype(float)
    S0 = np.vstack([np.zeros_like(v), np.zeros_like(v), np.full_like(v, EXIT_Z), v * np.cos(th), np.zeros_like(v), v * np.sin(th)])
    return fly(S0, k, lambda S: d - S[0])


def solve_v(th, d, k):
    """speed hitting the aim height at horizontal distance d for each launch angle th (vector)."""
    th = np.atleast_1d(th).astype(float)
    # drag-free speed for this angle as the first guess, then secant on z(v) at x = d
    c2 = np.cos(th) ** 2
    denom = 2 * c2 * (d * np.tan(th) - h)
    v0 = np.sqrt(np.where(denom > 0, G * d * d / np.maximum(denom, 1e-9), 900.0 ** 2))
    v0 = np.minimum(v0, 900.0)

    def z(v):
        T, O = at_d(v, th, d, k)
        return np.where(np.isnan(T), -200.0, O[2] - AIM[2])
    va = v0 * 1.02; za = z(va)
    vb = v0 * 1.06; zb = z(vb)
    for _ in range(25):
        dz = zb - za
        vn = np.where(np.abs(dz) > 1e-12, vb - zb * (vb - va) / np.where(np.abs(dz) > 1e-12, dz, 1), vb)
        vn = np.clip(vn, 50, 1500)
        va, za = vb, zb
        vb = vn; zb = z(vb)
        if np.nanmax(np.abs(zb)) < 1e-6:
            break
    return np.where(np.abs(zb) < 1e-3, vb, np.nan)


def vmin(d, k):
    th = np.radians(np.linspace(40, 80, 81))
    v = solve_v(th, d, k)
    i = np.nanargmin(v)
    th2 = np.linspace(th[max(i - 2, 0)], th[min(i + 2, len(th) - 1)], 81)
    v2 = solve_v(th2, d, k)
    j = np.nanargmin(v2)
    return v2[j], th2[j]


def descent(v, th, d, k):
    T, O = at_d(v, th, d, k)
    return np.degrees(np.arctan2(-O[5], O[3])), T


def for_descent(d, beta, k, lo_deg=None):
    th = np.radians(np.linspace(math.degrees(math.atan(h / d)) + 3, 86, 241))
    v = solve_v(th, d, k)
    b, T = descent(v, th, d, k)
    ok = ~np.isnan(b)
    th, v, b = th[ok], v[ok], b[ok]
    i = np.searchsorted(b, beta)
    if i == 0 or i >= len(b):
        return None
    th_lo, th_hi = th[i - 1], th[i]
    fine = np.linspace(th_lo, th_hi, 41)
    vf = solve_v(fine, d, k)
    bf, _ = descent(vf, fine, d, k)
    m = float(np.interp(beta, bf, fine))
    return solve_v(np.array([m]), d, k)[0], m


CASES = [("nodrag", 0.0), ("40g Cd0.45", kq(40, 0.45)), ("25g Cd0.47", kq(25, 0.47)), ("15g Cd0.50", kq(15, 0.5))]
