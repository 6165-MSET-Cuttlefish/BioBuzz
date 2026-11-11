"""Scoring windows around the aim-point shot: exit speed (%), hood (deg, exit point moves with the hood) and distance (in)."""
import math
import numpy as np
from cadcore import (simulate, score_of, v_aim, COS, SIN, Z_BOTTOM, AIM_DX, slack)

S_GRID = np.arange(-0.15, 0.2001, 0.0025)
H_GRID = np.arange(-15.0, 15.001, 0.25)
D_OFF = np.arange(-40.0, 40.001, 0.1)
NREF = 17


def _interval(good, i0):
    """Contiguous True run containing index i0 -> (first, last) indices, or None."""
    if not good[i0]:
        return None
    a = i0
    while a > 0 and good[a - 1]:
        a -= 1
    b = i0
    while b < len(good) - 1 and good[b + 1]:
        b += 1
    return a, b


def analyze(dia, exit_fn, th, D, kq=0.0, kl=0.0, v=None):
    """th: 1-D launch angles (deg). exit_fn(th) -> (x0, z0). Returns dict of per-angle arrays."""
    th = np.asarray(th, float)
    x0, z0 = exit_fn(th)
    x0 = np.broadcast_to(x0, th.shape).astype(float)
    z0 = np.broadcast_to(z0, th.shape).astype(float)
    if v is None:
        v = v_aim(th, x0, z0, D, kq, kl)
    n = th.size
    nom = simulate(v, th, x0, z0, D, kq, kl, keep=True)
    ok, sl, inc = score_of(nom, dia)
    out = dict(th=th, v=v, x0=x0, z0=z0, ok=ok, slack=sl, inc=inc, w=nom["w"], tp=nom["tp"], ta=nom["ta"],
               vzp=nom["vzp"], vxp=nom["vxp"], descent=np.degrees(np.arctan2(-nom["vzp"], nom["vxp"])),
               vza=nom["vza"], descent_aim=np.degrees(np.arctan2(-nom["vza"], nom["vxa"])))
    for k in ("s_lo", "s_hi", "h_lo", "h_hi", "d_lo", "d_hi"):
        out[k] = np.full(n, np.nan)
    idx = np.where(ok & ~np.isnan(v))[0]
    if idx.size == 0:
        return out
    # ---- coarse scans: speed and hood
    ns, nh = len(S_GRID), len(H_GRID)
    vv = np.concatenate([np.outer(v[idx], 1 + S_GRID).ravel(), np.repeat(v[idx], nh)])
    tt = np.concatenate([np.repeat(th[idx], ns), (th[idx][:, None] + H_GRID[None, :]).ravel()])
    xe, ze = exit_fn(tt)
    xe = np.broadcast_to(xe, tt.shape)
    ze = np.broadcast_to(ze, tt.shape)
    r = simulate(vv, tt, xe, ze, D, kq, kl)
    good = score_of(r, dia)[0]
    gs = good[:idx.size * ns].reshape(idx.size, ns)
    gh = good[idx.size * ns:].reshape(idx.size, nh)
    i0s = int(np.argmin(np.abs(S_GRID)))
    i0h = int(np.argmin(np.abs(H_GRID)))
    # ---- refine each edge between the last-in and first-out grid points
    jobs = []   # (row, kind, a_in, b_out)
    for k, j in enumerate(idx):
        for kind, g, grid, i0 in (("s", gs[k], S_GRID, i0s), ("h", gh[k], H_GRID, i0h)):
            iv = _interval(g, i0)
            if iv is None:
                continue
            a, b = iv
            jobs.append((j, kind + "_lo", grid[a], grid[a - 1] if a > 0 else None))
            jobs.append((j, kind + "_hi", grid[b], grid[b + 1] if b < len(grid) - 1 else None))
    rv, rt, meta = [], [], []
    for j, key, xin, xout in jobs:
        if xout is None:
            out[key][j] = xin
            continue
        fr = np.linspace(xin, xout, NREF)
        meta.append((j, key, fr))
        if key[0] == "s":
            rv.append(v[j] * (1 + fr)); rt.append(np.full(NREF, th[j]))
        else:
            rv.append(np.full(NREF, v[j])); rt.append(th[j] + fr)
    if rv:
        rv = np.concatenate(rv); rt = np.concatenate(rt)
        xe, ze = exit_fn(rt)
        r2 = simulate(rv, rt, np.broadcast_to(xe, rt.shape), np.broadcast_to(ze, rt.shape), D, kq, kl)
        g2 = score_of(r2, dia)[0].reshape(len(meta), NREF)
        for q, (j, key, fr) in enumerate(meta):
            gq = g2[q]
            last = 0
            while last + 1 < NREF and gq[last + 1]:
                last += 1
            out[key][j] = fr[last]
    # ---- distance window from the stored nominal trajectories (target moved by delta, trajectory fixed)
    T, X, Z, VX, VZ = nom["traj"]
    xo0 = D - AIM_DX
    for j in idx:
        side0 = -COS * (X[:, j] - xo0) + SIN * (Z[:, j] - Z_BOTTOM)
        cm = np.minimum.accumulate(side0)
        thr = -COS * D_OFF                     # crossing for target D + delta when side0 <= -COS*delta
        k1 = np.searchsorted(-cm, -thr, side="left")
        valid = (k1 >= 1) & (k1 < len(T))
        k1c = np.clip(k1, 1, len(T) - 1)
        k0 = k1c - 1
        s0, s1 = side0[k0] - thr, side0[k1c] - thr
        f = np.clip(s0 / np.where(s0 - s1 == 0, 1, s0 - s1), 0, 1)
        lin = lambda A: A[k0, j] + f * (A[k1c, j] - A[k0, j])
        x, z, vx, vz = lin(X), lin(Z), lin(VX), lin(VZ)
        w = (x - (xo0 + D_OFF)) * SIN + (z - Z_BOTTOM) * COS
        sp = np.hypot(vx, vz)
        inc_ = (vx * COS - vz * SIN) / sp
        g = valid & (vz < 0) & (slack(w, inc_, dia) >= 0)
        iv = _interval(g, int(np.argmin(np.abs(D_OFF))))
        if iv is not None:
            out["d_lo"][j], out["d_hi"][j] = D_OFF[iv[0]], D_OFF[iv[1]]
    out["s_w"] = out["s_hi"] - out["s_lo"]
    out["h_w"] = out["h_hi"] - out["h_lo"]
    out["d_w"] = out["d_hi"] - out["d_lo"]
    return out
