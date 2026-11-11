"""Opening-plane crossing for drag-free or quadratic-drag flight, in the opening's local frame (see geom.py)."""
import math
import numpy as np
from geom import *  # noqa


def launch_point(d, phi_deg):
    p = math.radians(phi_deg)
    return np.array([AIM[0] - d * math.sin(p), AIM[1] - d * math.cos(p), EXIT_Z])


def launch_vel(v, th, phi_deg):
    p = math.radians(phi_deg)
    vh = v * math.cos(th)
    return np.array([vh * math.sin(p), vh * math.cos(p), v * math.sin(th)])


def traj(p0, v0, kq, tmax=3.0, dt=1e-4):
    """RK4 with quadratic drag a = -kq |v| v - g z (kq in 1/in). Returns arrays t, P, V."""
    n = int(tmax / dt)
    P = np.empty((n + 1, 3)); V = np.empty((n + 1, 3))
    P[0] = p0; V[0] = v0
    gvec = np.array([0, 0, -G])

    def acc(v):
        return gvec - kq * np.linalg.norm(v) * v

    p = p0.copy(); v = v0.copy()
    for i in range(n):
        if kq == 0:
            P[i + 1] = p + v * dt + 0.5 * gvec * dt * dt
            V[i + 1] = v + gvec * dt
        else:
            k1v = acc(v); k1p = v
            k2v = acc(v + 0.5 * dt * k1v); k2p = v + 0.5 * dt * k1v
            k3v = acc(v + 0.5 * dt * k2v); k3p = v + 0.5 * dt * k2v
            k4v = acc(v + dt * k3v); k4p = v + dt * k3v
            P[i + 1] = p + dt / 6 * (k1p + 2 * k2p + 2 * k3p + k4p)
            V[i + 1] = v + dt / 6 * (k1v + 2 * k2v + 2 * k3v + k4v)
        p = P[i + 1]; v = V[i + 1]
        if p[2] < 0:
            return np.arange(i + 2) * dt, P[:i + 2], V[:i + 2]
    return np.arange(n + 1) * dt, P, V


def crossing_nodrag(p0, v0):
    """Inward crossings of the opening plane (side goes + to -). Exact quadratic."""
    # s(t) = N.(p0 - O) + N.v0 t - 0.5 G N_z t^2
    a = -0.5 * G * N[2]; b = N @ v0; c = N @ (p0 - O)
    disc = b * b - 4 * a * c
    if disc < 0:
        return None, c
    roots = sorted([(-b - math.sqrt(disc)) / (2 * a), (-b + math.sqrt(disc)) / (2 * a)])
    out = []
    for t in roots:
        if t <= 0:
            continue
        ds = b + 2 * a * t
        if ds < 0:
            out.append(t)
    if not out:
        return None, c
    t = out[0]
    p = p0 + v0 * t + np.array([0, 0, -0.5 * G * t * t])
    v = v0 + np.array([0, 0, -G * t])
    return (t, p, v), c


def entry_eval(p, v, rim=RIM, dia=POLLEN_D):
    dp = p - O
    u = dp @ U; w = dp @ Wv
    sp = np.linalg.norm(v)
    inc = -(v @ N) / sp
    margin = dia / 2 / max(inc, 0.33) + rim
    return u, w, inc, margin, edge_slack(u, w, margin), v[2] < 0
