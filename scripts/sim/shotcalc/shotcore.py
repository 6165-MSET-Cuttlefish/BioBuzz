"""Core physics for the shot-calculation study (numpy only).

2-D vertical plane through the launcher and the CELL opening's centre line (u = 0, straight-on shot).
x: horizontal, toward the HIVE, launcher at x = 0.  z: height above the TILES.
Opening geometry from BioBuzzField.java: bottom edge 53.5 in, pentagon 20 x 14 in, shoulders 7.61 in,
normal tilted 30 deg above horizontal pointing back at the shooter.  Aim point = pentagon centroid,
3 in inside (ShotTuning.aimDepthIn).  D = horizontal distance launcher -> aim point (as ShotSolver).
"""
import math
import os
import numpy as np

G = 386.09                      # in/s^2 (ShotSolver.GRAVITY_IN_PER_S2)
TILT = math.radians(30.0)
Z_BOTTOM = 53.5
OPEN_W, OPEN_H, SHOULDER = 20.0, 14.0, 7.61
AIM_DEPTH = 3.0
EXIT_Z = 14.0                   # ShotTuning exitHeightIn default
BALL_D = 2.8
RIM_CLEAR = 1.0
MARGIN = BALL_D / 2 + RIM_CLEAR  # scoring rule: centre inside the outline less the radius and 1 in
SLANT = math.hypot(OPEN_W / 2, OPEN_H - SHOULDER)
W_HI_FACTOR = SLANT / (OPEN_W / 2)   # at u=0, roof distance = (14-w)*10/slant


def centroid_w():
    rect = OPEN_W * SHOULDER
    tri = OPEN_W * (OPEN_H - SHOULDER) / 2
    tri_c = SHOULDER + (OPEN_H - SHOULDER) / 3
    return (rect * SHOULDER / 2 + tri * tri_c) / (rect + tri)


WC = centroid_w()
# unit vectors in (x, z)
W_VEC = (math.sin(TILT), math.cos(TILT))
N_VEC = (-math.cos(TILT), math.sin(TILT))
# aim point relative to opening bottom-centre o: o + WC*w - depth*n
AIM_DX = WC * W_VEC[0] - AIM_DEPTH * N_VEC[0]
AIM_DZ = WC * W_VEC[1] - AIM_DEPTH * N_VEC[1]
Z_AIM = Z_BOTTOM + AIM_DZ


def w_band(margin=MARGIN):
    """Allowed w interval for the ball centre at u = 0."""
    return margin, OPEN_H - margin * W_HI_FACTOR


W_LO, W_HI = w_band()
W_MID = 0.5 * (W_LO + W_HI)
U_HALF = OPEN_W / 2 - MARGIN     # lateral allowance near the bottom of the opening (w < shoulder)


def opening_origin_x(D):
    return D - AIM_DX


# ----------------------------------------------------------------------------- truth models
RHO = 1.2           # kg/m^3, sea-level-ish room air (assumed)
AREA_28 = math.pi * (0.0254 * BALL_D / 2) ** 2


def quad_k_per_in(mass_g, cd, diam_in=BALL_D):
    """Quadratic drag constant kq [1/in]: a = -kq |v| v."""
    area = math.pi * (0.0254 * diam_in / 2) ** 2
    k_per_m = RHO * cd * area / (2 * mass_g / 1000.0)
    return k_per_m * 0.0254


TRUTHS = {
    "T1_vac": dict(kind="T1", mass=None, cd=0.0, cl=0.0),
    "T2_10g_cd.6": dict(kind="T2", mass=10, cd=0.6, cl=0.0),
    "T2_10g_cd.4": dict(kind="T2", mass=10, cd=0.4, cl=0.0),
    "T2_25g_cd.5": dict(kind="T2", mass=25, cd=0.5, cl=0.0),
    "T2_45g_cd.6": dict(kind="T2", mass=45, cd=0.6, cl=0.0),
    "T2_45g_cd.4": dict(kind="T2", mass=45, cd=0.4, cl=0.0),
    "T3_25g_cd.5_cl.15": dict(kind="T3", mass=25, cd=0.5, cl=0.15),
    "T3_25g_cd.5_cl.30": dict(kind="T3", mass=25, cd=0.5, cl=0.30),
    "T3_10g_cd.6_cl.30": dict(kind="T3", mass=10, cd=0.6, cl=0.30),
}
MAIN_TRUTHS = ["T1_vac", "T2_10g_cd.6", "T2_25g_cd.5", "T2_45g_cd.4", "T3_25g_cd.5_cl.30", "T3_10g_cd.6_cl.30"]


def truth_consts(name):
    t = TRUTHS[name]
    if t["mass"] is None:
        return 0.0, 0.0
    kq = quad_k_per_in(t["mass"], t["cd"])
    kl = kq * (t["cl"] / t["cd"]) if t["cd"] > 0 else 0.0
    return kq, kl


def accel(vx, vz, kq, kl):
    sp = np.sqrt(vx * vx + vz * vz)
    ax = -kq * sp * vx - kl * sp * vz
    az = -G - kq * sp * vz + kl * sp * vx
    return ax, az


def integrate(v0, th0, kq, kl, dt=0.002, tmax=1.7, z0=EXIT_Z):
    """RK4 for arrays of launch speed/angle. Returns t, X, Z, VX, VZ with shape (steps+1, n)."""
    v0 = np.asarray(v0, float)
    th0 = np.asarray(th0, float)
    n = v0.size
    steps = int(round(tmax / dt))
    X = np.empty((steps + 1, n))
    Z = np.empty((steps + 1, n))
    VX = np.empty((steps + 1, n))
    VZ = np.empty((steps + 1, n))
    x = np.zeros(n)
    z = np.full(n, z0)
    vx = v0 * np.cos(th0)
    vz = v0 * np.sin(th0)
    X[0], Z[0], VX[0], VZ[0] = x, z, vx, vz
    for i in range(1, steps + 1):
        if kq == 0 and kl == 0:
            x = x + vx * dt
            z = z + vz * dt - 0.5 * G * dt * dt
            vz = vz - G * dt
        else:
            a1x, a1z = accel(vx, vz, kq, kl)
            k1x, k1z = vx, vz
            v2x, v2z = vx + 0.5 * dt * a1x, vz + 0.5 * dt * a1z
            a2x, a2z = accel(v2x, v2z, kq, kl)
            v3x, v3z = vx + 0.5 * dt * a2x, vz + 0.5 * dt * a2z
            a3x, a3z = accel(v3x, v3z, kq, kl)
            v4x, v4z = vx + dt * a3x, vz + dt * a3z
            a4x, a4z = accel(v4x, v4z, kq, kl)
            x = x + dt / 6 * (k1x + 2 * v2x + 2 * v3x + v4x)
            z = z + dt / 6 * (k1z + 2 * v2z + 2 * v3z + v4z)
            vx = vx + dt / 6 * (a1x + 2 * a2x + 2 * a3x + a4x)
            vz = vz + dt / 6 * (a1z + 2 * a2z + 2 * a3z + a4z)
        X[i], Z[i], VX[i], VZ[i] = x, z, vx, vz
    t = np.arange(steps + 1) * dt
    return t, X, Z, VX, VZ


def crossings(t, X, Z, VX, VZ, Ds, kq=0.0):
    """First inward crossing of the opening plane for every trajectory (columns) and every D.

    Returns dict of arrays shape (len(Ds), n): w, vz, vx, tof, ok (crossed before hitting the floor),
    lat (effective lead time for a small inherited lateral velocity: integral of exp(-int kq|v| dt)).
    """
    steps1, n = X.shape
    s, c = math.sin(TILT), math.cos(TILT)
    # plane side for opening at origin x_o: sd = -(x - x_o) c + (z - Zb) s ; crossing when q >= x_o * c
    q = X * c - (Z - Z_BOTTOM) * s
    alive = np.minimum.accumulate(Z >= -1e-9, axis=0)
    q = np.where(alive, q, -1e18)
    cm = np.maximum.accumulate(q, axis=0)
    # lateral decay factor for inherited velocity: dv_lat/dt = -kq |v| v_lat
    if kq > 0:
        sp = np.sqrt(VX * VX + VZ * VZ)
        dt = t[1] - t[0]
        integ = np.concatenate([np.zeros((1, n)), np.cumsum(0.5 * (sp[1:] + sp[:-1]) * dt, axis=0)])
        decay = np.exp(-kq * integ)
        lat = np.concatenate([np.zeros((1, n)), np.cumsum(0.5 * (decay[1:] + decay[:-1]) * dt, axis=0)])
    else:
        lat = np.broadcast_to(t[:, None], X.shape)
    # globally sorted trick: column j occupies [j*steps1, (j+1)*steps1)
    big = 1e7
    flat = (cm.T + (np.arange(n) * big)[:, None]).ravel()
    out = {k: np.full((len(Ds), n), np.nan) for k in ("w", "vz", "vx", "tof", "lat")}
    out["ok"] = np.zeros((len(Ds), n), bool)
    cols = np.arange(n)
    for di, D in enumerate(Ds):
        thr = opening_origin_x(D) * c
        idx = np.searchsorted(flat, thr + cols * big, side="left") - cols * steps1
        ok = (idx >= 1) & (idx < steps1)
        i1 = np.where(ok, idx, 1)
        i0 = i1 - 1
        q0 = q[i0, cols]
        q1 = q[i1, cols]
        f = np.clip((thr - q0) / np.where(q1 - q0 == 0, 1, q1 - q0), 0, 1)
        lin = lambda A: A[i0, cols] + f * (A[i1, cols] - A[i0, cols])
        x = lin(X)
        z = lin(Z)
        xo = opening_origin_x(D)
        w = (x - xo) * s + (z - Z_BOTTOM) * c
        out["w"][di] = np.where(ok, w, np.nan)
        out["vz"][di] = np.where(ok, lin(VZ), np.nan)
        out["vx"][di] = np.where(ok, lin(VX), np.nan)
        out["tof"][di] = np.where(ok, t[i0] + f * (t[i1] - t[i0]), np.nan)
        out["lat"][di] = np.where(ok, lin(np.asarray(lat)), np.nan)
        out["ok"][di] = ok
    return out


# ----------------------------------------------------------------------------- field grids
V_MIN, V_MAX, NV = 150.0, 650.0, 360
TH_MIN, TH_MAX, NTH = 20.0, 88.0, 273      # 0.25 deg
D_GRID = np.arange(30.0, 141.0, 1.0)
LNV = np.linspace(math.log(V_MIN), math.log(V_MAX), NV)
THG = np.linspace(TH_MIN, TH_MAX, NTH)
DLNV = LNV[1] - LNV[0]
DTH = THG[1] - THG[0]

HERE = os.path.dirname(os.path.abspath(__file__))
CACHE = os.path.join(HERE, "cache")


def compute_field(name, chunk=24):
    kq, kl = truth_consts(name)
    nd = len(D_GRID)
    F = {k: np.full((nd, NV, NTH), np.nan, np.float32) for k in ("w", "vz", "vx", "tof", "lat")}
    F["ok"] = np.zeros((nd, NV, NTH), bool)
    V = np.exp(LNV)
    for j0 in range(0, NTH, chunk):
        ths = THG[j0:j0 + chunk]
        vv, tt = np.meshgrid(V, ths, indexing="ij")
        t, X, Z, VX, VZ = integrate(vv.ravel(), np.radians(tt.ravel()), kq, kl)
        cr = crossings(t, X, Z, VX, VZ, D_GRID, kq)
        for k in F:
            F[k][:, :, j0:j0 + len(ths)] = cr[k].reshape(nd, NV, len(ths))
    return F


def load_field(name):
    os.makedirs(CACHE, exist_ok=True)
    path = os.path.join(CACHE, f"field_{name}.npz")
    if os.path.exists(path):
        d = np.load(path)
        return {k: d[k] for k in d.files}
    F = compute_field(name)
    np.savez_compressed(path, **F)
    return F


DESCENT_MIN_DEG = float(os.environ.get("DESCENT_MIN_DEG", "0"))


def descending(vz, vx):
    if DESCENT_MIN_DEG <= 0:
        return vz < 0
    return vz < -math.tan(math.radians(DESCENT_MIN_DEG)) * np.abs(vx)


def score_indicator(F, margin=MARGIN):
    lo, hi = w_band(margin)
    w = F["w"]
    with np.errstate(invalid="ignore"):
        return F["ok"] & descending(F["vz"], F["vx"]) & (w >= lo) & (w <= hi)


def gauss_matrix(n, sigma_cells):
    i = np.arange(n)
    K = np.exp(-0.5 * ((i[:, None] - i[None, :]) / sigma_cells) ** 2)
    K /= (sigma_cells * math.sqrt(2 * math.pi))
    return K


def blur_prob(S, sig_v_rel, sig_th_deg):
    """P(score) when exit speed has relative sigma and angle has sigma (deg): blur indicator per D slice."""
    Kv = gauss_matrix(NV, sig_v_rel / DLNV) if sig_v_rel > 0 else np.eye(NV)
    Kt = gauss_matrix(NTH, sig_th_deg / DTH) if sig_th_deg > 0 else np.eye(NTH)
    S = S.astype(np.float64)
    return np.matmul(np.matmul(Kv, S), Kt.T).astype(np.float32)


def trilinear(A, D, v, th):
    """Interpolate field A[D, lnv, th] at arrays D, v (in/s), th (deg). Out-of-grid -> nan."""
    D = np.asarray(D, float)
    fd = (D - D_GRID[0]) / (D_GRID[1] - D_GRID[0])
    fv = (np.log(np.maximum(v, 1e-9)) - LNV[0]) / DLNV
    ft = (np.asarray(th, float) - THG[0]) / DTH
    out_of = (fd < 0) | (fd > len(D_GRID) - 1) | (fv < 0) | (fv > NV - 1) | (ft < 0) | (ft > NTH - 1)
    fd = np.clip(fd, 0, len(D_GRID) - 1 - 1e-9)
    fv = np.clip(fv, 0, NV - 1 - 1e-9)
    ft = np.clip(ft, 0, NTH - 1 - 1e-9)
    i, j, k = fd.astype(int), fv.astype(int), ft.astype(int)
    a, b, c = fd - i, fv - j, ft - k
    res = 0
    for di, wa in ((0, 1 - a), (1, a)):
        for dj, wb in ((0, 1 - b), (1, b)):
            for dk, wc in ((0, 1 - c), (1, c)):
                res = res + wa * wb * wc * A[i + di, j + dj, k + dk].astype(np.float64)
    return np.where(out_of, np.nan, res)


# ----------------------------------------------------------------------------- shooter map
K_GAIN = 0.09        # in/s per rpm at eta = 1 (assumed: ~96 mm wheel, ball ~45% of surface speed)
ETA_DROP = float(os.environ.get("ETA_DROP", "0.04"))   # exit-speed efficiency falls 4% from flattest to steepest hood (assumed)
TH_LO, TH_HI = 30.0, 80.0
BETA = float(os.environ.get("BETA", "0.15"))           # bow of the hood linkage: theta = lo + span*(h + beta h(1-h))
SPEED_CURVE = float(os.environ.get("SPEED_CURVE", "0"))  # exit speed per rpm falls by this fraction per 3000 rpm above 3000


def hood_to_theta(h, beta=BETA, lo=TH_LO, hi=TH_HI):
    h = np.asarray(h, float)
    return lo + (hi - lo) * (h + beta * h * (1 - h))


def theta_to_hood(th, beta=BETA, lo=TH_LO, hi=TH_HI):
    y = (np.asarray(th, float) - lo) / (hi - lo)
    if beta == 0:
        return y
    # beta h^2 - (1+beta) h + y = 0
    return ((1 + beta) - np.sqrt((1 + beta) ** 2 - 4 * beta * y)) / (2 * beta)


def eta(h, drop=ETA_DROP):
    return 1.0 - drop * np.asarray(h, float)


def rpm_to_v(rpm, h, gain=K_GAIN, drop=ETA_DROP):
    rpm = np.asarray(rpm, float)
    return gain * eta(h, drop) * rpm * (1 - SPEED_CURVE * (rpm - 3000) / 3000)


def v_to_rpm(v, h, gain=K_GAIN, drop=ETA_DROP):
    return np.asarray(v, float) / (gain * eta(h, drop))


# ----------------------------------------------------------------------------- hood/rpm grid helpers
H_GRID = np.linspace(0.0, 1.0, 401)          # hood step 0.0025
R_GRID = np.arange(1500.0, 7000.0, 5.0)      # rpm step 5


def field_on_rh(A, D, R=R_GRID, H=H_GRID, gain=K_GAIN):
    """Sample a field at distance D on the (rpm, hood) grid -> shape (len(R), len(H))."""
    rr, hh = np.meshgrid(R, H, indexing="ij")
    return trilinear(A, np.full(rr.shape, D), rpm_to_v(rr, hh, gain), hood_to_theta(hh))
