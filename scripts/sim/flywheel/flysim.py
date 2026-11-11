"""Vectorised flywheel + FTC control-loop simulator (numpy, 5 kHz plant).

Plant: goBILDA 5203 1:1 (6000 rpm) bare motor, direct drive, ideal battery + Rint.
Specs (https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/):
  12 V nominal, 6000 rpm no-load, 0.25 A no-load, 9.2 A stall, 1.47 kg.cm stall torque.
Inductance ignored (L/R << 1 ms vs mechanical tau >= 0.2 s); PWM modelled as average voltage.
"""
import math
import numpy as np

TWO_PI = 2.0 * math.pi
RPM = 60.0 / TWO_PI            # rad/s -> rpm
CPR = 8192.0                   # REV Through Bore on the flywheel
CPR_MOTOR = 28.0               # goBILDA motor encoder at the output shaft (1:1)

V_NOM, W_FREE_RPM, I_FREE, I_STALL = 12.0, 6000.0, 0.25, 9.2
T_STALL = 1.47 * 0.0980665     # kg.cm -> N.m
RM = V_NOM / I_STALL           # 1.304 ohm
KT = T_STALL / I_STALL         # N.m/A from stall data
KE = (V_NOM - I_FREE * RM) / (W_FREE_RPM / RPM)   # V.s/rad from free speed
# Sensitivity overrides, off by default: FLY_KT=ke sets KT = KE (19% more torque), FLY_QW scales the
# Kalman process noise on w, FLY_DT sets the plant step.
import os as _os
if _os.environ.get('FLY_KT') == 'ke':
    KT = KE
QW = float(_os.environ.get('FLY_QW', '1.0'))

WHEEL_R = 0.048                # 96 mm wheel (72 mm => multiply ball mass by 0.5625)
EXIT_FRAC = 0.45               # ball centre exit speed / wheel surface speed
SHELL = 5.0 / 6.0              # thin-shell ball rolling on the hood: F_wheel = (5/6) m dv/dt
DT = float(_os.environ.get('FLY_DT', '2e-4'))                      # 5 kHz plant step

FRICTION_LEVELS = {            # total friction torque at 3000 rpm (N.m), split 30% Coulomb / 50% viscous / 20% quadratic
    'low': 0.004, 'mid': 0.008, 'high': 0.016}


def friction_coeffs(T3000):
    w3 = 3000.0 / RPM
    T3000 = np.asarray(T3000, float)
    return 0.3 * T3000, 0.5 * T3000 / w3, 0.2 * T3000 / (w3 * w3)


def v_ss(w, Tcf, bf, cf):
    """Motor terminal volts needed to hold w (rad/s) in steady state (true plant)."""
    return KE * w + RM * (Tcf * np.tanh(w / 1.0) + bf * w + cf * w * np.abs(w)) / KT


def ff_table_volts(target_w, Tcf, bf, cf):
    """Interpolated table measured every 500 rpm (what a team's measured table would be)."""
    grid_rpm = np.arange(0, 6001, 500.0)
    t_rpm = target_w * RPM
    out = np.empty_like(np.asarray(t_rpm, float))
    for i, (tr, a, b, c) in enumerate(np.broadcast(t_rpm, Tcf, bf, cf)):
        tab = v_ss(grid_rpm / RPM, a, b, c)
        out.flat[i] = np.interp(tr, grid_rpm, tab)
    return out


def kskv_fit(Tcf, bf, cf):
    """Least-squares kS + kV*w fit of the true curve over 1000-5000 rpm."""
    g = np.arange(1000, 5001, 250.0) / RPM
    ks, kv = [], []
    for a, b, c in np.broadcast(Tcf, bf, cf):
        A = np.vstack([np.ones_like(g), g]).T
        sol = np.linalg.lstsq(A, v_ss(g, a, b, c), rcond=None)[0]
        ks.append(sol[0]); kv.append(sol[1])
    return np.array(ks), np.array(kv)


def ball_k(m):
    """Wheel-referred inertia a ball adds by the end of contact (angular momentum taken = k*w)."""
    return SHELL * m * EXIT_FRAC * WHEEL_R ** 2


def free_speed_rpm(Voc, Rint, Tcf, bf, cf, Iot=0.0, p=1.0):
    """Steady speed at constant power p (bisection on net torque)."""
    def net(w):
        Vb = (Voc - Rint * Iot + Rint * p * KE * w / RM) / (1 + Rint * p * p / RM)
        Im = (p * Vb - KE * w) / RM
        return KT * Im - (Tcf * math.tanh(w) + bf * w + cf * w * w)
    lo, hi = 0.0, 2000.0
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if net(mid) > 0:
            lo = mid
        else:
            hi = mid
    return lo * RPM


# ------------------------------------------------------------------------------------------
# Kalman gain for the [theta, omega, d] observer (computed offline per unique parameter set)
# ------------------------------------------------------------------------------------------
_kf_cache = {}


def kf_gain(kA, kV, T, qd, sigma_theta):
    key = (round(kA, 12), round(kV, 12), round(T, 6), qd, round(sigma_theta, 9), QW)
    if key in _kf_cache:
        return _kf_cache[key]
    from scipy.linalg import expm, solve_discrete_are
    lam = kV / kA
    Ac = np.array([[0, 1, 0], [0, -lam, 1], [0, 0, 0]], float)
    Ad = expm(Ac * T)
    Q = np.diag([1e-12, QW, qd]) * T
    C = np.array([[1.0, 0, 0]])
    R = np.array([[sigma_theta ** 2]])
    Pp = solve_discrete_are(Ad.T, C.T, Q, R)
    L = (Pp @ C.T) / (C @ Pp @ C.T + R)
    _kf_cache[key] = L[:, 0]
    return L[:, 0]


# ------------------------------------------------------------------------------------------
# Simulation
# ------------------------------------------------------------------------------------------
def simulate(P, ctrl, C, scen):
    """P: per-sim arrays (N,). ctrl: controller name. C: per-sim controller params. scen: scenario dict.

    Required P keys: J, m, Tc, Voc, Rint, fric (T at 3000 rpm), T (loop s), W (window s),
                     tsA (timestamp error half-range s), ffk (FF scale), kAk (model kA scale), seed (int)
    scen keys: T_end, w0_rpm, target_rpm, shots (nb,) nominal touch times (abs s), shot_jit (s),
               sag (I, t0, t1), pwm ('avg'|'coast'), ss_win (t0,t1), cur_win (t0,t1)
    """
    N = max(np.size(v) for v in P.values())
    f = lambda k: np.asarray(P[k], float) * np.ones(N)
    J, mball, Tcon, Voc, Rint = f('J'), f('m'), f('Tc'), f('Voc'), f('Rint')
    Tcf, bf, cf = friction_coeffs(f('fric'))
    Tloop, Wwin, tsA, ffk, kAk = f('T'), f('W'), f('tsA'), f('ffk'), f('kAk')
    seed = np.asarray(P['seed'], int) * np.ones(N, int)
    T_end = scen['T_end']
    nsteps = int(round(T_end / DT))
    tgt = scen['target_rpm'] / RPM
    w0 = scen['w0_rpm'] / RPM
    pwm = scen.get('pwm', 'avg')
    vs_tau = scen.get('vs_tau', 0.1)
    vs_hold_steps = int(round(scen.get('vs_hold', 0.0) / DT))
    shots = np.asarray(scen.get('shots', []), float)
    nb = len(shots)
    sag_I, sag_t0, sag_t1 = scen.get('sag', (0.0, 0.0, 0.0))
    ss0, ss1 = scen.get('ss_win', (T_end - 0.5, T_end))
    cw0, cw1 = scen.get('cur_win', (0.0, T_end))

    is_hub = ctrl == 'hub'
    is_oracle = ctrl == 'oracle'
    if is_hub:
        Tloop = np.full(N, 0.010)
    # ---- per-seed random streams (common random numbers across controllers / gains) ----
    useeds = np.unique(seed)
    Kmax = int(T_end / 0.006) + 4
    sj = {}
    for s in useeds:
        r = np.random.default_rng(int(s))
        sj[s] = dict(jit=r.uniform(-1, 1, Kmax), tse=r.uniform(-1, 1, Kmax), lat=r.uniform(0, 1, Kmax),
                     ph=r.uniform(0, 1), bj=r.uniform(-1, 1, max(nb, 1)), th0=r.uniform(0, TWO_PI))
    jit = np.stack([sj[s]['jit'] for s in seed])
    tse = np.stack([sj[s]['tse'] for s in seed])
    latu = np.stack([sj[s]['lat'] for s in seed])
    ph = np.array([sj[s]['ph'] for s in seed])
    bj = np.stack([sj[s]['bj'] for s in seed])
    th = np.array([sj[s]['th0'] for s in seed])
    jit_amp = 0.0 if is_hub else 0.002
    fire_t = ph[:, None] * Tloop[:, None] + np.concatenate(
        [np.zeros((N, 1)), np.cumsum(Tloop[:, None] + jit_amp * jit[:, :-1], axis=1)], axis=1)
    fire = np.rint(fire_t / DT).astype(np.int64)
    lat_steps = np.rint((0.002 + 0.002 * latu) / DT).astype(np.int64)
    if is_hub:
        lat_steps[:] = 1
    kidx = np.zeros(N, np.int64)
    next_fire = fire[:, 0].copy()
    ar = np.arange(N)

    # ---- shots ----
    nc = np.rint(Tcon / DT).astype(np.int64)
    if nb:
        tb_nom = np.broadcast_to(shots, (N, nb)).copy()
        tb = tb_nom + scen.get('shot_jit', 0.005) * bj[:, :nb]
        tb_step = np.rint(tb / DT).astype(np.int64)
    else:
        tb_nom = np.full((N, 1), 1e9)
        tb_step = np.full((N, 1), 1 << 60, np.int64)
    nbb = tb_step.shape[1]
    kb = ball_k(mball)
    bi = np.zeros(N, np.int64)
    cur_start = tb_step[:, 0].copy()
    w_touch = np.full((N, nbb), np.nan)
    wm_touch = np.full((N, nbb), np.nan)
    w_exit = np.full((N, nbb), np.nan)
    last_start = tb_step[:, -1].copy() if nb else np.zeros(N, np.int64)
    dip_end = tb_step[:, 1].copy() if nb > 1 else (tb_step[:, 0] + int(round(0.6 / DT)))

    # ---- feed-forward ----
    ffV = ffk * ff_table_volts(np.full(N, tgt), Tcf, bf, cf)
    ks_, kv_ = kskv_fit(Tcf, bf, cf)
    ks_ *= ffk; kv_ *= ffk
    kA = kAk * RM * J / KT                      # model V per rad/s^2

    # ---- controller state ----
    Kh = fire.shape[1]
    hist_c = np.zeros((N, Kh))
    hist_tau = np.zeros((N, Kh))
    hist_V = np.zeros((N, Kh))                  # estimated applied motor volts of command k
    I = np.zeros(N)
    wm_last = np.full(N, w0)
    est_last = np.full(N, w0)
    est_touch = np.full((N, max(nb, 1)), np.nan)
    wc_tol = scen.get('wc_tol', 0.0)
    wc_val = np.full(N, np.nan)
    wc_t = np.full(N, -1.0)
    sig_override = scen.get('kf_sig_ts', None)
    tau_prev = np.zeros(N)
    Vprev = np.full(N, 0.0)
    Vprev2 = np.full(N, 0.0)
    g = lambda k, d=0.0: np.asarray(C.get(k, d), float) * np.ones(N)
    kP, kI, kD = g('kP'), g('kI'), g('kD')
    thr, alpha, iz = g('thr', 1e9), g('alpha', 0.9), g('iz', 1e9)
    Bst, lead, dur = g('B'), g('lead'), g('dur')
    if ctrl == 'kf':
        qd = g('qd', 1e6)
        dr = g('dr', 1.0)
        sig = (w0 if w0 > 0 else tgt) * (0.0005 if sig_override is None else sig_override) / math.sqrt(3.0) + TWO_PI / CPR / math.sqrt(12)
        Lg = np.zeros((N, 3))
        for n in range(N):
            Lg[n] = kf_gain(kA[n], kv_[n], Tloop[n], float(qd[n]), sig)
        xth = th.copy(); xw = np.full(N, w0); xd = np.zeros(N)
    if is_hub:
        maxV_tps = np.array([free_speed_rpm(12.5, r_, a, b, c) for r_, a, b, c in zip(Rint, Tcf, bf, cf)]) / 60 * CPR_MOTOR
        Fh = 32767.0 / maxV_tps * g('Fk', 1.0) * ffk
        tgt_tps = tgt * RPM / 60 * CPR_MOTOR
        hub_c = np.zeros((N, 5)); hub_n = np.zeros(N, np.int64)
        hub_I = np.zeros(N); hub_eprev = np.zeros(N)

    # ---- plant state ----
    w = np.full(N, w0)
    th = th + 0.0
    Vs = Voc.copy()
    p_app = np.zeros(N)
    pend_p = np.zeros(N)
    pend_step = np.full(N, -1, np.int64)
    if w0 > 0:   # start in (approximately) steady state
        p_app = np.clip(v_ss(w, Tcf, bf, cf) / Voc, 0, 1)
        Vprev[:] = Vprev2[:] = p_app * Voc

    # ---- metric accumulators ----
    bands = (25.0, 50.0, 100.0)
    last_out = {b: np.full(N, -1.0) for b in bands}
    last_out_spin = np.full(N, -1.0)
    first_in50 = np.full(N, np.nan)
    wmin = np.full(N, np.inf)
    wmax = np.full(N, -np.inf)
    wmax_after = np.full(N, -np.inf)
    ss_s = np.zeros(N); ss_q = np.zeros(N); ss_n = 0
    ssm_s = np.zeros(N); ssm_q = np.zeros(N); ssm_n = np.zeros(N)
    cur_q = np.zeros(N); cur_n = 0; cur_max = np.zeros(N)
    ss0s, ss1s, cw0s, cw1s = int(ss0 / DT), int(ss1 / DT), int(cw0 / DT), int(cw1 / DT)
    tgt_rpm = scen['target_rpm']
    pi_c = math.pi

    for s in range(nsteps):
        t = s * DT
        # ---------------- apply delayed commands ----------------
        m = pend_step == s
        if m.any():
            p_app[m] = pend_p[m]
            pend_step[m] = -1
        # ---------------- ball touches ----------------
        mt = cur_start == s
        if mt.any():
            ii = np.nonzero(mt)[0]
            w_touch[ii, bi[ii]] = w[ii] * RPM
            wm_touch[ii, bi[ii]] = wm_last[ii] * RPM
            est_touch[ii, bi[ii]] = est_last[ii] * RPM
        # ---------------- controller ----------------
        if is_oracle:
            p = np.where(w < tgt, 1.0, np.clip(v_ss(np.full(N, tgt), Tcf, bf, cf) / Vb_prev, 0, 1)) if s > 0 else p_app
            p_app = np.trunc(p * 32767.0) / 32767.0
            wm_last = w.copy()
        else:
            mf = next_fire == s
            if mf.any():
                idx = np.nonzero(mf)[0]
                k = kidx[idx]
                if is_hub:
                    c28 = np.floor(th[idx] * CPR_MOTOR / TWO_PI)
                    hc = hub_c[idx]
                    hc[:, :-1] = hc[:, 1:]
                    hc[:, -1] = c28
                    hub_c[idx] = hc
                    hub_n[idx] += 1
                    nn = np.minimum(hub_n[idx], 5)
                    old = hc[np.arange(len(idx)), 5 - nn]
                    span = np.maximum(nn - 1, 1) * 0.010
                    vt = np.where(nn > 1, (c28 - old) / span, w0 * RPM / 60 * CPR_MOTOR)
                    e = tgt_tps - vt
                    hub_I[idx] = np.clip(hub_I[idx] + kI[idx] * e * 0.010, -32767, 32767)
                    out = Fh[idx] * tgt_tps + kP[idx] * e + hub_I[idx] + kD[idx] * (e - hub_eprev[idx]) / 0.010
                    hub_eprev[idx] = e
                    p = np.clip(out / 32767.0, -1, 1)
                    wm_last[idx] = vt / CPR_MOTOR * 60 / RPM
                else:
                    tau = t + tsA[idx] * tse[idx, k]
                    c = np.floor(th[idx] * CPR / TWO_PI)
                    hist_c[idx, k] = c
                    hist_tau[idx, k] = tau
                    offs = np.arange(1, 17)
                    jj = k[:, None] - offs[None, :]
                    jjc = np.maximum(jj, 0)
                    age = tau[:, None] - hist_tau[idx[:, None], jjc]
                    ok = (jj >= 0) & (age <= Wwin[idx][:, None] + 1e-9)
                    off = np.maximum(ok.sum(1), 1)
                    j = np.maximum(k - off, 0)
                    den = tau - hist_tau[idx, j]
                    wm = np.where(k >= 1, (c - hist_c[idx, j]) / np.where(den > 0, den, 1.0) * TWO_PI / CPR, w0)
                    dtk = np.where(k >= 1, tau - tau_prev[idx], Tloop[idx])
                    vs = Vs[idx]
                    est = wm
                    e = tgt - wm
                    if ctrl in ('ffpi', 'ffpid', 'ffpiz'):
                        # mirrors PidController: integral sum holds while the PID output is pinned at
                        # [-ff, headroom-ff] in the direction of the error; D on measurement
                        ff = ffV[idx]
                        Dt = np.where(k >= 1, -kD[idx] * (wm - wm_last[idx]) / dtk, 0.0) if ctrl == 'ffpid' else 0.0
                        out0 = kP[idx] * e + kI[idx] * I[idx] + Dt
                        pinned = ((out0 >= vs - ff) & (e > 0)) | ((out0 <= -ff) & (e < 0)) | (kI[idx] == 0) | (np.abs(e) > iz[idx])
                        Ii = np.where(pinned, np.where(kI[idx] == 0, 0.0, I[idx]), I[idx] + e * dtk)
                        I[idx] = Ii
                        V = ff + kP[idx] * e + kI[idx] * Ii + Dt
                        p = V / vs
                    elif ctrl == 'ffp':
                        V = ks_[idx] + kv_[idx] * tgt + kP[idx] * e
                        p = V / vs
                    elif ctrl == 'bb':
                        p = np.where(wm < tgt, 1.0, 0.0)
                    elif ctrl == 'bbff':
                        p = np.where(wm < tgt, 1.0, alpha[idx] * ffV[idx] / vs)
                    elif ctrl == 'hybrid':
                        ff = ffV[idx]
                        In = I[idx] + kI[idx] * e * dtk
                        Vn = ff + kP[idx] * e + In
                        sat = (Vn > vs) | (Vn < 0) | (e > thr[idx])
                        Ii = np.where(sat, I[idx], In)
                        I[idx] = Ii
                        V = np.where(e > thr[idx], vs, ff + kP[idx] * e + Ii)
                        p = V / vs
                    elif ctrl == 'boost':
                        tn = tb_nom[idx]
                        act = ((tau[:, None] >= tn + lead[idx][:, None]) &
                               (tau[:, None] < tn + lead[idx][:, None] + dur[idx][:, None])).any(1)
                        V = ffV[idx] + kP[idx] * e + np.where(act, Bst[idx], 0.0)
                        p = V / vs
                    elif ctrl == 'pred':
                        # predict w at the instant this command lands, from window centre, with the motor model
                        tmid = 0.5 * (tau + hist_tau[idx, j])
                        tl = tau + 0.003
                        M = 8
                        ts_ = tmid[:, None] + (tl - tmid)[:, None] * (np.arange(M) + 0.5)[None, :] / M
                        kk = np.maximum(k[:, None] - np.arange(1, 13)[None, :], 0)
                        tapp = hist_tau[idx[:, None], kk] + 0.003
                        Vh = hist_V[idx[:, None], kk]
                        act = ts_[:, :, None] >= tapp[:, None, :]          # (n,M,12) newest first
                        first = np.argmax(act, axis=2)
                        Vt = np.take_along_axis(np.broadcast_to(Vh[:, None, :], act.shape), first[:, :, None], 2)[:, :, 0]
                        Vt = np.where(act.any(2), Vt, Vh[:, -1:])
                        acc = (Vt.mean(1) - (ks_[idx] + kv_[idx] * wm)) / kA[idx]
                        wp = np.where(k >= 1, wm + acc * (tl - tmid), wm)
                        est = wp
                        V = ffV[idx] + kP[idx] * (tgt - wp)
                        p = V / vs
                    elif ctrl == 'kf':
                        # predict from tau_prev to tau with the two piecewise-constant commands, then correct
                        lam = kv_[idx] / kA[idx]
                        a = 1.0 / kA[idx]
                        x_th, x_w, x_d = xth[idx], xw[idx], xd[idx]
                        seg1 = np.minimum(0.003, dtk)
                        for Vseg, hseg in ((Vprev2[idx], seg1), (Vprev[idx], dtk - seg1)):
                            winf = (a * (Vseg - ks_[idx]) + x_d) / lam
                            ex = np.exp(-lam * hseg)
                            x_th = x_th + winf * hseg + (x_w - winf) * (1 - ex) / lam
                            x_w = winf + (x_w - winf) * ex
                        y = c * TWO_PI / CPR
                        inn = np.where(k >= 1, y - x_th, 0.0)
                        x_th = np.where(k >= 1, x_th + Lg[idx, 0] * inn, y)
                        x_w = x_w + Lg[idx, 1] * inn
                        x_d = x_d + Lg[idx, 2] * inn
                        xth[idx], xw[idx], xd[idx] = x_th, x_w, x_d
                        est = x_w
                        wp = x_w + (a * (Vprev[idx] - ks_[idx] - kv_[idx] * x_w) + x_d) * 0.003
                        V = ks_[idx] + kv_[idx] * tgt + kP[idx] * (tgt - wp) - dr[idx] * kA[idx] * x_d
                        p = V / vs
                    else:
                        raise ValueError(ctrl)
                    wm_last[idx] = wm
                    est_last[idx] = est
                    tau_prev[idx] = tau
                if is_hub:
                    est_last[idx] = wm_last[idx]
                if not is_hub:
                    p = np.clip(p, 0.0, 1.0)
                p = np.trunc(p * 32767.0) / 32767.0
                if wc_tol > 0 and not is_hub:
                    cv = wc_val[idx]; ct = wc_t[idx]
                    with np.errstate(invalid='ignore'):
                        sw = ((np.abs(p - cv) > wc_tol) | ((p == 0) & (cv != 0)) | ((p != 0) & (cv == 0)) |
                              ((p >= 1) & ~(cv >= 1)) | np.isnan(cv) | (t - ct > 0.25))
                    p = np.where(sw, p, cv)
                    wc_val[idx] = p
                    wc_t[idx] = np.where(sw, t, ct)
                Vap = p * Vs[idx]
                if not is_hub:
                    hist_V[idx, k] = Vap
                Vprev2[idx] = Vprev[idx]
                Vprev[idx] = Vap
                pend_p[idx] = p
                pend_step[idx] = s + lat_steps[idx, k]
                kidx[idx] = k + 1
                next_fire[idx] = fire[idx, np.minimum(k + 1, Kh - 1)]
                if ss0s <= s < ss1s:
                    wv = wm_last[idx] * RPM
                    ssm_s[idx] += wv; ssm_q[idx] += wv * wv; ssm_n[idx] += 1
        # ---------------- plant ----------------
        Iot = sag_I if sag_t0 <= t < sag_t1 else 0.0
        pa = p_app
        Vb = (Voc - Rint * Iot + Rint * pa * KE * w / RM) / (1.0 + Rint * pa * pa / RM)
        Im = (pa * Vb - KE * w) / RM
        if pwm == 'coast':
            off_ = (pa == 0.0) | (Im * np.sign(pa) < 0)
        else:
            off_ = pa == 0.0
        if off_.any():
            Im = np.where(off_, 0.0, Im)
            Vb = np.where(off_, Voc - Rint * Iot, Vb)
        Vb_prev = Vb
        Tf = Tcf * np.tanh(w) + bf * w + cf * w * np.abs(w)
        phs = (s - cur_start) / nc
        act = (phs >= 0) & (phs < 1)
        if act.any():
            sc = np.where(act, 0.5 * (1 - np.cos(pi_c * phs)), 0.0)
            sd = np.where(act, pi_c / (2 * Tcon) * np.sin(pi_c * phs), 0.0)
            wd = (KT * Im - Tf - kb * w * sd) / (J + kb * sc)
        else:
            wd = (KT * Im - Tf) / J
        w = w + wd * DT
        th = th + w * DT
        if vs_hold_steps > 0:
            if s % vs_hold_steps == 0:
                Vs = Vb.copy()
        elif vs_tau > 0:
            Vs = Vs + (Vb - Vs) * (DT / vs_tau)
        else:
            Vs = Vb.copy()
        # ball end
        me = (s - cur_start) == nc - 1
        if me.any():
            ii = np.nonzero(me)[0]
            w_exit[ii, bi[ii]] = w[ii] * RPM
            bi[ii] += 1
            cur_start[ii] = np.where(bi[ii] < nbb, tb_step[ii, np.minimum(bi[ii], nbb - 1)], 1 << 60)
        # ---------------- metrics ----------------
        wr = w * RPM
        err = np.abs(wr - tgt_rpm)
        o = err > 50.0
        last_out_spin[o] = t
        fi = (~o) & np.isnan(first_in50)
        if fi.any():
            first_in50[fi] = t
        after = s >= last_start
        for b in bands:
            mm = after & (err > b)
            last_out[b][mm] = t
        indip = (s >= tb_step[:, 0]) & (s < dip_end)
        wmin = np.where(indip, np.minimum(wmin, wr), wmin)
        wmax = np.maximum(wmax, wr)
        wmax_after = np.where(after, np.maximum(wmax_after, wr), wmax_after)
        if ss0s <= s < ss1s:
            ss_s += wr; ss_q += wr * wr; ss_n += 1
        if cw0s <= s < cw1s:
            cur_q += Im * Im; cur_n += 1
            cur_max = np.maximum(cur_max, np.abs(Im))

    res = {}
    res['t_settle50'] = np.where(last_out_spin > T_end - 0.3, np.nan, last_out_spin + DT)
    res['overshoot'] = wmax - tgt_rpm
    res['t_first50'] = first_in50
    mean_ss = ss_s / max(ss_n, 1)
    res['ss_bias'] = mean_ss - tgt_rpm
    res['ss_std'] = np.sqrt(np.maximum(ss_q / max(ss_n, 1) - mean_ss ** 2, 0))
    mm_ = ssm_s / np.maximum(ssm_n, 1)
    res['ssm_std'] = np.sqrt(np.maximum(ssm_q / np.maximum(ssm_n, 1) - mm_ ** 2, 0))
    res['I_rms'] = np.sqrt(cur_q / max(cur_n, 1))
    res['I_max'] = cur_max
    if nb:
        t_last = tb_step[:, -1] * DT
        for b in bands:
            lo = last_out[b]
            rec = np.where(lo < 0, 0.0, lo + DT - t_last)
            res[f'rec{int(b)}'] = np.where(lo > T_end - 0.15, np.nan, rec)
        res['dip'] = wmin - tgt_rpm
        res['e_touch'] = w_touch - tgt_rpm
        res['meas_err_touch'] = wm_touch - w_touch
        res['est_err_touch'] = est_touch - w_touch
        res['w_touch'] = w_touch
        res['w_exit'] = w_exit
        res['overshoot_after'] = wmax_after - tgt_rpm
    return res
