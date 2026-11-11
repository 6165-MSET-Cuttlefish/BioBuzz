"""Flywheel + control-loop simulator for the robot's two shooters (vectorised numpy, 5 kHz plant).

Derived from ../../flywheel/flysim.py. Kept: battery Voc+Rint algebraic model,
average-voltage PWM with FLOAT at p=0, 1/32767 power quantisation, 30/50/20 Coulomb/viscous/quadratic friction,
8192 CPR position windows with timestamp error and the 'oldest sample with age <= W' rule (Shooter.measure()),
and the ball model: thin-shell ball rolling on a fixed hood, F_wheel = (5/6) m dv/dt, exit speed = f x surface
speed, wheel loses k*w with k = (5/6) m f r^2 over a raised-cosine ramp of length Tc.

Changed / added:
- every plant and ball parameter is per simulation (target, wheel radius, exit fraction, motor count, gear ratio,
  kt), so one batch can hold mixed configurations;
- motor: goBILDA 5000-0002-4008 page (12 V, 5800 rpm tested / 6000 theoretical, 0.25 A, 9.2 A, 1.47 kg.cm);
  ke from the TESTED free speed by default (env FLY_FREE=6000 to use the theoretical one);
- controllers: 'oracle' (true w at 5 kHz, full power below target else exact FF), 'ff' (exact FF only),
  'team' (Shooter.java + PidController.java: table FF + PI, integral held while pinned, no I-zone, D=0,
  voltage sampled every 50 loops, applied volts capped at min(13 V, battery), command applied at write() time,
  i.e. after follower.update), 'rec' (full power when > thr below target, else FF + P + I only within iz rpm,
  voltage every loop, full duty allowed, command sent right after compute);
- measurement 'hub' (position windows, as before) or 'oq' (OctoQuad: velocity latched every Nq on the board
  clock as counts per interval, read value = newest latch at least 6.6 ms old, no timestamp error);
- records per-ball touch/exit speed, energy drawn by each ball, recovery to 1/2/3 %.
"""
import math
import os
import numpy as np

TWO_PI = 2.0 * math.pi
RPM = 60.0 / TWO_PI
CPR = 8192.0
DT = float(os.environ.get('FLY_DT', '2e-4'))

V_NOM, I_FREE, I_STALL = 12.0, 0.25, 9.2
W_FREE_RPM = float(os.environ.get('FLY_FREE', '5800'))
T_STALL = 1.47 * 0.0980665
RM = V_NOM / I_STALL
KT_STALL = T_STALL / I_STALL
KE = (V_NOM - I_FREE * RM) / (W_FREE_RPM / RPM)
SHELL = 5.0 / 6.0
OQ_READ = 0.0066


def friction_coeffs(T3000):
    w3 = 3000.0 / RPM
    T3000 = np.asarray(T3000, float)
    return 0.3 * T3000, 0.5 * T3000 / w3, 0.2 * T3000 / (w3 * w3)


def tfric(w, Tcf, bf, cf):
    return Tcf * np.tanh(w) + bf * w + cf * w * np.abs(w)


def plant_consts(nmot, g, ktm):
    """Wheel-referred: torque = ktw * Im (Im = current of each motor), back-EMF = kew * w_wheel."""
    nmot = np.asarray(nmot, float); g = np.asarray(g, float); ktm = np.asarray(ktm, float)
    return nmot * ktm / g, KE / g


def v_ss(w, Tcf, bf, cf, ktw, kew):
    return kew * w + RM * tfric(w, Tcf, bf, cf) / ktw


def free_speed(Voc, Rint, fric, nmot=1, g=1.0, ktm=KT_STALL, p=1.0):
    Tcf, bf, cf = friction_coeffs(fric)
    ktw, kew = plant_consts(nmot, g, ktm)

    def net(w):
        Vb = (Voc + Rint * nmot * p * kew * w / RM) / (1 + Rint * nmot * p * p / RM)
        Im = (p * Vb - kew * w) / RM
        return ktw * Im - tfric(w, Tcf, bf, cf)
    lo, hi = 0.0, 3000.0
    for _ in range(100):
        mid = 0.5 * (lo + hi)
        if net(mid) > 0:
            lo = mid
        else:
            hi = mid
    return lo


def avail_power(w, Voc, Rint, fric, nmot=1, g=1.0, ktm=KT_STALL, vcap=None):
    """Net mechanical power above friction at wheel speed w at full duty (or duty capped to vcap volts)."""
    Tcf, bf, cf = friction_coeffs(fric)
    ktw, kew = plant_consts(nmot, g, ktm)
    p = 1.0
    if vcap is not None:
        Vb1 = (Voc + Rint * nmot * kew * w / RM) / (1 + Rint * nmot / RM)
        p = min(1.0, vcap / Vb1)
    Vb = (Voc + Rint * nmot * p * kew * w / RM) / (1 + Rint * nmot * p * p / RM)
    Im = (p * Vb - kew * w) / RM
    return (ktw * Im - tfric(w, Tcf, bf, cf)) * w


def simulate(P, ctrl, meas, nb, warm, T_end, shot_jit=0.005):
    """P: dict of per-sim arrays. Required keys:
    J, m, Tc, Voc, Rint, fric, T, W, tsA, seed, tgt (rad/s), r, f, nmot, g, ktm, Nq, lat_lo, lat_hi,
    kP (V per rad/s), kI (V per rad), iz (rad/s), thr (fraction), spacing (s).
    """
    N = max(np.size(v) for v in P.values())
    f_ = lambda k: np.asarray(P[k], float) * np.ones(N)
    J, mball, Tcon, Voc, Rint = f_('J'), f_('m'), f_('Tc'), f_('Voc'), f_('Rint')
    Tcf, bf, cf = friction_coeffs(f_('fric'))
    Tloop, Wwin, tsA = f_('T'), f_('W'), f_('tsA')
    tgt, r, fx = f_('tgt'), f_('r'), f_('f')
    ktw, kew = plant_consts(f_('nmot'), f_('g'), f_('ktm'))
    nmot = f_('nmot')
    Nq, lat_lo, lat_hi = f_('Nq'), f_('lat_lo'), f_('lat_hi')
    kP, kI, iz, thr = f_('kP'), f_('kI'), f_('iz'), f_('thr')
    spacing = f_('spacing')
    seed = np.asarray(P['seed'], int) * np.ones(N, int)
    nsteps = int(round(T_end / DT))
    w0 = tgt.copy()
    ar = np.arange(N)

    Kmax = int(T_end / 0.006) + 4
    sj = {}
    for s in np.unique(seed):
        rg = np.random.default_rng(int(s))
        sj[s] = dict(jit=rg.uniform(-1, 1, Kmax), tse=rg.uniform(-1, 1, Kmax), lat=rg.uniform(0, 1, Kmax),
                     ph=rg.uniform(0, 1), bj=rg.uniform(-1, 1, max(nb, 1)), th0=rg.uniform(0, TWO_PI),
                     qph=rg.uniform(0, 1), hv=rg.uniform(0, 1, (2, Kmax)))
    jit = np.stack([sj[s]['jit'] for s in seed])
    tse = np.stack([sj[s]['tse'] for s in seed])
    latu = np.stack([sj[s]['lat'] for s in seed])
    ph = np.array([sj[s]['ph'] for s in seed])
    bj = np.stack([sj[s]['bj'] for s in seed])
    th = np.array([sj[s]['th0'] for s in seed])
    qph = np.array([sj[s]['qph'] for s in seed])

    if os.environ.get('FLY_HEAVY'):
        # 95 % uniform 10-21 ms, 5 % uniform 25-45 ms: a tail like the loop times in docs/loop-time-findings.md
        hv = np.stack([sj[s]['hv'] for s in seed]); u1 = hv[:, 0, :-1]; u2 = hv[:, 1, :-1]
        per = np.where(u1 < 0.95, 0.010 + 0.011 * u2, 0.025 + 0.020 * u2)
        fire_t = ph[:, None] * Tloop[:, None] + np.concatenate([np.zeros((N, 1)), np.cumsum(per, axis=1)], axis=1)
    else:
        fire_t = ph[:, None] * Tloop[:, None] + np.concatenate(
            [np.zeros((N, 1)), np.cumsum(Tloop[:, None] + 0.002 * jit[:, :-1], axis=1)], axis=1)
    fire = np.rint(fire_t / DT).astype(np.int64)
    lat_steps = np.rint((lat_lo[:, None] + (lat_hi - lat_lo)[:, None] * latu) / DT).astype(np.int64)
    lat_steps = np.maximum(lat_steps, 1)
    kidx = np.zeros(N, np.int64)
    next_fire = fire[:, 0].copy()
    Kh = fire.shape[1]

    # shots
    nc = np.maximum(np.rint(Tcon / DT).astype(np.int64), 1)
    tb = warm + np.arange(nb)[None, :] * spacing[:, None] + shot_jit * bj[:, :nb]
    tb_step = np.rint(tb / DT).astype(np.int64)
    kb = SHELL * mball * fx * r * r
    bi = np.zeros(N, np.int64)
    cur_start = tb_step[:, 0].copy()
    w_touch = np.full((N, nb), np.nan)
    wm_touch = np.full((N, nb), np.nan)
    w_exit = np.full((N, nb), np.nan)
    e_ball = np.zeros((N, nb))
    Lb_prev = np.zeros(N)
    first_step = tb_step[:, 0]
    last_step = tb_step[:, -1]

    ffV = v_ss(tgt, Tcf, bf, cf, ktw, kew)
    vcap = 13.0 if ctrl == 'team' else 1e9

    hist_c = np.zeros((N, Kh))
    hist_tau = np.zeros((N, Kh))
    I = np.zeros(N)
    wm_last = w0.copy()
    tau_prev = np.zeros(N)

    # OctoQuad latch ring buffer (oldest first)
    nq_steps = np.maximum(np.rint(Nq / DT).astype(np.int64), 1)
    next_latch = np.rint(qph * nq_steps).astype(np.int64)
    q_last_c = np.floor((th - w0 * (1.0 - qph) * Nq) * CPR / TWO_PI)
    q_t = np.full((N, 4), -1.0)
    q_v = np.tile(w0[:, None], (1, 4))

    w = w0.copy()
    p_app = np.clip(v_ss(w, Tcf, bf, cf, ktw, kew) / Voc, 0, 1)
    pend_p = np.zeros(N)
    pend_step = np.full(N, -1, np.int64)
    Vb_prev = Voc.copy()
    Vs = Voc.copy()

    bands = (0.01, 0.02, 0.03)
    last_out = {b: np.full(N, -1.0) for b in bands}
    last_out_first = {b: np.full(N, -1.0) for b in bands}
    wmin1 = np.full(N, np.inf)
    st0, st1 = int((warm - 0.3) / DT), int(warm / DT) - 1
    ss_s = np.zeros(N); ss_q = np.zeros(N); ss_n = 0
    sm_s = np.zeros(N); sm_q = np.zeros(N); sm_n = np.zeros(N)
    t_first = tb_step[:, 0] * DT
    second_step = tb_step[:, 1] if nb > 1 else np.full(N, 1 << 60)
    pi_c = math.pi

    trace = [] if os.environ.get('FLY_TRACE') else None
    for s in range(nsteps):
        t = s * DT
        if trace is not None and s % 5 == 0:
            trace.append((t, w[0] * RPM, wm_last[0] * RPM, p_app[0], I[0]))
        mp = pend_step == s
        if mp.any():
            p_app[mp] = pend_p[mp]
            pend_step[mp] = -1
        mt = cur_start == s
        if mt.any():
            ii = np.nonzero(mt)[0]
            w_touch[ii, bi[ii]] = w[ii]
            wm_touch[ii, bi[ii]] = wm_last[ii]
        if meas == 'oq':
            ml = next_latch == s
            if ml.any():
                ii = np.nonzero(ml)[0]
                c = np.floor(th[ii] * CPR / TWO_PI)
                vq = (c - q_last_c[ii]) / Nq[ii] * TWO_PI / CPR
                q_last_c[ii] = c
                q_t[ii, :-1] = q_t[ii, 1:]; q_t[ii, -1] = t
                q_v[ii, :-1] = q_v[ii, 1:]; q_v[ii, -1] = vq
                next_latch[ii] += nq_steps[ii]
        # controller
        if ctrl == 'oracle':
            p = np.where(w < tgt, 1.0, np.clip(ffV / Vb_prev, 0, 1)) if s > 0 else p_app
            p_app = np.trunc(p * 32767.0) / 32767.0
            wm_last = w.copy()
        elif ctrl == 'ff':
            p_app = np.trunc(np.clip(ffV / Vb_prev, 0, 1) * 32767.0) / 32767.0
            wm_last = w.copy()
        else:
            mf = next_fire == s
            if mf.any():
                idx = np.nonzero(mf)[0]
                k = kidx[idx]
                if meas == 'hub':
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
                    wm = np.where(k >= 1, (c - hist_c[idx, j]) / np.where(den > 0, den, 1.0) * TWO_PI / CPR, w0[idx])
                    dtk = np.where(k >= 1, tau - tau_prev[idx], Tloop[idx])
                    tau_prev[idx] = tau
                else:
                    okq = q_t[idx] <= t - OQ_READ + 1e-12
                    col = np.maximum(okq.sum(1) - 1, 0)
                    wm = q_v[idx, col]
                    dtk = np.where(k >= 1, t - tau_prev[idx], Tloop[idx])
                    tau_prev[idx] = t
                e = tgt[idx] - wm
                ff = ffV[idx]
                if ctrl == 'team':
                    upd = (k % 50) == 0
                    Vs[idx] = np.where(upd, Vb_prev[idx], Vs[idx])
                    vs = Vs[idx]
                    cap = np.minimum(vs, vcap)
                    out0 = kP[idx] * e + kI[idx] * I[idx]
                    pinned = ((ff + out0 >= cap) & (e > 0)) | ((out0 <= -ff) & (e < 0))
                    In = np.where(kI[idx] == 0, 0.0, np.where(pinned, I[idx], I[idx] + e * dtk))
                    I[idx] = In
                    V = np.clip(ff + kP[idx] * e + kI[idx] * In, 0.0, np.minimum(13.0, 1e9))
                    p = np.minimum(V / vs, 1.0)
                elif ctrl == 'rec':
                    Vs[idx] = Vb_prev[idx]
                    vs = Vs[idx]
                    big = e > thr[idx] * tgt[idx]
                    inz = np.abs(e) < iz[idx]
                    In = I[idx] + e * dtk
                    Vn = ff + kP[idx] * e + kI[idx] * In
                    sat = (Vn > vs) | (Vn < 0)
                    In = np.where(inz & ~sat & ~big, In, I[idx])
                    I[idx] = In
                    V = np.where(big, vs, ff + kP[idx] * e + kI[idx] * In)
                    p = V / vs
                else:
                    raise ValueError(ctrl)
                wm_last[idx] = wm
                p = np.clip(p, 0.0, 1.0)
                p = np.trunc(p * 32767.0) / 32767.0
                pend_p[idx] = p
                pend_step[idx] = s + lat_steps[idx, k]
                kidx[idx] = k + 1
                next_fire[idx] = fire[idx, np.minimum(k + 1, Kh - 1)]
                if st0 <= s < st1:
                    sm_s[idx] += wm; sm_q[idx] += wm * wm; sm_n[idx] += 1
        # plant
        pa = p_app
        Vb = (Voc + Rint * nmot * pa * kew * w / RM) / (1.0 + Rint * nmot * pa * pa / RM)
        Im = (pa * Vb - kew * w) / RM
        off_ = pa == 0.0
        if off_.any():
            Im = np.where(off_, 0.0, Im)
            Vb = np.where(off_, Voc, Vb)
        Vb_prev = Vb
        Tf = tfric(w, Tcf, bf, cf)
        phs = (s - cur_start) / nc
        act = (phs >= 0) & (phs < 1)
        if act.any():
            sc = np.where(act, 0.5 * (1 - np.cos(pi_c * phs)), 0.0)
            sd = np.where(act, pi_c / (2 * Tcon) * np.sin(pi_c * phs), 0.0)
            wd = (ktw * Im - Tf - kb * w * sd) / (J + kb * sc)
            wn = w + wd * DT
            phs2 = (s + 1 - cur_start) / nc
            sc2 = np.where(act, 0.5 * (1 - np.cos(pi_c * np.minimum(phs2, 1.0))), 0.0)
            Lb = kb * sc2 * wn
            ii = np.nonzero(act)[0]
            e_ball[ii, bi[ii]] += w[ii] * (Lb[ii] - Lb_prev[ii])
            Lb_prev = np.where(act, Lb, 0.0)
        else:
            wd = (ktw * Im - Tf) / J
            wn = w + wd * DT
        w = wn
        th = th + w * DT
        me = (s - cur_start) == nc - 1
        if me.any():
            ii = np.nonzero(me)[0]
            w_exit[ii, bi[ii]] = w[ii]
            bi[ii] += 1
            Lb_prev[ii] = 0.0
            cur_start[ii] = np.where(bi[ii] < nb, tb_step[ii, np.minimum(bi[ii], nb - 1)], 1 << 60)
        # metrics
        err = np.abs(w - tgt) / tgt
        after_last = s >= last_step
        after_first = s >= first_step
        for b in bands:
            last_out[b][after_last & (err > b)] = t
            last_out_first[b][after_first & (s < second_step) & (err > b)] = t
        win1 = after_first & (s < second_step)
        wmin1 = np.where(win1, np.minimum(wmin1, w), wmin1)
        if st0 <= s < st1:
            ss_s += w; ss_q += w * w; ss_n += 1

    res = {}
    mean_ss = ss_s / max(ss_n, 1)
    res['ss_bias_rpm'] = (mean_ss - tgt) * RPM
    res['ss_std_rpm'] = np.sqrt(np.maximum(ss_q / max(ss_n, 1) - mean_ss ** 2, 0)) * RPM
    mm = sm_s / np.maximum(sm_n, 1)
    res['meas_std_rpm'] = np.sqrt(np.maximum(sm_q / np.maximum(sm_n, 1) - mm ** 2, 0)) * RPM
    t_last = tb_step[:, -1] * DT
    for b in bands:
        lo = last_out[b]
        res[f'rec_last_{int(b*100)}'] = np.where(lo > T_end - 0.05, np.nan, np.where(lo < 0, 0.0, lo + DT - t_last))
        lo1 = last_out_first[b]
        res[f'rec_first_{int(b*100)}'] = np.where(lo1 < 0, 0.0, lo1 + DT - t_first)
    res['dip1_rpm'] = (wmin1 - tgt) * RPM
    res['w_touch'] = w_touch * RPM
    res['wm_touch'] = wm_touch * RPM
    res['w_exit'] = w_exit * RPM
    res['e_ball'] = e_ball
    res['tgt_rpm'] = tgt * RPM
    res['kb'] = kb
    if trace is not None:
        res['trace'] = np.array(trace)
    return res
