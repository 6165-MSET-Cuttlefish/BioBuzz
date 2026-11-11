"""Robot numbers (CAD / vendor / estimate) and derived shot targets for the flywheel study."""
import math
import numpy as np
import flysim_cad as F
import ballistics as B

IN = 0.0254
RPM = F.RPM

# ---- inertia (kg m^2), wheel-referred ----
J_DESIGN = 0.78e-4            # [inf] CAD model as of 2026-11-10: wheel 7.3e-5 + hub + shaft + pulleys (no rotor, no roller)
J_ROTOR = 5.0e-6              # [est] RS-555-class armature ~65 g, r ~12.2 mm, 0.5 m r^2 -> 4.8e-6; range 3e-6..8e-6
STEALTH_2IN_MASS = 0.1 * 0.45359237   # [vendor] AndyMark Stealth 2 in family weight list "0.03, 0.1, 0.17, 0.3 lbs"
J_ROLLER = 0.6 * STEALTH_2IN_MASS * (1.0 * IN) ** 2 + 0.2e-5   # [est] 0.6 m r^2 (TPU tread on PC hub) + shaft/pulley/hub
J_ASBUILT = {'pollen': J_DESIGN + J_ROTOR + J_ROLLER,      # roller belted 1:1 [CAD]
             'nectar': J_DESIGN + J_ROTOR}                 # only the wheel is modelled for Nectar [CAD]; no roller assumed
J_STEEL82 = 1651e-7           # [vendor] goBILDA 3628-0032-0082, 152 g, MOI 1651 g cm^2 (current page; old URL slug says 840)
J_STEEL82_OLD = 840e-7
J_SWEEP = [1.2e-4, 1.6e-4, 2.4e-4, 4.0e-4]

# 96 mm Rhino (3614-0014-0096, 142 g [vendor]); inertia scaled from the 72 mm wheel's I/(m r^2) = 0.663 [est]
J_WHEEL72 = 7.3e-5
K72 = J_WHEEL72 / (0.085 * 0.036 ** 2)
J_WHEEL96 = K72 * 0.142 * 0.048 ** 2

BALL = {'pollen': dict(m=0.0249, m_lo=0.0235, m_hi=0.0272, R=0.0356, dia_in=2.8),
        'nectar': dict(m=0.0413, m_lo=0.039, m_hi=0.045, R=0.0457, dia_in=3.6)}
HOOD = {'pollen': dict(Rhood=4.173 * IN, wrap=31.0, wrap_max=54.0, squeeze=1.1e-3),
        'nectar': dict(Rhood=4.965 * IN, wrap=23.0, wrap_max=32.0, squeeze=1.3e-3)}

R_WHEEL = 0.036
F_EXIT = [0.35, 0.45, 0.50]
DRAG = [1.00, 1.14]
BASE = dict(Voc=12.5, Rint=0.10, fric=0.008, T=0.015, W=0.050, tsA=0.5e-3, Nq=0.020)
LAT = {'team': (0.008, 0.012), 'rec': (0.0015, 0.003), 'oracle': (0.0, 0.0), 'ff': (0.0, 0.0)}

# Scoring windows (%): ../../ballistics/windows_nodrag.py, min-speed line, azimuth 0, 14 in exit, 2.8 in ball
WIN_REF = B.REF_WIN
# Hood "known deficit" bound for the optimistic line (rpm): 80 at 36 in rising to 170 at 72 in, 170 beyond
def hood_absorb_rpm(d):
    return float(np.interp(d, [36, 72], [80, 170]))


def path_len(sh, r_wheel=R_WHEEL, wrap=None):
    h = HOOD[sh]; b = BALL[sh]
    Rc = r_wheel + b['R'] - h['squeeze'] / 2
    return Rc * math.radians(h['wrap'] if wrap is None else wrap)


def contact_time(sh, v_exit_ms, r_wheel=R_WHEEL, wrap=None):
    """Tc = 2 L / v_exit: ball centre accelerates from rest to v_exit along an arc L = R_c * wrap (mean speed v/2)."""
    return 2.0 * path_len(sh, r_wheel, wrap) / v_exit_ms


SHOTS = {sh: {d: B.shot(sh, d) for d in B.DS} for sh in B.SHOOTERS}
SHOTS_CAL = {sh: {d: B.shot(sh, d, calibrated=True) for d in B.DS} for sh in B.SHOOTERS}


def kball(m, f, r):
    return F.SHELL * m * f * r * r


def rho_ff(J, kb, Tc, tgt, nmot=1, g=1.0, ktm=F.KT_STALL, Voc=12.5, Rint=0.1, fric=0.008):
    """w_exit / w_target for one ball from a steady wheel held by exact FF (no feedback acts within contact)."""
    P = dict(J=J, m=kb / (F.SHELL * 0.45 * R_WHEEL ** 2), Tc=Tc, Voc=Voc, Rint=Rint, fric=fric, T=0.015, W=0.05,
             tsA=0.0, seed=1, tgt=tgt, r=R_WHEEL, f=0.45, nmot=nmot, g=g, ktm=ktm, Nq=0.02, lat_lo=0, lat_hi=0,
             kP=0, kI=0, iz=0, thr=0, spacing=1.0)
    res = F.simulate(P, 'ff', 'hub', nb=1, warm=0.05, T_end=0.05 + Tc + 0.01, shot_jit=0.0)
    return float(res['w_exit'][0, 0] / res['tgt_rpm'][0])


def target(sh, d, f, drag, J, r=R_WHEEL, nmot=1, g=1.0, ktm=F.KT_STALL, m=None, shots=None, wrap=None):
    """Flywheel target (rad/s) so that the ball exits at the needed speed, accounting for the in-contact dip."""
    s = (shots or SHOTS)[sh][d]
    v = s['v_in_s'] * IN * drag
    mb = BALL[sh]['m'] if m is None else m
    w_exit = v / (f * r)
    Tc = contact_time(sh, v, r, wrap)
    kb = kball(mb, f, r)
    rho = J / (J + kb)
    for _ in range(2):
        rho = rho_ff(J, kb, Tc, w_exit / rho, nmot, g, ktm)
    return w_exit / rho, dict(v_exit_ms=v, w_exit_rpm=w_exit * RPM, Tc=Tc, kb=kb, rho=rho)
