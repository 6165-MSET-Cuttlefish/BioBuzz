"""Checks: semi-analytic burst model for the IDEAL loop (cross-check of the study's oracle results).
Each ball: w -> w * J/(J+k) at touch (closed-form angular-momentum transfer, motor torque during contact credited
afterwards); between touches the wheel accelerates at full duty (battery Voc 12.5 V, Rint 0.1, the study's friction law)
until it reaches target, integrated with scipy. Exit deviation of ball n = w_touch_n / w_target - 1.
Also: steady-state sustained interval = E_ball / P_net, and the 10-ball energy-budget bound."""
import math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np
from scipy.integrate import solve_ivp
import cadcfg as C
import ballistics as B

RPM = 60 / (2 * math.pi)
R, KE, KT = 12.0 / 9.2, (12.0 - 0.25 * 12.0 / 9.2) / (5800 / RPM), 1.47 * 0.0980665 / 9.2
INT = [0.092, 0.105, 0.120, 0.135, 0.150, 0.175, 0.200, 0.250, 0.300]


def Tnet(w, Voc=12.5, Rint=0.1, T3=0.008, n=1, kt=KT):
    w3 = 3000 / RPM
    Tf = 0.3 * T3 * math.tanh(w) + 0.5 * T3 / w3 * w + 0.2 * T3 / w3 ** 2 * w * w
    Im = (Voc - KE * w) / (R + n * Rint)
    return n * kt * Im - Tf


def burst(J, k, wt, dt_iv, nb=10, n=1, T3=0.008):
    w = wt; devs = []
    for i in range(nb):
        devs.append(100 * (w / wt - 1))
        w = w * J / (J + k)
        sol = solve_ivp(lambda t, y: [Tnet(y[0], n=n, T3=T3) / J], (0, dt_iv), [w],
                        events=lambda t, y: y[0] - wt, rtol=1e-9, max_step=1e-3)
        w = min(sol.y[0, -1], wt)
    return devs


def fastest(J, k, wt, lo, hi, n=1, T3=0.008):
    best = None
    for iv in sorted(INT, reverse=True):
        d = burst(J, k, wt, iv, n=n, T3=T3)
        if min(d) >= lo and max(d) <= hi:
            best = iv
        else:
            break
    return '>300' if best is None else str(int(round(best * 1000)))


print('ideal-loop cross-check, ball 1 on nominal, reference window, f .45: semi-analytic check vs study oracle')
import json
S = json.load(open(os.path.join(HERE, '..', 'results', 'summary.json')))
A = dict(S['burst']); A.update(S['changes'])
for sh in ('pollen', 'nectar'):
    for pk, J in (('asbuilt', C.J_ASBUILT[sh]), ('steel82', C.J_ASBUILT[sh] + C.J_STEEL82), ('J4.0', 4e-4)):
        for dr in (1.0, 1.14):
            chk, stu = [], []
            for d in (36, 60, 84, 108, 132):
                wt, info = C.target(sh, d, 0.45, dr, J)
                if wt * RPM > 0.995 * 5533:
                    chk.append('inf'); stu.append('inf'); continue
                lo, hi = B.REF_WIN[d]
                chk.append(fastest(J, info['kb'], wt, lo, hi))
                r = A.get(f'{sh}|{pk}|{d}|0.45|{dr}|oracle|none|0.02')
                stu.append('inf' if r is None or r.get('infeasible') else ('>300' if r['fast_v_10'] is None else str(int(round(r['fast_v_10'] * 1000)))))
            print(f'{sh:6s} {pk:8s} x{dr:.2f}: check {"/".join(chk):26s} study {"/".join(stu)}')

print('\nsustained interval with no margin = E_wheel_per_ball / P_net (ms), and minimum J that buffers a 10-ball 92 ms burst within the low window')
for sh in ('pollen', 'nectar'):
    for d in (36, 84, 132):
        J = C.J_ASBUILT[sh]
        wt, info = C.target(sh, d, 0.45, 1.0, J)
        m = C.BALL[sh]['m']; v = info['v_exit_ms']
        E = (5 / 6) * m * v * v / 0.45 * (1 + info['kb'] / (2 * J))
        P = Tnet(wt) * wt
        need = 10 * E - P * 9 * 0.092          # energy the stored kinetic energy must supply over 10 balls at 92 ms
        lo = -B.REF_WIN[d][0] / 100
        Jmin = need / (0.5 * wt * wt * (1 - (1 - lo) ** 2)) if need > 0 else 0.0
        print(f'{sh} {d}: E_ball {E:.2f} J, P_net {P:.1f} W -> sustained {1000*E/P:.0f} ms/ball; 10 balls @92 ms need {need:+.1f} J from storage '
              f'-> J >= {Jmin:.2e} kg m^2 to stay above {-100*lo:.2f} % (as-built {J:.2e})')

print('\nfull-power recovery from the post-ball speed J/(J+k) w_t back to w_t: the sustained interval of the ideal loop, no margin')
for sh in ('pollen', 'nectar'):
    for pk, J in (('asbuilt', C.J_ASBUILT[sh]), ('steel82', C.J_ASBUILT[sh] + C.J_STEEL82)):
        for d in (36, 60, 84, 108, 132):
            wt, info = C.target(sh, d, 0.45, 1.0, J)
            w0 = wt * J / (J + info['kb'])
            ev = lambda t, y: y[0] - wt
            ev.terminal = True
            ev.direction = 1
            sol = solve_ivp(lambda t, y: [Tnet(y[0]) / J], (0, 5.0), [w0], events=ev, rtol=1e-9, max_step=1e-3)
            tr = sol.t_events[0][0]
            print(f'{sh} {pk:8s} {d}: recovery {1000*tr:5.0f} ms (average power {0.5*J*(wt*wt-w0*w0)/tr:5.1f} W)')
