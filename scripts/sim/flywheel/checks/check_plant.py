"""Plant checks against analytic answers."""
import os, sys, math
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import numpy as np
import flysim as F

print(f"DT={F.DT} KT={F.KT:.5f} KE={F.KE:.5f} RM={F.RM:.4f}")
base = dict(m=0.03, Tc=0.015, Voc=12.5, Rint=0.0, fric=0.0, T=0.015, W=0.05, tsA=0.0, ffk=1.0, kAk=1.0)


def P_(N, **kw):
    d = {k: np.full(N, v, float) for k, v in base.items()}
    for k, v in kw.items():
        d[k] = np.asarray(v, float) * np.ones(N)
    d['seed'] = np.arange(N) + 1
    return d


# 1. Open-loop ball: float, no friction -> w_exit/w_touch must equal J/(J+k) (angular momentum), energy ratio 1/0.45
print("\n1. open-loop momentum check (p=0 float, no friction, Rint 0)")
Js = [5e-5, 1e-4, 2e-4, 5e-4]
for m in (0.01, 0.03, 0.06):
    for tc in (0.015, 0.03):
        P = P_(4, J=Js, m=m, Tc=tc)
        scen = dict(T_end=0.8, w0_rpm=3000.0, target_rpm=0.0, shots=[0.5], shot_jit=0.0)
        r = F.simulate(P, 'bb', {}, scen)
        wt, we = r['w_touch'][:, 0], r['w_exit'][:, 0]
        k = F.ball_k(m)
        an = np.array(Js) / (np.array(Js) + k)
        Ewheel = 0.5 * np.array(Js) * ((wt / F.RPM) ** 2 - (we / F.RPM) ** 2)
        v = F.EXIT_FRAC * F.WHEEL_R * we / F.RPM
        Eball = F.SHELL * m * v * v
        print(f"  m={m*1e3:.0f}g Tc={tc*1e3:.0f}ms  sim/analytic exit ratio: " +
              ' '.join(f"{a:.5f}/{b:.5f}" for a, b in zip(we / wt, an)) +
              "  | E_wheel/E_ball: " + ' '.join(f"{x:.3f}" for x in Ewheel / Eball))

# 2. Spin-up with full power, no friction, Rint 0: t(2950) = -tau ln(1 - w/w_inf)
print("\n2. spin-up vs analytic exponential (oracle, fric 0, Rint 0, Voc 12.5)")
P = P_(4, J=Js)
r = F.simulate(P, 'oracle', {}, dict(T_end=3.0, w0_rpm=0.0, target_rpm=3000.0, shots=[]))
winf = 12.5 / F.KE
for J, tf in zip(Js, r['t_first50']):
    tau = J * F.RM / (F.KT * F.KE)
    ta = -tau * math.log(1 - (2950 / F.RPM) / winf)
    print(f"  J={J:.0e}: sim t(2950) {tf:.4f} s  analytic {ta:.4f} s  err {100*(tf-ta)/ta:+.2f}%")

# 3. Feed-forward only (ffpi with kP=kI=0): steady-state bias must be ~0 (table point at 3000 is exact)
print("\n3. FF-only steady state (table FF, mid friction, Rint 0.1, 100 ms Vs filter)")
P = P_(4, J=Js, Rint=0.1, fric=0.008)
r = F.simulate(P, 'ffpi', dict(kP=0.0, kI=0.0), dict(T_end=12.0, w0_rpm=0.0, target_rpm=3000.0, shots=[], ss_win=(11.0, 12.0)))
print("  ss_bias rpm:", np.round(r['ss_bias'], 2))
P = P_(4, J=Js, Rint=0.1, fric=0.008)
r = F.simulate(P, 'ffp', dict(kP=0.0), dict(T_end=12.0, w0_rpm=0.0, target_rpm=3000.0, shots=[], ss_win=(11.0, 12.0)))
print("  kS+kV-only ss_bias rpm:", np.round(r['ss_bias'], 2))
for tr in (2000.0, 2500.0, 3500.0, 4000.0, 4500.0):
    r1 = F.simulate(P_(1, J=1e-4, Rint=0.1, fric=0.008), 'ffpi', dict(kP=0.0, kI=0.0), dict(T_end=6.0, w0_rpm=tr, target_rpm=tr, shots=[], ss_win=(5.0, 6.0)))
    r2 = F.simulate(P_(1, J=1e-4, Rint=0.1, fric=0.008), 'ffp', dict(kP=0.0), dict(T_end=6.0, w0_rpm=tr, target_rpm=tr, shots=[], ss_win=(5.0, 6.0)))
    print(f"  target {tr:.0f}: table-FF bias {r1['ss_bias'][0]:+.1f} rpm, kS+kV bias {r2['ss_bias'][0]:+.1f} rpm")

# 4. Measured-speed noise: effective window and std at ts +-0.5 ms (true speed held constant: big J, FF only)
print("\n4. windowed-measurement noise, true w constant (J=1e3, FF only), T=15+-2 ms")
for W in (0.015, 0.03, 0.05):
    for A in (0.5e-3, 1.5e-3):
        N = 8
        P = P_(N, J=1e3, Rint=0.1, fric=0.008, W=W, tsA=A)
        r = F.simulate(P, 'ffpi', dict(kP=0.0, kI=0.0), dict(T_end=3.0, w0_rpm=3000.0, target_rpm=3000.0, shots=[], ss_win=(0.2, 3.0)))
        an = 3000 * math.sqrt(2) * A / math.sqrt(3) / W
        print(f"  W={W*1e3:.0f} ms ts+-{A*1e3:.1f}: sim meas std {np.mean(r['ssm_std']):6.1f} rpm  analytic(W nominal) {an:6.1f}")
# effective window lengths with the 'oldest sample with age<=W, at least 1 back' rule
rng = np.random.default_rng(0)
for T in (0.008, 0.015, 0.025):
    for W in (0.015, 0.03, 0.05):
        dts = T + 0.002 * rng.uniform(-1, 1, 200000)
        t = np.cumsum(dts)
        eff = []
        for i in range(20, 20000):
            j = i - 1
            while j - 1 >= 0 and t[i] - t[j - 1] <= W:
                j -= 1
            eff.append(t[i] - t[j])
        eff = np.array(eff)
        print(f"  T={T*1e3:.0f} W={W*1e3:.0f}: effective window mean {eff.mean()*1e3:5.1f} ms, min {eff.min()*1e3:5.1f}, max {eff.max()*1e3:5.1f}; "
              f"rms(1/W_eff)*W_nom = {np.sqrt(np.mean((W/eff)**2)):.2f}")
