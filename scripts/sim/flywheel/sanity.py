import os, sys, math, time, json
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np
import flysim as F

out = {}
print(f"Motor: RM={F.RM:.4f} ohm  KT={F.KT:.5f} N.m/A  KE={F.KE:.5f} V.s/rad  (KT/KE={F.KT/F.KE:.3f})")
out['motor'] = dict(RM=F.RM, KT=F.KT, KE=F.KE)

# 1. free speed / stall
for lvl, T3 in F.FRICTION_LEVELS.items():
    a, b, c = F.friction_coeffs(T3)
    for Voc, Rint in ((12.0, 0.0), (12.5, 0.1), (13.5, 0.05)):
        ws = F.free_speed_rpm(Voc, Rint, a, b, c)
        Vb = (Voc + Rint * F.KE * ws / F.RPM / F.RM) / (1 + Rint / F.RM)
        Im = (Vb - F.KE * ws / F.RPM) / F.RM
        print(f"free speed fric={lvl:4s} Voc={Voc} Rint={Rint}: {ws:7.0f} rpm, I={Im:.2f} A")
        out[f'free_{lvl}_{Voc}_{Rint}'] = (ws, Im)
print(f"stall at 12 V, Rint=0: I={12/F.RM:.2f} A, torque={F.KT*12/F.RM:.4f} N.m (spec 9.2 A, {F.T_STALL:.4f} N.m)")

# 2. steady-state power at 3000 rpm
w3 = 3000 / F.RPM
for lvl, T3 in F.FRICTION_LEVELS.items():
    a, b, c = F.friction_coeffs(T3)
    V = F.v_ss(w3, a, b, c)
    print(f"steady 3000 rpm fric={lvl}: V_motor={V:.3f} V -> power {V/12:.3f} @12V, {V/13:.3f} @13V; I={T3/F.KT:.2f} A")
    out[f'ss3000_{lvl}'] = dict(V=V, p12=V / 12, p13=V / 13)
a, b, c = F.friction_coeffs(0.008)
ks, kv = F.kskv_fit(a, b, c)
tab = F.ff_table_volts(np.array([w3]), a, b, c)
print(f"kS={ks[0]:.3f} V kV={kv[0]:.5f} V/(rad/s); at 3000 rpm: true {F.v_ss(w3,a,b,c):.4f}, table {tab[0]:.4f}, kSkV {ks[0]+kv[0]*w3:.4f}")

# 3. time constants and open-loop spin-up
print("\nJ (kg m^2) | tau_mech=J*RM/(KT*KE) s | full-power 0->3000 (s) @12.5V Rint .1 | FF-only 0->2950 (s)")
for J in (5e-5, 1e-4, 2e-4, 5e-4):
    tau = J * F.RM / (F.KT * F.KE)
    w, t, dt = 0.0, 0.0, F.DT
    Voc, Rint = 12.5, 0.1
    while w * F.RPM < 3000 and t < 20:
        Vb = (Voc + Rint * F.KE * w / F.RM) / (1 + Rint / F.RM)
        Im = (Vb - F.KE * w) / F.RM
        w += (F.KT * Im - (a * math.tanh(w) + b * w + c * w * w)) / J * dt
        t += dt
    tfull = t
    w, t = 0.0, 0.0
    Vff = F.v_ss(w3, a, b, c)
    p = Vff / Voc
    while w * F.RPM < 2950 and t < 30:
        Vb = (Voc + Rint * p * F.KE * w / F.RM) / (1 + Rint * p * p / F.RM)
        Im = (p * Vb - F.KE * w) / F.RM
        w += (F.KT * Im - (a * math.tanh(w) + b * w + c * w * w)) / J * dt
        t += dt
    print(f"{J:.0e} | {tau:.3f} | {tfull:.3f} | {t:.3f}")
    out[f'J_{J}'] = dict(tau=tau, t_full=tfull, t_ff=t)

# 4. ball load: open-loop speed loss and energy
print("\nBall: k=(5/6) m 0.45 r^2; open-loop w_end/w0 = J/(J+k); dip at 3000 rpm (rpm) [no motor torque]")
for J in (5e-5, 1e-4, 2e-4, 5e-4):
    row = []
    for m in (0.010, 0.030, 0.060):
        k = F.ball_k(m)
        row.append(3000 * (1 - J / (J + k)))
    print(f"J={J:.0e}: m=10g {row[0]:.0f}  30g {row[1]:.0f}  60g {row[2]:.0f}")
    out[f'dip_open_{J}'] = row
m = 0.03
vs = w3 * F.WHEEL_R
v = 0.45 * vs
print(f"surface speed at 3000 rpm {vs:.2f} m/s, exit {v:.2f} m/s; ball KE (incl. spin) {F.SHELL*m*v*v:.3f} J; "
      f"wheel energy lost (impulse model) {F.SHELL*m*v*vs:.3f} J (ratio {vs/v:.2f})")
print(f"full-power net torque at 3000 rpm 12.5 V: {(F.KT*(12.5-F.KE*w3)/(F.RM+0.1) - (a*1+b*w3+c*w3*w3)):.4f} N.m; "
      f"time to restore 30 g ball momentum {F.ball_k(0.03)*w3/(F.KT*(12.5-F.KE*w3)/(F.RM+0.1) - (a+b*w3+c*w3*w3)):.3f} s")

# 5. measurement noise from timestamp error (analytic): sigma_dt = sqrt(2)*A/sqrt(3)
print("\nWindowed-velocity noise at 3000 rpm from timestamp error (std, rpm), analytic:")
for A in (0.0, 0.5e-3, 1.5e-3):
    print(f"  ts +-{A*1e3:.1f} ms: " + "  ".join(f"W={W*1e3:.0f}ms {3000*math.sqrt(2)*A/math.sqrt(3)/W:.1f}" for W in (0.015, 0.03, 0.05)))
print(f"  quantisation 1 count over 15 ms = {60/8192/0.015:.2f} rpm; 28 CPR over 40 ms = {60/28/0.04:.1f} rpm")

# 6. benchmark the vectorised simulator
N = 64
P = dict(J=1e-4, m=0.03, Tc=0.02, Voc=12.5, Rint=0.1, fric=0.008, T=0.015, W=0.05, tsA=0.5e-3, ffk=1.0, kAk=1.0,
         seed=np.arange(N) % 8)
scen = dict(T_end=2.0, w0_rpm=3000, target_rpm=3000, shots=[1.0, 1.25, 1.5])
t0 = time.time()
r = F.simulate(P, 'ffpi', dict(kP=0.02 * F.RPM, kI=0.05 * F.RPM), scen)
print(f"\nbench: N={N}, 2.0 s sim: {time.time()-t0:.2f} s; e_touch mean {np.nanmean(r['e_touch'],0)}, dip {np.mean(r['dip']):.0f}")
json.dump(out, open(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'sanity.json'), 'w'), indent=1, default=float)
