"""Checks: closed-form limits for the flywheel plant and ball contact, independent ODE integration
(scipy RK45) of one shot, energy bookkeeping, motor power, the reflected-rotor-inertia formula and the net
wheel torque against gear ratio. Imports the study's modules only to compare against them."""
import math, os, sys, json
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import numpy as np
from scipy.integrate import solve_ivp
import flysim_cad as F
import cadcfg as C
import run_cad as Rc

RPM = 60 / (2 * math.pi)
out = {}

# ---- 1. motor constants from the vendor page (independent arithmetic) ----
V, I0, Is, Wf = 12.0, 0.25, 9.2, 5800 / RPM
Ts_page = 1.47 * 0.0980665
Ts_pdf = 1.74 * 0.0980665
oz_in = 20.45 * 0.00706155
R = V / Is
ke = (V - I0 * R) / Wf
kt = Ts_page / Is
print(f"20.45 oz-in = {oz_in:.5f} N m = {oz_in/0.0980665:.4f} kg cm  (page 1.47, PDF 1.74)")
print(f"R {R:.4f}  ke {ke:.5f}  kt(1.47) {kt:.5f}  kt/ke {kt/ke:.3f}")
print(f"stall torque implied by kt=ke and (Istall-I0): {ke*(Is-I0):.4f} N m = {ke*(Is-I0)/0.0980665:.3f} kg cm "
      f"-> PDF's 1.74 kg cm is within {100*(1.74/(ke*(Is-I0)/0.0980665)-1):+.1f}% of the kt=ke value; page's 1.47 is {100*(1.47/(ke*(Is-I0)/0.0980665)-1):+.1f}%")
print(f"study: R {F.RM:.4f} ke {F.KE:.5f} kt {F.KT_STALL:.5f}")
out['motor'] = dict(R=R, ke=ke, kt=kt, kt_over_ke=kt / ke, stall_kgcm_if_kt_eq_ke=ke * (Is - I0) / 0.0980665)

# ---- 2. closed-form momentum / energy for a free wheel (no motor, no friction) ----
def free_shot(J, m, f, r, w0, Tc):
    """Independent: d/dt[(J + k s(t)) w] = 0 with s raised cosine; RK45."""
    k = 5 / 6 * m * f * r * r
    def s(t): return 0.5 * (1 - math.cos(math.pi * min(max(t / Tc, 0), 1)))
    def ds(t): return (math.pi / (2 * Tc)) * math.sin(math.pi * t / Tc) if 0 <= t <= Tc else 0.0
    def rhs(t, y):
        w, E = y
        wd = -k * w * ds(t) / (J + k * s(t))
        # energy into ball = integral of w dL_ball, L_ball = k s w
        dL = k * (ds(t) * w + s(t) * wd)
        return [wd, w * dL]
    sol = solve_ivp(rhs, (0, Tc), [w0, 0.0], rtol=1e-10, atol=1e-12, max_step=Tc / 200)
    return sol.y[0, -1], sol.y[1, -1], k

rows = []
for sh in ('pollen', 'nectar'):
    m = C.BALL[sh]['m']
    for J in (C.J_ASBUILT[sh], 2.4e-4):
        for f in (0.35, 0.45, 0.50):
            w0 = 4000 / RPM
            we, Eb, k = free_shot(J, m, f, 0.036, w0, 0.013)
            rho_cf = J / (J + k)
            dE_wheel = 0.5 * J * (w0 ** 2 - we ** 2)
            v = f * 0.036 * we
            KE = 5 / 6 * m * v * v
            ratio_cf = (1 / f) * (1 + k / (2 * J))
            rows.append(dict(sh=sh, J=J, f=f, rho_ode=we / w0, rho_cf=rho_cf, E_ball_ode=Eb, dE_wheel=dE_wheel,
                             KE_ball=KE, ratio=dE_wheel / KE, ratio_cf=ratio_cf))
            print(f"{sh} J {J:.3e} f {f}: rho ODE {we/w0:.5f} closed {rho_cf:.5f} | wheel loss {dE_wheel:.4f} J, "
                  f"int w dL {Eb:.4f} J | ball KE(5/6 m v^2) {KE:.4f} -> wheel/KE {dE_wheel/KE:.4f} (closed (1/f)(1+k/2J) {ratio_cf:.4f})")
out['free_wheel'] = rows

for sh in ('pollen', 'nectar'):
    J = C.J_ASBUILT[sh]; m = C.BALL[sh]['m']
    for d in (36, 132):
        tgt, info = C.target(sh, d, 0.45, 1.0, J)
        print(f"{sh} {d}: study target {tgt*RPM:.1f} rpm, rho {info['rho']:.4f}, closed-form J/(J+k) {J/(J+info['kb']):.4f}, Tc {1000*info['Tc']:.2f} ms")

# ---- 3. independent single-shot ODE WITH the motor held at FF duty (no feedback in contact) ----
def motor_shot(J, m, f, r, w0, Tc, Voc=12.5, Rint=0.1, T3=0.008, kt_=kt, ke_=ke, R_=R):
    w3 = 3000 / RPM
    Tcf, bf, cf = 0.3 * T3, 0.5 * T3 / w3, 0.2 * T3 / w3 ** 2
    Tf = lambda w: Tcf * math.tanh(w) + bf * w + cf * w * abs(w)
    # FF duty that holds w0 at steady state (solve p from battery+motor algebra)
    Vneed = ke_ * w0 + R_ * Tf(w0) / kt_
    Im0 = Tf(w0) / kt_
    Vb0 = Voc - Rint * (Vneed / Voc) * Im0  # first guess
    p = Vneed / Vb0
    for _ in range(50):
        Vb0 = Voc - Rint * p * Im0
        p = Vneed / Vb0
    k = 5 / 6 * m * f * r * r
    def s(t): return 0.5 * (1 - math.cos(math.pi * min(max(t / Tc, 0), 1)))
    def ds(t): return (math.pi / (2 * Tc)) * math.sin(math.pi * t / Tc) if 0 <= t <= Tc else 0.0
    def rhs(t, y):
        w = y[0]
        Vb = (Voc + Rint * p * ke_ * w / R_) / (1 + Rint * p * p / R_)
        Im = (p * Vb - ke_ * w) / R_
        return [(kt_ * Im - Tf(w) - k * w * ds(t)) / (J + k * s(t))]
    sol = solve_ivp(rhs, (0, Tc), [w0], rtol=1e-10, atol=1e-12, max_step=Tc / 400)
    return sol.y[0, -1] / w0

for sh in ('pollen', 'nectar'):
    J = C.J_ASBUILT[sh]; m = C.BALL[sh]['m']
    for d in (36, 84, 132):
        tgt, info = C.target(sh, d, 0.45, 1.0, J)
        rho_check = motor_shot(J, m, 0.45, 0.036, tgt, info['Tc'])
        print(f"{sh} {d}: rho (motor at FF through contact) study {info['rho']:.5f} check RK45 {rho_check:.5f}")

# ---- 4. motor power and free speed, independent ----
def net_power(w, Voc, Rint=0.1, T3=0.008, n=1, g=1.0, kt_=kt):
    w3 = 3000 / RPM
    Tf = 0.3 * T3 * math.tanh(w) + 0.5 * T3 / w3 * w + 0.2 * T3 / w3 ** 2 * w * w
    wm = w / g
    # n motors in parallel at full duty: battery current n*Im
    Im = (Voc - ke * wm) / (R + n * Rint)
    return (n * kt_ * Im / g - Tf) * w

for sh in ('pollen', 'nectar'):
    for d in (36, 84, 132):
        tgt, info = C.target(sh, d, 0.45, 1.0, C.J_ASBUILT[sh])
        P_check = net_power(tgt, 12.5)
        P_study = F.avail_power(tgt, 12.5, 0.1, 0.008)
        print(f"{sh} {d} in @ {tgt*RPM:.0f} rpm: net power check {P_check:.2f} W, study {P_study:.2f} W; 2 motors check {net_power(tgt,12.5,T3=0.010,n=2):.2f} W study {F.avail_power(tgt,12.5,0.1,0.010,2):.2f}")

# max gross mechanical power of one motor at 12 V (no Rint): kt*V^2/(4 R ke)
Pmax = kt * V * V / (4 * R * ke)
print(f"one motor, 12 V, no Rint: peak gross mechanical power {Pmax:.1f} W at {V/(2*ke)*RPM:.0f} rpm motor speed (kt=ke: {ke*V*V/(4*R*ke):.1f} W)")
out['Pmax_gross_12V'] = Pmax

# ---- 5. reflected rotor inertia with g = wheel speed / motor speed ----
for g in (0.8, 1.25, 1.5):
    wm_over_ww = 1 / g
    print(f"g {g}: motor turns at w/g -> reflected rotor J = J_r/g^2 = {C.J_ROTOR/g**2:.2e}"
          f" (Pollen J {C.J_ASBUILT['pollen']-C.J_ROTOR+C.J_ROTOR/g**2:.3e}, run_cad.plants {Rc.plants('pollen')[f'g{g}']['J']:.3e})")
# check the sim's convention: kew = KE/g means back-EMF = KE * w_wheel / g = KE * w_motor
ktw, kew = F.plant_consts(1, 1.5, F.KT_STALL)
print(f"plant_consts(g=1.5): kew {kew:.5f} = KE/1.5 -> motor speed = wheel speed / g; so rotor reflects as 1/g^2")

# ---- 6. f upper bound: ball rolling without slip on a fixed hood exits at half the wheel surface speed ----
print("f <= 0.5 for a ball rolling on a fixed hood (centre speed = (wheel surface + 0)/2 at no slip); f = 0.50 is the physical ceiling")
for sh in ('pollen', 'nectar'):
    rho = C.target(sh, 84, 0.45, 1.0, C.J_ASBUILT[sh])[1]['rho']
    print(f"{sh}: if a test reports v/(w_touch r) = 0.45, the model's f (vs exit wheel speed) is {0.45/rho:.3f}")

print('net wheel torque at full duty vs gear ratio g (wheel/motor), 12.5 V, Rint 0.1, friction 0.008; ratio to 1:1')
gs = np.round(np.arange(0.50, 3.0001, 0.01), 2)
i1 = int(np.where(gs == 1.0)[0][0])
for sh, d in (('pollen', 84), ('pollen', 132), ('nectar', 84), ('nectar', 132)):
    w, _ = C.target(sh, d, 0.45, 1.0, C.J_ASBUILT[sh])
    fr = F.tfric(w, *F.friction_coeffs(0.008))
    net = np.array([F.avail_power(w, 12.5, 0.1, 0.008, 1, g) / w for g in gs])
    gross = net + fr
    ia, ib = int(np.argmax(net)), int(np.argmax(gross))
    at = lambda g: net[int(np.where(gs == g)[0][0])] / net[i1]
    print(f"{sh} {d} in ({w*RPM:.0f} rpm): best g net {gs[ia]:.2f} -> {net[ia]/net[i1]:.2f}x net, gross best g {gs[ib]:.2f} -> "
          f"{gross[ib]/gross[i1]:.2f}x; g1.25 {at(1.25):.2f}x, g1.5 {at(1.5):.2f}x net")

os.makedirs(os.path.join(HERE, 'results'), exist_ok=True)
json.dump(out, open(os.path.join(HERE, 'results', 'check_physics.json'), 'w'), indent=1, default=float)
