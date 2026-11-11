"""Independent bookkeeping of the flywheel sag / recovery model in sag_energy.py.
All flywheel numbers are [inf]/[asm] except the stall torque 1.47 kg-cm and 6000 rpm no-load, re-fetched from
https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/ (2026-11-10)."""
import json, math, os
HERE = os.path.dirname(os.path.abspath(__file__))
src = json.load(open(os.path.join(HERE, "..", "out", "sag_energy.json")))

R_W, R_B = 0.036, {"pollen": 0.0711 / 2, "nectar": 0.0914 / 2}
TAU_S = 1.47 * 9.80665 / 100          # N m from the vendor kg-cm figure
M = {"pollen": 0.0249, "nectar": 0.0413}
WF = 5400 * 2 * math.pi / 60

print(f"stall torque {TAU_S:.5f} N m (study 0.1442)")
# 1. closed form vs the study's table
worst = 0
for r in src["sag"]:
    k = 2 / 3 if r["spin"] == "shell" else 2 / 5
    s = 100 * (1 + k) / 2 * M[r["ball"]] * r["eta"] * R_W ** 2 / r["I"]
    worst = max(worst, abs(s - r["sag_pct"]))
print(f"1. per-ball sag closed form vs sag_energy.json: max |diff| {worst:.2e} pp")

# 2. impulse / energy bookkeeping for one ball (Pollen, eta .45, I .78e-4, 70 % free), three contact models
I = 0.78e-4
for name in ("pollen", "nectar"):
    m, rb = M[name], R_B[name]
    w0 = 0.7 * WF
    v = 0.45 * w0 * R_W
    print(f"2. {name}: v {v:.2f} m/s, spin if rolling {v / rb:.0f} rad/s (spin ratio 1.0)")
    for label, J1 in (("rolls on hood, thin shell  J1=(1+2/3)/2 mv", (1 + 2 / 3) / 2 * m * v),
                      ("rolls on hood, solid       J1=(1+2/5)/2 mv", (1 + 2 / 5) / 2 * m * v),
                      ("frictionless hood          J1=mv", m * v),
                      ("hood drags back 0.2 mv     J1=1.2 mv", 1.2 * m * v)):
        dw = J1 * R_W / I
        e_wheel = 0.5 * I * (w0 ** 2 - (w0 - dw) ** 2)
        if "frictionless" in label:
            ke = 0.5 * m * v ** 2 + 0.5 * (2 / 3 * m * rb ** 2) * (J1 * rb / (2 / 3 * m * rb ** 2)) ** 2
        elif "drags" in label:
            ke = 0.5 * m * v ** 2 * (1 + 2 / 3)
        else:
            k = 2 / 3 if "shell" in label else 2 / 5
            ke = 0.5 * m * v ** 2 * (1 + k)
        print(f"   {label}: sag {100 * dw / w0:5.1f}%  wheel energy out {e_wheel:.3f} J  ball KE {ke:.3f} J  dissipated {e_wheel - ke:+.3f} J")

# 3. rolling on a stationary hood: ball top moves at 2v, so the wheel (v/eta) can only drive it if eta <= 0.5
print("3. rolling constraint: top of ball at 2v vs wheel surface v/eta -> eta must be <= 0.50; eta 0.50 = no slip at the wheel")

# 4. sustained rate: tau(1-f)/dL, reproduce
worst = 0
for r in src["rate"]:
    c = (1 + 2 / 3) / 2 * M[r["ball"]] * r["eta"] * R_W ** 2
    w = r["frac"] * r["free"] * 2 * math.pi / 60
    rate = 0.1442 * (1 - r["frac"]) / (c * w)
    worst = max(worst, abs(rate - r["rate_per_s"]))
print(f"4. sustained rate reproduced, max |diff| {worst:.2e} balls/s")
# viscous-friction equivalence: tau_s(1 - w/6000rpm) - c w with c set so free = 5400 gives tau_s(1 - w/5400)
w = 0.8 * WF
lhs = TAU_S * (1 - w / (6000 * 2 * math.pi / 60)) - TAU_S * (1 - 5400 / 6000) * w / WF
print(f"   torque at 80% of 5400: vendor line minus viscous loss {lhs:.4f} N m vs model {TAU_S * 0.2:.4f} N m")

# 5. what the flywheel encoder sees after one ball (Pollen, eta .45, 70 % free, I .78e-4), ideal full-power recovery
name, f = "pollen", 0.7
c = (1 + 2 / 3) / 2 * M[name] * 0.45 * R_W ** 2
w0 = f * WF
wd = w0 * (1 - c / I)
tc = I * WF / 0.1442
def w_at(t):
    return min(w0, WF - (WF - wd) * math.exp(-t / tc))
t_rec = -tc * math.log((WF - w0) / (WF - wd))
print(f"5. {name} {f:.0%} free: drop to {100 * wd / w0:.1f}% of setpoint, full recovery after {1000 * t_rec:.0f} ms (ball gap 92 ms)")
for t_end in (0.010, 0.020, 0.030, 0.050, 0.070, 0.090):
    # OctoQuad-style 20 ms latched average ending t_end after the ball
    n = 200
    avg = sum(w_at(max(0.0, t_end - 0.020 + 0.020 * (i + 0.5) / n)) if t_end - 0.020 + 0.020 * (i + 0.5) / n >= 0 else w0 for i in range(n)) / n
    print(f"   20 ms window ending {1000 * t_end:3.0f} ms after the ball: apparent droop {100 * (1 - avg / w0):5.1f}%  (next ball sees {100 * (1 - w_at(1 / 10.9) / w0):.1f}%)")

# 6. burst at 10.9/s for one shooter vs the same channel alternating between two shooters (5.45/s each)
for rate in (10.9, 5.45):
    for name in ("pollen", "nectar"):
        c = (1 + 2 / 3) / 2 * M[name] * 0.45 * R_W ** 2
        cells = []
        for f in (0.6, 0.7, 0.8):
            w0 = f * WF
            w = w0
            sp = []
            for _ in range(5):
                sp.append(w)
                w *= 1 - c / I
                w = WF - (WF - w) * math.exp(-(1 / rate) / (I * WF / 0.1442))
                w = min(w, w0)
            cells.append(f"{f:.0%}: {100 * (1 - sp[-1] / sp[0]):4.1f}%")
        print(f"6. {rate:5.2f} balls/s {name}: ball-5 deficit at I .78e-4: " + "  ".join(cells))
