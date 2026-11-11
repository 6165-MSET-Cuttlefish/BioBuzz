# Shooter simulations

The simulations and calculations behind [`docs/flywheel-control-findings.md`](../../docs/flywheel-control-findings.md)
and [`docs/shot-calculation-findings.md`](../../docs/shot-calculation-findings.md). The conclusions are on those pages;
this file says how to rerun the numbers and where measured values go.

| folder | what it is |
|---|---|
| `flywheel/` | generic flywheel control study: one motor, a 96 mm wheel, assumed inertia |
| `shotcalc/` | generic shot-calculation study: a 2.8 in ball from a 14 in exit, assumed shooter map |
| `cad/flywheel/` | the flywheel study rerun with the robot's numbers from the CAD model as of 2026-11-10 |
| `cad/shotcalc/` | the shot study rerun with the robot's hood geometry and exit points from the same CAD model |
| `ballistics/` | closed-form (drag-free) shot calculations, plus one quadratic-drag repeat |

**Every physical input that decides the answers is assumed** in the generic studies: flywheel inertia, ball mass, drag
and contact time, and how RPM and hood position map to exit speed and angle. The CAD reruns take the wheel, motor,
gearing, hood travel and exit points from the CAD model as of 2026-11-10
([`docs/robots-findings.md`](../../docs/robots-findings.md)), but the exit-speed fraction, inertia, friction, torque
constant, ball masses and drag are still not measured. When one is measured, set it where
[Knobs for measured values](#knobs-for-measured-values) says, delete whatever it invalidates, rerun the steps that
feed the numbers you care about, and update the page.

## Before running

- Python 3 with numpy. Steps that need scipy as well: in `flywheel/`, everything that runs the Kalman-filter
  controller `kf` (`run_all.py`, `checks/exp_rank.py`, `checks/check_ctrl.py kfqw` and `est`, `checks/check_misc.py
  dt`); in `cad/flywheel/checks/`, `check_physics.py` and `check_burst_bound.py`; in `cad/shotcalc/checks/`,
  `indep.py`, `exp_misc.py` and `probe_backspin.py`. Everything else needs numpy only.
- Run on 2026-11-10 with Python 3.14.7, numpy 2.4.6 and scipy 1.17.1.
- Run every command from the repo root. Each script finds its files relative to its own folder.
- Everything a script writes lands next to it, and `scripts/sim/.gitignore` keeps all of it out of git. Scripts
  that only print say so in their tables; redirect them to a `.txt` next to the script to keep the output.
- Times below are wall-clock, measured on 2026-11-10 on a busy machine, with the process counts the scripts set
  themselves: 8 or 10 workers for the flywheel stages, 6 for `cad/flywheel/run_cad.py` and 8 for its sensitivity
  and rerun steps; one process for study 1 and the single checks, 4 to 8 for the other shot steps. Treat them as
  rough: under heavy load two `run_cad.py` stages once took 11 and 30 minutes.

## Flywheel control: `flywheel/`

### What it models

All simulated. The model is in `flysim.py`; its constants and their sources are in the comments there.

- **Motor (a generic choice, not the robot's):** one goBILDA 5203 1:1 (6000 RPM) driving a 96 mm flywheel directly,
  from the vendor spec (12 V; 6000 RPM and 0.25 A free; 9.2 A and 1.47 kg·cm stall; URL in `flysim.py`). The spec
  isn't self-consistent (torque constant over back-EMF constant is 0.843); the torque constant comes from stall, the
  conservative choice. No inductance, PWM as average voltage, a 12.5 V battery behind 0.10 Ω, and friction of
  0.008 N·m at 3000 RPM (30% Coulomb, 50% viscous, 20% quadratic). The plant steps at 5 kHz. The robot's shooters
  use a goBILDA 5000-0002-4008 through a 1:1 belt (Nectar's ratio assumed) on a 72 mm wheel; `cad/flywheel/` reruns
  the study with those.
- **Ball (derived, not measured):** during a 15 to 30 ms contact the ball takes angular momentum k·ω from the wheel,
  k = (5/6) · m · 0.45 · r², for a thin shell rolling on a fixed hood off a 96 mm wheel. Masses of 10, 30 and 60 g,
  in bursts of 3 balls 150, 250 or 400 ms apart.
- **Measurement and output:** the 8192-count encoder is read every 15 ± 2 ms loop; each sample's timestamp is off
  by up to ±0.5 ms (uniform); RPM is the count difference to the oldest sample no older than the window (50 ms), the
  same rule as `measure()` in `biobuzz/modules/Shooter.java`; a command lands 2 to 4 ms after it is computed; power is
  clipped to 0..1 and truncated to steps of 1/32767. Voltage compensation sees the battery through a 100 ms lag by
  default; `extra.py` and `checks/check_ctrl.py team` also run the robot code's read every 50 loops.
- **Controllers:** 11, plus an oracle that sees the true speed at 5 kHz and marks the physics floor. `ffpi` is our
  loop (`architecture/control/PidController.java`: feedforward table plus PI, integral held while the output is
  pinned). `hub` approximates the hub's own velocity PIDF from secondary sources (10 ms loop, the motor's 28-count
  encoder, no voltage compensation); its units and anti-windup are unverified. Each controller's gains are tuned per
  inertia by grid search (`GRIDS` and `cost_of` in `run_all.py`).

### Run it

Run `tune` first: every later step reads the tuned gains from `flywheel/results.json`.

| step | command | time | writes, in `flywheel/` |
|---|---|---|---|
| plant sanity check | `python3 -I scripts/sim/flywheel/sanity.py` | 1 s | `sanity.json`; the table goes to the terminal |
| tune every controller | `python3 -I scripts/sim/flywheel/run_all.py tune` | 25 s | `results.json` |
| core sweeps | `python3 -I scripts/sim/flywheel/run_all.py core` | 35 s | `results.json` |
| robustness | `python3 -I scripts/sim/flywheel/run_all.py robust` | 4 min | `results.json` |
| hub PIDF at REV-guide gains | `python3 -I scripts/sim/flywheel/run_all.py hubdefault` | 3 s | `results.json` |
| loop period, window and timestamp error | `python3 -I scripts/sim/flywheel/run_all.py timing` | 4 min | `results.json` |
| battery sag, target RPM | `python3 -I scripts/sim/flywheel/extra.py` | 40 s | `results.json`, `results_extra.txt` |
| tables | `python3 -I scripts/sim/flywheel/tables.py` | 2 s | `results_tables.txt` |

`run_all.py` with no argument runs tune, core, robust, timing and hubdefault in that order. `results.json` grows to
about 5 MB. On 2026-11-10 every stage above, run from these files, reproduced the results the page quotes bit for
bit, and so did every check below at its default settings and with `FLY_KT=ke`.

### Checks: `flywheel/checks/`

Each prints its results. `check_ctrl.py`, `exp_rank.py` and `check_misc.py dt` read the tuned gains from
`results.json`, so run `tune` first.

| check | command | time |
|---|---|---|
| plant against closed-form answers; RPM noise and the effective window under timestamp error | `python3 -I scripts/sim/flywheel/checks/check_plant.py` | 30 s |
| is the overshoot from the integral or from P on a delayed measurement | `python3 -I scripts/sim/flywheel/checks/check_ctrl.py windup` | 10 s |
| our 50-loop voltage hold and 0.005 write cache | `python3 -I scripts/sim/flywheel/checks/check_ctrl.py team` | 1 min |
| each controller's own speed estimate against the windowed RPM | `python3 -I scripts/sim/flywheel/checks/check_ctrl.py est` | 35 s |
| Kalman process noise (rerun with `FLY_QW` at 1, 100 and 10000) | `FLY_QW=100 python3 -I scripts/sim/flywheel/checks/check_ctrl.py kfqw` | 50 s per value |
| I-zone against the same gains without one | `python3 -I scripts/sim/flywheel/checks/check_misc.py izone` | 20 s |
| oracle with a 60 g ball at 150 ms (rerun with `FLY_KT=ke`) | `python3 -I scripts/sim/flywheel/checks/check_misc.py oracle` | 15 s |
| plant step size (rerun with `FLY_DT=5e-5`) | `python3 -I scripts/sim/flywheel/checks/check_misc.py dt` | 10 s, about 4 times longer at `5e-5` |
| rankings on 16 new seeds | `python3 -I scripts/sim/flywheel/checks/exp_rank.py seeds301 base 301 16` | 50 s |
| rankings with 19% more torque, retuned | `FLY_KT=ke python3 -I scripts/sim/flywheel/checks/exp_rank.py ktke_retune retune 301 16` | 50 s |
| rankings with a cost dominated by shot accuracy, retuned | `python3 -I scripts/sim/flywheel/checks/exp_rank.py shotcost shot 301 16` | 50 s |

`exp_rank.py` also writes `checks/out_<tag>.json` (1.3 MB each) and takes its worker count from `NW` (default 8).
The environment overrides are read by `flysim.py` and default to the model above: `FLY_KT=ke` sets the torque
constant equal to the back-EMF constant (19% more torque), `FLY_QW` scales the Kalman filter's process noise on speed,
`FLY_DT` sets the plant step.

### Where the page's numbers come from

Section names are the headings `tables.py` and `extra.py` print.

| on the page | from |
|---|---|
| open-loop dip per 30 g ball at each inertia | `sanity.py` |
| fastest recovery to within 50 RPM by ball mass, and what a real loop adds | `tables.py`, SINGLE-SHOT RECOVERY (oracle row against the rest) |
| fastest recovery against target RPM | `extra.py`, TARGET RPM |
| exit-speed change for a ±5 g ball spread | no script prints it: (k(35 g) − k(25 g)) / (J + k(30 g)), with k from `ball_k()` in `flysim.py` |
| control laws within 5 to 15 RPM; rankings move with seed, torque constant and cost weights | `tables.py`, CORE and WINNERS; `checks/exp_rank.py`, the three runs above |
| RPM noise from timestamp error, and the effective window length | `checks/check_plant.py`, section 4 |
| first ball low under a 25 A drivetrain sag, by how often voltage is read | `extra.py`, VOLTAGE SENSING (the 750 ms hold is 50 loops) |
| integral without a zone: next ball 190 to 370 RPM fast; with one, within 12 | `checks/check_misc.py izone` |
| steady error with the table 10% off, P alone against P plus zoned I | `tables.py`, ROBUSTNESS, columns `ff-10` and `ff+10` |
| 120 RPM overshoot from a large kP on the lagged measurement | `checks/check_ctrl.py windup` |
| D had no measurable effect; bang-bang last in every ranking | `tables.py`, CORE, WINNERS and TIMING SWEEP; `checks/exp_rank.py` |
| hub velocity mode under the sag | `extra.py`, VOLTAGE SENSING, J = 1e-04 |
| boost pulse: best RPM at arrival, less consistent exit speed | `checks/exp_rank.py`, metric `exdev`; the paired per-seed difference was worked out from `out_seeds301.json` and no script prints it |

The spare-power table and the encoder resolutions on the page are arithmetic on the vendor spec, not outputs of these
scripts.

## Shot calculation: `shotcalc/`

### What it models

All simulated, in the vertical plane through the centre of the opening (a straight-on shot):

- **Target:** the CELL opening as `biobuzz/BioBuzzField.java` defines it (bottom edge 53.5 in up, 20 × 14 in
  pentagon, 7.61 in shoulders, face tilted 30°), aimed at the solver's aim point (the pentagon's centroid, 3 in
  inside, as in `biobuzz/ShotTuning.java`) from a 14 in exit height. A shot scores when the ball's centre first
  crosses the opening inward inside the outline less the ball's radius and 1 in, descending, without touching the
  floor first.
- **Ball flight, nine models:** vacuum; quadratic drag on a 2.8 in ball in 1.2 kg/m³ air (assumed) at 10 g with
  Cd 0.6 and 0.4, 25 g with 0.5, 45 g with 0.6 and 0.4; and drag plus constant backspin lift (lift coefficient 0.15
  or 0.30, no spin decay; unverified). RK4 at 2 ms.
- **Shooter (assumed):** exit speed = 0.09 in/s per RPM × (1 − 0.04 × hood) × RPM; launch angle 30 to 80° over the
  hood's 0 to 1 range, slightly bowed. Shot-to-shot noise of 1.5% of speed and 0.5° (one sigma); the combined score
  adds 40 RPM of flywheel error and 3 in of distance error. The robot's hood geometry is in `cad/shotcalc/`.
- **Studies:** 1, the family of scoring shots at each distance and policies for picking one; 2, table against curve
  against physics with a simulated tuner, with study 4 (lead time for moving shots) on the same calibrations; 3, hood
  compensation for flywheel sag; 4b, time-of-flight accuracy.

### The trajectory cache

`build_fields.py` flies every launch speed (150 to 650 in/s) at every angle (20 to 88° in 0.25° steps) for each ball
model and stores where each crosses the opening at every distance from 30 to 140 in:
`shotcalc/cache/field_<model>.npz`, 140 to 160 MB each, 1.4 GB for all nine. The studies add scoring probabilities
blurred by the shot noise, `cache/blur_*.npy`, 44 MB per model and noise level. After every study and check the
folder holds about 3.4 GB.

All of it regenerates. Delete it with `rm -rf scripts/sim/shotcalc/cache`, and do so after changing anything the
cache depends on (see [Known limits](#known-limits)).

### Run it

Study 1 first: `aimcheck.py`, `kdrag_equiv.py`, `report.py` and several checks read `results/study1.json`. A study
builds any field it is missing, so `build_fields.py` is only a quicker way to build all nine (4 processes). With no
arguments, `build_fields.py` and `study1.py` cover all nine models and the other studies the six in `MAIN_TRUTHS`.

| step | command | time | writes, in `shotcalc/` |
|---|---|---|---|
| all nine fields | `python3 -I scripts/sim/shotcalc/build_fields.py` | 15 to 20 s per field | `cache/` |
| study 1 | `python3 -I scripts/sim/shotcalc/study1.py` | about 20 s per model with its field built | `results/study1.json`, `.txt` |
| study 1, steeper entry rule (then again with 30) | `DESCENT_MIN_DEG=15 python3 -I scripts/sim/shotcalc/study1.py T1_vac T2_10g_cd.6 T2_25g_cd.5 T3_10g_cd.6_cl.30` | not timed | `results/study1_desc15.json`, `.txt` |
| studies 2 and 4 | `NREP=80 python3 -I scripts/sim/shotcalc/study2.py` | about 10 min | `results/study2_4.json` |
| studies 2 and 4, nonlinear shooter | `NREP=40 BETA=0.35 ETA_DROP=0.08 SPEED_CURVE=0.1 TAG=_nasty python3 -I scripts/sim/shotcalc/study2.py T1_vac T2_25g_cd.5 T2_10g_cd.6 T3_10g_cd.6_cl.30` | about 18 min | `results/study2_4_nasty.json` |
| study 3 | `python3 -I scripts/sim/shotcalc/study3.py` | about 1.5 min | `results/study3.json` |
| study 4b | `python3 -I scripts/sim/shotcalc/study4b.py` | not timed | `results/study4b.json` |
| aim check; kDragPerS equivalent to the true lead | `python3 -I scripts/sim/shotcalc/aimcheck.py`, then `kdrag_equiv.py` | seconds | `results/*.txt` |
| report | `python3 -I scripts/sim/shotcalc/report.py` | seconds | `results/RESULTS.txt` |

The study 2 and 3 times were taken on 2026-11-10 during the runs the page quotes. `results/` ends at about 5 MB.
On 2026-11-10 three fields (vacuum, 10 g drag, 10 g drag and lift), study 1 on the vacuum model and four checks, run
from these files, matched the page's runs bit for bit; the other studies were not rerun from them.

### Checks: `shotcalc/checks/`

Each writes its output into `checks/`, and builds any field it needs that isn't cached yet. A row with only a
script name runs as `python3 -I scripts/sim/shotcalc/checks/<script>`.

| check | command |
|---|---|
| numerical core: vacuum crossings against the closed form, RK4 step, energy, linear-drag closed form, field interpolation; 4 s with its three fields built | `python3 -I scripts/sim/shotcalc/checks/check_numerics.py` |
| study 2 with a new seed and four tuner models, then its summary | `SPACINGS=even4,even5,even6,even8,uneven4,uneven5,uneven6,uneven8 python3 -I scripts/sim/shotcalc/checks/study2_variants.py`, then `python3 -I scripts/sim/shotcalc/checks/summarise_variants.py` |
| the same, nonlinear shooter | `BETA=0.35 ETA_DROP=0.08 SPEED_CURVE=0.1 NREP=30 SEED=4711 MODES=orig,hoodfixed SPACINGS=even4,even5,even6,even8,uneven4,uneven8 TAG=_nasty python3 -I scripts/sim/shotcalc/checks/study2_variants.py T1_vac T2_25g_cd.5 T2_10g_cd.6 T3_10g_cd.6_cl.30`, then `summarise_variants.py scripts/sim/shotcalc/checks/study2_variants_nasty.json` |
| the same, physics solved through the solver's aim point | `AIMMODE=solver NREP=30 MODES=orig,hoodfixed SPACINGS=even4,even5,even8,uneven4 TAG=_solveraim` and the same script and models |
| study 3 part 2 with a calmer tuner | `python3 -I scripts/sim/shotcalc/checks/study3_variant.py` |
| how spread the simulated tuner's points are; is aiming at the band centre optimal | `python3 -I scripts/sim/shotcalc/checks/check_tuner_aim.py` |
| hood against sag, best-probability hood and centre of the window | `check_droop_hp.py`, `check_droop_curve.py` |
| largest sag the hood can cover, closed form (needs study 1) | `check_droop_sign.py` |
| kDragPerS picked from strafing tests | `check_kdrag_tuning.py` |
| additive against scaling trim (needs study 1) | `check_trim.py` |
| solver's entry check when tofS is timed to the opening (needs study 1) | `check_solver_entry.py` |

### Where the page's numbers come from

Section names are the headings `report.py` prints.

| on the page | from |
|---|---|
| the two best shot choices; lob, flat shot, constant hood and constant RPM 8 to 35 points lower | `study1.py`, STUDY 1 |
| table against curve against physics by number of distances, with the hood held to a schedule or re-picked; a bad point | `checks/study2_variants.py` and `summarise_variants.py` (`hoodfixed` and `hoodjit2` against `orig`) |
| physics with constants measured off the robot; 5% wear and one trim | `study2.py`, STUDY 2 |
| additive against scaling trim at 10% | `checks/check_trim.py` |
| sag the hood covers when the shot is flatter than minimum speed | `study3.py`, STUDY 3 hood capacity (`maxDef`), checked by `checks/check_droop_sign.py` |
| the correction changes size and direction with distance | `checks/check_droop_hp.py` |
| two droop points beat one; time-of-flight change of 22 to 63 ms; the 3-ball burst | `study3.py`, STUDY 3; `checks/study3_variant.py` |
| kDragPerS = 0 over-leads by 19, 44 and 125 ms | `study2.py`, STUDY 4 |
| kDragPerS tuned from strafing tests | `checks/check_kdrag_tuning.py` |
| time of flight from 240 fps video against a fitted model | `study4b.py`, STUDY 4b; `study2.py`, STUDY 4 |
| near-peak short shots refused as `RISING` | `checks/check_solver_entry.py` |
| numerical core within 0.03 in | `checks/check_numerics.py` |

The minimum-speed table, the speed and hood sensitivities, the speed window of the opening, the check at 0 to 60° off
the opening's normal, the sag the hood covers at a 45° or 50° descent, the time-of-flight change of up to 134 ms on a
steep shot and the 0.05 to 0.23% interpolation cost are closed-form projectile calculations from `ballistics/`; the
table under [Closed-form shot calculations](#closed-form-shot-calculations-ballistics) maps each to its script.

## Flywheel with the robot's numbers: `cad/flywheel/`

### What it models

All simulated. The plant is `flysim_cad.py`, built on `flywheel/flysim.py` with the same battery, PWM, friction law,
encoder windows and ball contact, but with every plant and ball value set per run. Robot numbers and their sources:

| input | value | source |
|---|---|---|
| motor | goBILDA 5000-0002-4008, one per shooter: 12 V; 6000 RPM theoretical and 5800 RPM tested free; 0.25 A free; 9.2 A stall. The back-EMF constant comes from the tested 5800 RPM (`FLY_FREE=6000` uses the other) | the CAD model as of 2026-11-10 for the part; vendor spec sheet, https://www.gobilda.com/content/spec_sheets/5000-0002-4008_spec-sheet.pdf (saved copy read 2026-11-10) |
| stall torque | 1.47 kg·cm, so the torque constant is 0.01567 N·m/A | the product page, https://www.gobilda.com/5000-series-12vdc-motor-with-8mm-rex-pinion-shaft/, as read on 2026-11-10 (not saved). The spec sheet prints "1.74 kg.cm (20.45 oz-in)"; 20.45 oz-in is 1.47 kg·cm. Which figure is the typo is not settled; 1.74 is close to what a torque constant equal to the back-EMF constant gives, and `sens_cad.py` and `checks/rerun.py ktke` run that case |
| drive | 1:1 belt, 72 mm goBILDA Rhino wheel (3614-0014-0072), r = 36 mm | the CAD model as of 2026-11-10; Nectar's 1:1 is assumed |
| inertia | design 0.78e-4 kg·m² (wheel 7.3e-5, hub, shaft, pulleys); motor rotor 5e-6 (3e-6 to 8e-6); Pollen's feed roller 1.96e-5. As built: Pollen 1.026e-4, Nectar 0.83e-4 | design: inferred from the CAD model as of 2026-11-10. Rotor: estimated (an RS-555-class armature); the vendor lists no rotor inertia. Roller: estimated, 0.6 m r² for an AndyMark 2 in Stealth wheel taken as 0.1 lb (https://www.andymark.com/products/2-in-stealth-wheel-8mm-bore-60a-durometer lists 0.03, 0.1, 0.17 and 0.3 lb without sizes; saved copy read 2026-11-10) plus 2e-6 for shaft and pulley. Nectar is modelled without a roller |
| inertia sweep and changes | 1.2, 1.6, 2.4 and 4.0e-4; goBILDA 82 mm steel flywheel 3628-0032-0082, 152 g, 1651 g·cm²; a 96 mm Rhino wheel (3614-0014-0096, 142 g, inertia 2.17e-4 scaled from the 72 mm wheel); gear ratios 0.8, 1.25 and 1.5 (wheel speed over motor speed); a second motor | steel flywheel: https://www.gobilda.com/steel-flywheel-82mm-diameter-152g-1651-g-cm/ as read on 2026-11-10 (not saved), its fit on the shooter unverified. 96 mm wheel: https://www.servocity.com/3614-series-rhino-wheel-14mm-bore-96mm-diameter-30a-durometer as read on 2026-11-10 (not saved) |
| balls | Pollen 24.9 g (23.5 to 27.2), radius 35.6 mm; Nectar 41.3 g (39 to 45), radius 45.7 mm; thin shells | sizes from the CAD model as of 2026-11-10; masses from a vendor listing not re-checked (unverified) |
| exit-speed fraction f | 0.35, 0.45, 0.50 of the wheel's surface speed at exit | assumed; a ball rolling on a fixed hood cannot pass 0.50 |
| contact time | 2L / v, with L the hood arc the ball centre travels: radius (wheel + ball − 1.1 or 1.3 mm squeeze), wrap 31° Pollen, 23° Nectar (54° and 32° with the side guides, a sensitivity case) | wrap and radii from the CAD model as of 2026-11-10; the model is assumed |
| shots | drag-free minimum-speed shot from the CAD exit point (Pollen 12.46 to 13.83 in up and 1.56 to 1.99 in ahead of the turret axis over hood 0 to 30°; Nectar 12.53 to 13.57 in and 1.23 to 1.60 in over 0 to 20°), or the hood's lower stop (Pollen 57.7°, Nectar 60.8°) when that is steeper; drag as × 1.00 or × 1.14 on the required speed | exit points and stops from the CAD model as of 2026-11-10 (exit points inferred); drag factor assumed. `ballistics.py` |
| battery and friction | 12.5 V behind 0.10 Ω; 0.008 N·m at 3000 RPM (0.010 with two motors) | assumed |
| loop | 15 ± 2 ms; our loop's command lands 8 to 12 ms after it is computed, the others' 1.5 to 3 ms | assumed; the 8 to 12 ms from the `follower.update` time in [`docs/loop-time-findings.md`](../../docs/loop-time-findings.md) |

Controllers, as the tables name them:

- `oracle` (ideal): sees the true speed at 5 kHz, full power below target, exact feedforward above.
- `team`: our loop, `biobuzz/modules/Shooter.java` with `architecture/control/PidController.java` (table
  feedforward plus P; integral held while pinned; no I-zone), voltage read every 50 loops
  (`VOLTAGE_READ_LOOP_INTERVAL`, `architecture/core/EnhancedOpMode.java:38` as of 2026-11-10, commit 366596c),
  applied volts capped at 13 V.
- `rec`: full power while more than 2% below target, otherwise feedforward plus P plus I inside 75 RPM; voltage
  read every loop; command sent right after it is computed.
- `rect`: `rec` with the full-power threshold tuned between 2 and 10%.

Gains are grid-searched per plant, controller and measurement at 84 in and f 0.45 (`GRID` and `tune_cost` in
`run_cad.py`). Measurement is `hub` (8192-count positions in a 50 ms window with ±0.5 ms timestamp error, the
`Shooter.measure()` rule at `biobuzz/modules/Shooter.java:333`) or `oq` (OctoQuad counts per 10, 20 or 50 ms latch,
read 6.6 ms after the latch; the units and int16 overflow are unverified).

A burst of 10 balls (3 and 5 are prefixes) passes at a feed interval when every ball of all 8 seeds stays inside
the window, and every longer interval passes too. Intervals are 92, 105, 120, 135, 150, 175, 200, 250 and 300 ms;
92 ms is the transfer's limit of 10.9 balls a second. Deviations are measured from the mean first-ball exit speed.
Two windows: the **reference window**, for a 2.8 in ball from a 14 in exit at the turret axis on the minimum-speed
line (−2.10/+6.00% at 36 in down to −1.55/+2.41% at 132 in, from `ballistics/windows_nodrag.py`), and each
shooter's **own window** (its exit point, ball size and hood stop, from `ballistics.py`).

### Run it

`run_cad.py tune` first: every later stage reads `results/tune.json`. Run `energy_cad.py` before `tables_cad.py`,
and `tables_cad.py` before `sens_cad.py` (which appends to its output), `budget_cad.py` and `headline_cad.py`;
`compact_cad.py` last. `run_cad.py` takes a worker count as a second argument (default 6), `sens_cad.py` as its first
(default 8).

| step | command | time | writes, in `cad/flywheel/` |
|---|---|---|---|
| plant and target check | `python3 -I scripts/sim/cad/flywheel/sanity_cad.py` | 3 s | `sanity_cad.json`; the table goes to the terminal |
| shot geometry, checked against `ballistics/` first | `python3 -I scripts/sim/cad/flywheel/ballistics.py` | 1 s | terminal only |
| tune | `python3 -I scripts/sim/cad/flywheel/run_cad.py tune` | 22 s | `results/tune.json` |
| one ball | `python3 -I scripts/sim/cad/flywheel/run_cad.py single` | 10 s | `results/single.json` (10 MB) |
| bursts, base plants | `python3 -I scripts/sim/cad/flywheel/run_cad.py burst` | 40 s | `results/burst.json` (165 MB) |
| OctoQuad at 10 and 50 ms | `python3 -I scripts/sim/cad/flywheel/run_cad.py oqsweep` | 13 s | `results/oqsweep.json` (52 MB) |
| changes | `python3 -I scripts/sim/cad/flywheel/run_cad.py changes` | 53 s | `results/changes.json` (220 MB) |
| ball-mass spread | `python3 -I scripts/sim/cad/flywheel/run_cad.py mass` | 6 s | `results/mass.json` |
| energy per ball and sustained rate | `python3 -I scripts/sim/cad/flywheel/energy_cad.py` | 40 s | `results/energy.json`; a table to the terminal |
| tables | `python3 -I scripts/sim/cad/flywheel/tables_cad.py` | 35 s | `results_tables.txt`, `results/summary.json` (45 MB) |
| sensitivity | `python3 -I scripts/sim/cad/flywheel/sens_cad.py` | 25 s | appends to `results_tables.txt`; `results/sens.json` (71 MB), `results/sens_summary.json` |
| ball-mass budget | `python3 -I scripts/sim/cad/flywheel/budget_cad.py` | 1 s | `budget.txt`, `results/budget.json` |
| headline | `python3 -I scripts/sim/cad/flywheel/headline_cad.py` | 5 s | `headline.txt` |
| compact results; hood-optimistic line | `python3 -I scripts/sim/cad/flywheel/compact_cad.py` | 1 s | `results_cad.json`; the hood line goes to the terminal |
| one run, traced | `python3 -I scripts/sim/cad/flywheel/trace_cad.py team hub 0.01 0 0.15` (controller, measurement, kP, kI, interval, then optional plant and shooter) | 1 s | terminal only |

`results/` ends at about 570 MB. On 2026-11-10 every stage above, run from these files, reproduced the earlier run
bit for bit except the geared plants, which moved with the rotor-inertia fix described below.

### Checks: `cad/flywheel/checks/`

Run the stages above first: the checks read `results/tune.json` and `results/summary.json`. Each prints its
results; `span_analysis.py`, `rerun.py` and `compare.py` also write into `checks/results/`. A row with only a
script name runs as `python3 -I scripts/sim/cad/flywheel/checks/<script>`.

| check | command | time |
|---|---|---|
| motor constants and the 1.47 against 1.74 kg·cm question; closed-form momentum and energy for one ball; an independent ODE of one shot; motor power; the reflected rotor inertia; net wheel torque against gear ratio | `check_physics.py` | 1 s |
| drag-free shots and own windows recomputed with `ballistics/cross.py` | `check_ballistics.py` | 1 s |
| the ideal loop's bursts from a semi-analytic model; the sustained interval from the post-ball speed | `check_burst_bound.py` | 25 s |
| every stored burst re-scored under four pass rules (below); 3-ball against 10-ball; the bias each needs | `span_analysis.py` | 2 s |
| reruns: original seeds, new seeds, f 0.40, torque constant equal to back-EMF constant | `rerun.py orig`, `rerun.py seeds`, `rerun.py f040`, `rerun.py ktke` | 10 s each |
| rerun with a 1e-4 s plant step | `FLY_DT=1e-4 python3 -I scripts/sim/cad/flywheel/checks/rerun.py dt` | 10 s |
| rerun with a heavy-tailed loop period (95% of loops 10 to 21 ms, 5% 25 to 45 ms) | `FLY_HEAVY=1 python3 -I scripts/sim/cad/flywheel/checks/rerun.py heavy` | 5 s |
| the reruns against `results/summary.json` (needs all six reruns) | `compare.py` | 1 s |

The pass rules in `span_analysis.py`, `rerun.py` and `compare.py`:

- `anc`: the study's own rule. Every ball's deviation from the first ball must sit inside the reference window, as
  if the RPM table were set so the first ball of a burst is exactly on the minimum-speed shot.
- `anc_own`: the same inside each shooter's own window.
- `bias`: one constant setpoint offset is allowed for all balls and seeds, as if the RPM table were calibrated while
  firing the intended burst. A cell passes when the spread of deviations fits inside the own window's width.
- `bias_mass`: `bias`, with the ball-mass spread added to the spread.

### Where the page's numbers come from

The CAD section of [`docs/flywheel-control-findings.md`](../../docs/flywheel-control-findings.md). Section names are
the headings the scripts print.

| on the page | from |
|---|---|
| fastest feed interval per shooter and distance: ideal, our loop on the hub port, best real loop | `headline.txt`, CAN IT FIRE WITH NO WAITING?; `results_tables.txt`, FASTEST FEED INTERVAL |
| the same at f 0.35 and 0.50, and with drag × 1.14 | the same sections, other rows |
| the same with the ball-mass spread taken out of the window | `budget.txt`, MASS BUDGET |
| 3- and 5-ball bursts against 10 | `results_tables.txt`, "3- and 5-ball bursts"; `checks/span_analysis.py`, the step distribution |
| required target RPM and its share of the 5533 RPM loaded free speed; exit speeds; hood on its stop and the speed it costs; the calibrated launch angles | `results_tables.txt`, INPUTS / DERIVED; `ballistics.py` |
| the 3.3° and 6.5° the hood would need below its stop at 132 in | `ballistics.py`, `launch` against `vminang` at 132 in |
| one ball: dip, time to stay within 1, 2 and 3%, the in-contact loss k/(J + k) | `results_tables.txt`, ONE BALL; `sanity_cad.py`; `checks/check_physics.py` for the pure k/(J + k) |
| per-ball exit deviation at 92 ms | `results_tables.txt`, "Per-ball exit deviation"; every per-ball array is in `results/summary.json`, the as-built, f 0.45, drag-free ones also in `results_cad.json` |
| the hood-optimistic line | `compact_cad.py` (terminal); `results_tables.txt`, FASTEST FEED INTERVAL columns `Vh` and `Ch` |
| energy per ball, net power, sustained rate at 12.0, 12.5 and 13.5 V, two motors, the 13 V cap, wheel energy over ball energy | `energy_cad.py` (terminal); `results_tables.txt`, ENERGY |
| the sustained interval from the post-ball speed (60/73/92/116/153 ms Pollen as built) | `checks/check_burst_bound.py`, last section |
| ball-mass spread per plant | `results_tables.txt`, BALL-MASS SPREAD |
| hub port against OctoQuad: interval, measured noise, true ripple, error at touch, the int16 limit | `results_tables.txt`, MEASUREMENT |
| steel flywheel, more inertia, gearing, a second motor, a 96 mm wheel | `results_tables.txt`, WHAT CHANGES THE ANSWER; `headline.txt`, CHEAPEST CHANGE and PER CHANGE |
| torque constant, friction, battery voltage and wrap | `results_tables.txt`, SENSITIVITY |
| numbers under a burst-calibrated setpoint and in the own windows; the bias needed; the mass spread with a bias | `checks/span_analysis.py` |
| every real loop and measurement within one interval step under a burst-calibrated setpoint | `checks/span_analysis.py`, "rule bias by loop and measurement" |
| new seeds; f 0.40; torque constant equal to back-EMF constant under each rule; 1e-4 s step; heavy-tailed loop period | `checks/rerun.py`, then `checks/compare.py` |
| net wheel torque against gear ratio (2.2 times at Nectar 132 in) | `checks/check_physics.py`, last section |
| f capped at 0.50; what a measured exit speed over touch speed of 0.45 means for f | `checks/check_physics.py`, section 6 |

The friction comparison with the motor's no-load current, the hood servo's travel and the steel flywheel's fit are
arithmetic or reading, not outputs of these scripts.

### The rotor-inertia fix on 2026-11-10

`plants()` in `run_cad.py` used to reflect the motor rotor onto the wheel as J_rotor × g², with g the wheel speed over
the motor speed. The motor turns at the wheel speed over g, so the reflected inertia is J_rotor / g²; the code now
uses that. Only the geared plants changed: Pollen g1.5 J went from 1.09e-4 to 1.00e-4, g1.25 from 1.05e-4 to 1.01e-4,
g0.8 from 1.01e-4 to 1.05e-4; Nectar 0.89, 0.86 and 0.81e-4 became 0.80, 0.81 and 0.86e-4. Their gains were retuned.
Every other tuned gain and every other row of `results_tables.txt`, `headline.txt` and `budget.txt` came out bit for
bit the same, except the overall wheel-energy-over-ball-energy range in ENERGY (2.06 to 3.13 became 2.06 to 3.15).
Numbers an earlier write-up quoted that moved (best real loop, f 0.45, drag-free, 36/60/84/108/132 in):

| plant | before | now |
|---|---|---|
| Pollen g1.25 | 92/105/105/120/135 | 92/105/120/120/150 |
| Pollen g1.5 | 92/105/105/120/150 | 92/92/105/120/150 |
| Nectar g1.25 | 120/135/175/200/250 | 120/135/175/200/300 |
| Nectar g1.5 | 135/150/175/200/250 | 135/150/175/250/250 |
| Nectar g0.8, ideal loop | 135/175/300/infeasible/infeasible | 120/175/250/infeasible/infeasible |
| target as a share of free speed: Pollen g0.8 at 132 in; Nectar g1.25 and g1.5 at 132 in | 97%; 77% and 69% | 96%; 78% and 70% |

Pollen g0.8 (92/92/120/175/>300) and Pollen g1.25 with the steel flywheel (92/92/92/105/120) did not change, and
neither did any CHEAPEST CHANGE cell. The ball-mass spread of the geared plants is now 0.03 to 0.11 points wider
than as built instead of slightly narrower (Pollen g1.5 at 84 in, f 0.45: −0.99/+0.61% against −0.96/+0.59%).

## Shot calculation with the robot's geometry: `cad/shotcalc/`

### What it models

All simulated, in the vertical plane through the turret axis and the aim point (a straight-on shot, no sideways exit
offset). The target, aim point and entry rule are the generic study's, read from the code as of 2026-11-10 (commit
366596c): `OPENING_*` and `CELL_TILT_RAD` in `biobuzz/BioBuzzField.java:64-70`, `aimDepthIn = 3` and
`rimClearanceIn = 1` (marked "TODO: tune") in `biobuzz/ShotTuning.java:98-100`, `GRAVITY_IN_PER_S2` in
`biobuzz/ShotSolver.java:19` and the margin rule at `ShotSolver.java:373`.

| input | value | source |
|---|---|---|
| launch angle | base + m, m the hood angle: Pollen 57.7° + 0 to 30°, Nectar 60.8° + 0 to 20° (the `geo` base); a second, `cal` base of 55.9° and 57.4° | the CAD model as of 2026-11-10; the `cal` base is inferred. The Nectar hood was reshaped after these numbers and they were not re-derived |
| exit point (ball centre) | linear in m: Pollen 1.56 in ahead and 12.46 in up at m 0, 1.99 and 13.83 in at m 30; Nectar 1.23/12.53 in at 0, 1.60/13.57 in at 20; extended linearly outside the travel | inferred from the CAD model as of 2026-11-10 |
| balls | Pollen 2.8 in, 24.9 g (23.5 to 27.2); Nectar 3.6 in, 41.3 g (39 to 45) | sizes from the CAD model; masses unverified |
| drag | `none`; `low`, heaviest ball and Cd 0.4; `nom`, nominal ball and Cd 0.5; `high`, lightest ball and Cd 0.6; air 1.204 kg/m³ | assumed |
| spin lift | lift coefficient −0.30 and −0.15 (topspin) and +0.15 (backspin), nominal drag, `geo` base | assumed |
| flywheel | 72 mm wheel at 1:1, 0.14842 in/s of surface speed per RPM; exit speed = eta × surface speed, eta 0.35, 0.45 or 0.50; free speed 5400 or 5800 RPM | wheel from the CAD model; eta assumed; free speeds as given to the study |
| hood speed | 78°/s with no load: 0.09 s per 60° at 6 V through 8.571:1 | vendor, https://www.gobilda.com/2000-series-dual-mode-servo-25-3-speed/, as read on 2026-11-10 (not saved); slower at 4.8 V and under load |
| shot picks | robust: the widest speed window among shots whose hood window is at least ±2° and distance window at least ±3 in; best speed and robust picks also need an exit speed the flywheel can reach (`V_REACH`: 90% of 5400 RPM at eta 0.45, 324.6 in/s) and hood and distance windows open on both sides | assumed |
| sag model (`sag_energy.py`) | per-ball sag = (1 + k)/2 · m · eta · r² / I, I = 0.78e-4 (also 1e-4, 1.5e-4, 3e-4); stall 1.47 kg·cm at 12 V; 10.9 balls a second | inertia inferred from the CAD model as of 2026-11-10; motor from https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/; model order-of-magnitude |

Schedules in the tables: `i` is the best-tolerance schedule (the robust pick at each distance, made non-increasing in
m and rounded to 0.5°); `floor` is the lowest-speed angle inside the travel; `k3`, `k5` and `k8` sit 3, 5 and 8°
steeper than `floor` to leave room for the hood to cover sag.

### Run it

| step | command | time | writes, in `cad/shotcalc/` |
|---|---|---|---|
| reproduce the 14 in exit numbers of `ballistics/` first | `python3 -I scripts/sim/cad/shotcalc/validate.py` | 15 s | `out/validate.json`; the table goes to the terminal |
| all 22 cases (2 shooters × 2 bases × 4 drag cases, plus 6 spin cases) | `python3 -I scripts/sim/cad/shotcalc/study.py` | about 15 min: 5 s per drag-free case, 35 to 60 s per other case | `out/raw_<case>.npz`, 0.2 MB each |
| one case | `python3 -I scripts/sim/cad/shotcalc/study.py pollen geo nom` (then an optional lift coefficient, e.g. `-0.30`) | 40 s | `out/raw_pollen_geo_nom.npz` |
| report | `python3 -I scripts/sim/cad/shotcalc/report.py` | 7 s | `out/results.json` (2.6 MB), `out/tables.txt`, also printed |
| sag per ball and sustained rate | `python3 -I scripts/sim/cad/shotcalc/sag_energy.py` | 1 s | `out/sag_energy.json`; the tables go to the terminal |

`study.py` skips any case whose `out/raw_<case>.npz` exists: delete `out/` after changing anything in
`cadcore.py` or `windows.py`. `report.py` reports whichever cases exist; `tables.txt` prints the `nom` cases of each
shooter and base in full and every case in its CROSS-CASE SUMMARY. On 2026-11-10 `checks/rerun_case.py` reproduced
two cases bit for bit, and `report.py` on the stored cases reproduced the earlier report except for the fix below.

### Checks: `cad/shotcalc/checks/`

Each prints its results. A row with only a script name runs as `python3 -I scripts/sim/cad/shotcalc/checks/<script>`.

| check | command | time | needs |
|---|---|---|---|
| drag-free closed form against the stored drag-free cases | `closedform_check.py` | 1 s | the four `none` cases |
| the CAD exit point against a 14 in exit at the axis, on schedule `i` | `exit_sched.py` | 1 s | nothing |
| the same across the travel; interpolation against half the low side of the window; a 3-node hood at 60 in | `exp_checks2.py` | 1 s | `pollen` and `nectar` `geo nom` |
| robust-pick thresholds; which side of the distance window binds on schedule `i` | `exp_robust.py` | 1 s | `geo nom` for both |
| an independent solver (scipy DOP853), its own window edges | `indep.py` | 2 s | nothing; writes `checks/indep_out.json` |
| topspin past −0.30; the Nectar base angle and exit point moved | `exp_misc.py` | 45 s | nothing |
| rim clearance 0, 1 and 2 in; the rising rule off; the hood-floor benefit at each | `exp_rim.py` | 50 s | nothing; writes `checks/exp_rim.json` |
| the steep backspin shots the old best-speed pick chose | `probe_backspin.py` | 1 s | nothing |
| rerun two cases and compare bit for bit | `rerun_case.py` | 75 s | `pollen geo nom` and `nectar cal high`; writes `checks/rerun_out/` (delete it to rerun) |
| the sag model's bookkeeping, three more contact models, what the encoder sees after a ball, alternating shooters | `sag_check.py` | 1 s | `out/sag_energy.json` |

`exp_rim.py` reloads `cadcore.py` with the environment variable `RIM_CLEARANCE_IN`; set it yourself to run any step
with another rim clearance (and delete `out/` first).

### Where the page's numbers come from

The CAD section of [`docs/shot-calculation-findings.md`](../../docs/shot-calculation-findings.md). Section names are
the headings `report.py` prints into `out/tables.txt`; rows of other drag and spin cases are in `out/results.json`.

| on the page | from |
|---|---|
| minimum-speed angle and speed per distance; where it is inside the travel; the scoring range of m; the rising limit at 36 and 48 in | SCORING RANGE |
| steepest scoring m within 90% of free speed | STEEPEST REACHABLE m |
| the schedule to tune: m, launch angle, speed, RPM, time of flight, descent, speed, hood and distance windows | SCHEDULES (row `i`) and WINDOWS (row `robust`) |
| the most tolerant shots by speed width and by distance width | WINDOWS, rows `bestSpd` and `bestDst` |
| sag the hood covers on schedule `i`, at k = 3, 5 and 8°, and at literal m = 3, 5 and 8 | SAG COVER; SCHEDULES rows `k3`, `k5`, `k8`; SAG COVER AT LITERAL m |
| what leaving room costs against schedule `i` (width, RPM, time of flight) | SCHEDULES, the `width%`, `rpm@.45` and `tof ms` lines of rows `i` and `k3`, `k5`, `k8`; the differences are worked out by hand, no script prints them |
| hood offset needed to cover 5, 10 and 15% sag | HOOD OFFSET FOR A GIVEN SAG |
| reach at 70, 80 and 90% of free speed | RPM AND REACH |
| RPM against distance near a straight line; nodes needed for interpolation | SCHEDULES, `linearity` and `interp` lines; `checks/exp_checks2.py` for the stricter test and the 3-node hood |
| hood travel: width and time of flight at 132 in with a lower floor | HOOD FLOOR SWEEP; HOOD FLOOR |
| time of flight on each schedule and its change when the hood covers sag | TIME OF FLIGHT |
| window spread across drag cases and bases | CROSS-CASE SUMMARY; `out/results.json` |
| per-ball sag, sustained rate, spread over a 5-ball burst | `sag_energy.py` |
| other contact models, what the encoder reads after a ball, alternating the shooters | `checks/sag_check.py` |
| rim clearance 0 to 2 in; the rising rule off; the floor benefit at each rim | `checks/exp_rim.py` |
| robust thresholds; near or far side of the distance window | `checks/exp_robust.py` |
| topspin at −0.45 and −0.60; the Nectar base moved | `checks/exp_misc.py` |
| the CAD exit point's effect on the needed speed | `checks/exit_sched.py`, `checks/exp_checks2.py` part 1 |
| the engine against the closed form and an independent solver | `validate.py`, `checks/closedform_check.py`, `checks/indep.py` |

The comparisons with `rpmTolerance = 75` and `exitHeightIn = 14` (`ShotTuning.java:73` and `:51`) and the hood servo
arithmetic are worked out by hand from these tables.

### The shot-pick fix on 2026-11-10

`report.py` used to pick the best-speed shot from every scoring shot in the travel, and fell back to that shot when no
shot passed the robust filter. In the backspin case (+0.15) it chose shots at 79 to 84° that need 353 to 626 in/s and
have hood and distance windows closed on one side. Now both picks need a reachable speed and windows open on both
sides, and the robust pick is reported as `none` when nothing passes; schedule `i` then holds the last distance's m.
What moved:

- Nectar, `geo` base, 132 in, every drag case: the robust pick is `none` (it was the m 0 shot, 3.19% wide at nominal
  drag, whose hood window is only +1.86° on the high side). Schedule `i` still holds m 0 there and its row still
  shows 3.19%; the HOOD FLOOR line's "CAD best width" at 132 in now prints `-`.
- Backspin +0.15, Pollen: best-speed picks went from m 21.5 to 26.5 (353 to 626 in/s) to the normal m 0 to 13.5 (203
  to 289 in/s), and their sag cover from 43 to 60% to 0 to 1.2%. Nectar at 120 and 132 in: best speed from m 18.5
  and 18 to m 0; robust pick `none`.
- Nothing else: no schedule row, width, RPM, time of flight, reach, interpolation or CROSS-CASE SUMMARY number
  changed.

## Closed-form shot calculations: `ballistics/`

### What it models

A 2.8 in ball from a 14 in exit at the turret axis (`exitHeightIn = 14`, `ShotTuning.java:51`) into the CELL opening
as `BioBuzzField.java` defines it, aimed at the solver's aim point, scored by the `ShotSolver.checkEntry` rule with
1 in rim clearance; the constants are in `geom.py`. Drag-free and exact, except `run_drag.py`, which repeats the key
results with quadratic drag (air 1.204 kg/m³; 40 g with Cd 0.45, 25 g with 0.47, 15 g with 0.50; all assumed) by RK4
at 1 ms. Shot families: the minimum-speed line (`vmin`) and fixed descent angles of 30, 45 and 55° (`b30`, `b45`,
`b55`). The turret may sit 0, 20, 40 or 60° off the opening's normal (`phi`). `geom.py`, `cross.py` and `drag2.py` are
libraries; `cad/flywheel/checks/check_ballistics.py` also uses `geom.py` and `cross.py`.

### Run it

Each prints its results.

| step | command | time |
|---|---|---|
| minimum-speed line; hood-absorbable speed loss at a fixed descent; vertical shift per 1% of speed | `python3 -I scripts/sim/ballistics/nodrag.py` | 1 s |
| speed, hood and yaw windows by family, distance and phi; rim 0; hood offset on the minimum-speed line; time of flight when the hood covers a speed loss | `python3 -I scripts/sim/ballistics/windows_nodrag.py` | 5 s |
| when the launch point is behind the opening's plane; the binding edge; aim depth | `python3 -I scripts/sim/ballistics/b8_extra.py` | 1 s |
| interpolation error with 3, 4 and 5 calibration distances | `python3 -I scripts/sim/ballistics/interp.py` | 1 s |
| the key results with quadratic drag | `python3 -I scripts/sim/ballistics/run_drag.py` (or name cases: `run_drag.py nodrag "40g Cd0.45"`) | about 1 min per case |

On 2026-11-10 `windows_nodrag.py`, `b8_extra.py`, `interp.py` and `run_drag.py` reproduced the outputs the pages were
written from, line for line apart from headings.

### Where the page's numbers come from

| on the page | from |
|---|---|
| the minimum-speed table | `nodrag.py`, "min-speed line" |
| speed and hood sensitivities | `nodrag.py`, "vertical shift"; `windows_nodrag.py`, the min-speed-line hood offsets |
| the speed window of the opening | `windows_nodrag.py`, family `vmin`, phi 0 |
| 0 to 60° off the opening's normal | `windows_nodrag.py`, phi 20 to 60; `b8_extra.py` |
| sag the hood covers at a 45° or 50° descent | `nodrag.py`, "hood-absorbable speed loss"; `run_drag.py` for drag |
| time-of-flight change of up to 134 ms on a steep shot | `windows_nodrag.py`, "TOF at the aim point when the hood compensates" (55° descent, 4% at 84 in) |
| 0.05 to 0.23% interpolation cost | `interp.py`, 4 and 5 distances |
| the reference windows and speeds that `cad/` checks against | `windows_nodrag.py` and `nodrag.py`; the 40 g drag speeds from `run_drag.py` |

## Knobs for measured values

Line numbers are as of 2026-11-10.

### Flywheel

| quantity | where | notes |
|---|---|---|
| flywheel inertia J, kg·m², motor rotor included | `JS` in `run_all.py:10` and `tables.py:10`; `SPIN_T` and `REC_T`, `run_all.py:11-12` | The two `JS` lists must match, and every J needs a `SPIN_T` and `REC_T` entry, long enough for every controller to settle: today's are 3 to 9 times the full-power spin-up time `sanity.py` prints. The timing stage (`run_all.py:239`), `extra.py` (four loops, lines 28 to 58), `sanity.py:35,58` and the checks in `checks/` list their own J values. |
| ball mass, kg, and contact time, s | `m` and `Tc` in `BASE`, `run_all.py:13` (tuning, robust, timing, extra); the core sweep at `run_all.py:206`, `tables.py:72` and `checks/exp_rank.py:12`; `sanity.py:60` | The core sweep's lists in `run_all.py` and `tables.py` must match. |
| loop period, RPM window, timestamp error, s | `T`, `W` and `tsA` in `BASE`, `run_all.py:13`; the timing sweep at `run_all.py:238` and `:250` | `tsA` is the half-width of a uniform error. Loop jitter (±2 ms) is at `flysim.py:162`, command delay (2 to 4 ms) at `flysim.py:166`. |
| battery and friction | `Voc`, `Rint` and `fric` (N·m at 3000 RPM) in `BASE`, `run_all.py:13` | |
| wheel radius and exit fraction | `WHEEL_R`, `EXIT_FRAC`, `SHELL`, `flysim.py:28-30` | A 72 mm wheel acts like a ball 0.5625 times as heavy. |
| motor | `flysim.py:16-17` | |
| target RPM | `run_all.py:65` and `:73`; `extra.py:37` sweeps 2500 to 4500 | |
| how voltage compensation sees the battery | scenario key `vs_tau` (first-order lag, default 0.1 s, `flysim.py:136`) or `vs_hold` (sample-hold) | `extra.py` compares them. |

### Shot

| quantity | where | notes |
|---|---|---|
| ball mass, drag and lift | `TRUTHS` and `MAIN_TRUTHS`, `shotcore.py:69-80`; air density `RHO`, `:58`; ball diameter `BALL_D`, `:19` | Add a measured ball as a new name, and add the name to `short()` in `report.py:21` or the report fails. Changing an existing entry needs its cache files deleted. |
| exit speed per RPM and the hood map | `K_GAIN`, `ETA_DROP`, `TH_LO`, `TH_HI`, `BETA`, `SPEED_CURVE`, `shotcore.py:280-284`; the last three are also read from the environment | These don't touch the cache, but exit speeds must stay inside its 150 to 650 in/s. |
| exit height, aim depth, rim clearance, opening | `shotcore.py:14-20` | Delete the cache after changing any of them. |
| shot noise | `SIG_V`, `SIG_TH`, `evalcore.py:11-12`; flywheel and distance error, `study1.py:22-23` | Each noise level gets its own blur file. |
| burst model: 300 RPM lost per ball, recovery time, 120 ms feed | `report.py:372-378` | |
| video frame time for time of flight | `FRAME`, `study2.py:21`, `study4b.py:16`, `checks/check_kdrag_tuning.py:16` | |
| calibration distances | `SPACINGS`, `tuner.py:16` | |

### Flywheel with the robot's numbers (`cad/flywheel/`)

Rerun from `run_cad.py tune` after changing any of these.

| quantity | where | notes |
|---|---|---|
| exit-speed fraction f | `F_EXIT`, `cadcfg.py:32` | Gains are tuned at f 0.45 only (`run_cad.py:164`). A measured exit speed over the encoder's pre-touch speed is not f: divide it by the in-contact ratio (`checks/check_physics.py` section 6). |
| inertia | `J_DESIGN`, `J_ROTOR`, `J_ROLLER` and `STEALTH_2IN_MASS`, `cadcfg.py:11-16`; the sweep and options `J_STEEL82`, `J_SWEEP`, `J_WHEEL96`, `:17-24` | Plants are built from these in `plants()`, `run_cad.py:32-46`. A spin-down at known current gives the as-built J directly. |
| friction, N·m at 3000 RPM | `fric` in `plants()`, `run_cad.py:35` (0.010 with two motors, `:40-41`); the default of `rho_ff`, `cadcfg.py:63` | `sens_cad.py` runs 0.004 and 0.016 (`VARIANTS`, `sens_cad.py:13-21`). |
| torque constant | `T_STALL` (1.47 kg·cm), `flysim_cad.py:34`; free speed for the back-EMF constant, `FLY_FREE` (default 5800), `:33` | `sens_cad.py` variant `kt=ke` and `checks/rerun.py ktke` run the high case. |
| battery | `Voc` in `row()`, `run_cad.py:66`; the free speed used for feasibility in `tgt_info()`, `:60` | `sens_cad.py` runs 12.0 and 13.5 V. |
| ball masses | `BALL`, `cadcfg.py:26-27` | The `mass` stage fires the lightest, nominal and heaviest ball. |
| contact geometry | `HOOD` wrap and squeeze, `cadcfg.py:28-29`; `contact_time()`, `:50` | |
| hood base angle, travel and exit point | `SHOOTERS`, `ballistics.py:29-33` | |
| rim clearance and aim depth | `AIM_DEPTH, RIM`, `ballistics.py:13` | The reference windows `REF_WIN` and `REF_V` (`:144-145`) are for a 2.8 in ball at 14 in, and `validate()` asserts them. |
| loop period, window, timestamp error, latency | `T`, `W`, `tsA` in `row()`, `run_cad.py:73`; `LAT`, `cadcfg.py:35` | |
| feed intervals and seeds | `INTERVALS`, `SEEDS`, `TUNE_SEEDS`, `run_cad.py:16-20` | |

### Shot with the robot's geometry (`cad/shotcalc/`)

Delete `cad/shotcalc/out/` and rerun `study.py` after changing anything in `cadcore.py`; the `report.py` values only
need `report.py` again.

| quantity | where | notes |
|---|---|---|
| exit-speed fraction eta and free speed | `ETAS`, `FREES`, `report.py:10-11`; `V_REACH`, `:14` | RPM and reach only; the stored cases don't depend on them. |
| ball masses and diameters | `SHOOTERS`, `cadcore.py:40-45` | |
| drag cases and air density | `drag_consts()`, `cadcore.py:54-62`; `RHO`, `:36` | |
| hood base angle, travel, exit point | `SHOOTERS` `base`, `mmax`, `exit`, `cadcore.py:40-45` | |
| rim clearance | `RIM`, `cadcore.py:19`, or the environment variable `RIM_CLEARANCE_IN` | Every absolute window depends on it: 0, 1 and 2 in change the widths by about ±25% (`checks/exp_rim.py`). |
| aim depth and opening | `cadcore.py:14-18` | |
| lift coefficients | the spin cases in `study.py:49` | |
| robust-pick thresholds | `HOOD_MIN_SIDE`, `DIST_MIN_SIDE`, `report.py:13` | |
| hood speed | `HOOD_RATE`, `report.py:9` | |
| sag model: wheel radius, stall torque, inertia, ball masses, fire rate | `R_W`, `TAU_S`, `I_CAD`, `BALLS`, `sag_energy.py:13-17`; `RATE`, `:53` | |

### Closed-form calculations (`ballistics/`)

| quantity | where |
|---|---|
| exit height, aim depth, rim clearance, opening, ball diameter | `geom.py:4-13` |
| drag cases and air density | `CASES`, `drag2.py:117`; `RHO`, `:10` |

## Known limits

- **The shot cache is keyed only by model name and noise level.** It isn't rebuilt when a model's mass, Cd or lift
  changes, or after a change to `RHO`, `BALL_D`, `EXIT_Z`, the opening, `RIM_CLEAR`, the descent rule, the
  integration step or the grids. Delete `shotcalc/cache/`, or that model's files, after any of those. The same holds
  for `cad/shotcalc/out/`.
- **Flywheel:** one brushed motor with no inductance and average-voltage PWM; timestamp error is the only measurement
  noise; the ball contact is derived, not measured; the hub PIDF model rests on secondary sources. The vendor spec
  leaves the torque constant uncertain by about 19%: `FLY_KT=ke` reruns any step at the high end.
- **Flywheel with the robot's numbers:** one shooter on its own 12.5 V battery: both shooters and the drivetrain
  sharing the battery are not simulated. The feed roller's share of the ball's momentum is taken from the flywheel,
  which is conservative. The pass test has no budget for aim, turret, drag-model or interpolation error, and the
  answer depends on how the RPM table is calibrated: `anc` and `bias` differ by 1 to 3 interval steps. Gains are
  tuned at one operating point on 6 seeds; new seeds move about one cell in eleven by a step. The hood-optimistic
  line is not physically available on the minimum-speed line or where the hood sits on its stop. OctoQuad units and
  latch timing are unverified.
- **Shot:** straight-on flight only, with no sideways offset or turret error; the RPM and hood map is assumed; the
  scoring band uses a fixed 2.4 in margin where `ShotSolver` widens it with the angle of incidence, so absolute
  probabilities run slightly high; bounce-outs aren't modelled; the burst model in `report.py` is illustrative.
- **Shot with the robot's geometry:** straight-on only; the exit point is linear in hood angle and extended outside
  the travel; eta is the same at every RPM and hood angle; the rising-entry limit is the solver's rule, not a measured
  miss; the sag model assumes an ideal motor at full power with no battery sag or controller lag.
- **The tuner in `shotcalc/study2.py`** re-picks the hood at every distance, which about doubles the gap between a
  linear table and a smooth curve. `shotcalc/checks/study2_variants.py` also runs a tuner who holds the hood to a
  schedule; the page quotes both.
- **Monte Carlo noise:** differences under about 0.01 in mean scoring probability, or 2 to 5 RPM in the flywheel
  means, are within it; in `cad/flywheel/` one interval step is within it.
- **Memory:** each shot-study worker holds one model's field and blur arrays, about 0.3 to 0.4 GB (computed from the
  array sizes), and `study3.py` runs 8 workers. `cad/flywheel/results/` reaches about 570 MB and `tables_cad.py`
  loads most of it at once.
