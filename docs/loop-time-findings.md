# Loop-time findings

What costs loop time on the Betta bot and in the SDK, and what to change; read it before shortening the loop or adding
per-loop hub commands. Hub command costs come from [`hub-bench-findings.md`](hub-bench-findings.md); re-measure with
Ivy Path Test's log (`scripts/looptimes/analyze_ivy.py`) after any hardware or layout change.

## What to do

- **Wire the BIOBUZZ robot as in [`wiring-findings.md`](wiring-findings.md)**: nothing that is read goes on the
  Expansion Hub, so each loop pays one bulk read, and outputs that change every loop go on the Control Hub.
- **OctoQuad for odometry**: 3.2 ms against the Pinpoint's 5.8 ms every loop, with far less heading drift. Needs the
  guarded localizer [`hub-bench-findings.md`](hub-bench-findings.md) describes (a missing OctoQuad hangs INIT).
- **Write tolerances** (other teams' are [below](#other-teams-decode-code-2026-10-25)): `Shooter`'s flywheels already
  use `withCachingTolerance(0.005)`; try 0.002–0.005 on its hoods (0.001 now) and keep 0.001 on the turret. `Intran`'s
  intake and transfer switch between fixed powers, so they already write only on a change; give them 0.02 if they
  ever get a computed power.
- **Fewer per-loop allocations**, to cut collector pauses.
- **A rangefinder poller thread** cost about 3 ms per loop on a 14 ms driving loop, and will cost more on a longer
  one: use one only to read faster than the loop.
- **Count about 1.4 ms of SDK time in every `EnhancedOpMode` loop** ([below](#what-the-sdk-costs-every-loop)).
- **Don't lower `WriteCache`'s re-send period**: the SDK lets only every second re-send through, so a lost write is
  repaired in about 0.5 s either way ([below](#how-often-a-write-re-send-reaches-the-hub)).
- **Photon**, which sends writes without waiting for each ACK, cut the time spent writing 2.3x on the bench, but stays
  off the robot until its bench block says it is safe ([below](#photon)).

## Ivy Path Test log (2026-10-25, Betta bot, Pinpoint odometry)

Run 2 of `/sdcard/FIRST/looptimes/IvyPathTest.csv`: 1280 loops over 19.6 s after START. Run 1 (5.5 s, about 3 minutes
after boot) averaged 23 ms with one 130 ms loop: the code was still warming up, so judge only warmed-up runs.

| | mean | p50 | p90 | p99 | max |
|---|---|---|---|---|---|
| whole loop (ms) | 15.3 | 15.6 | 22.0 | 34.7 | 51.6 |
| driving a path | 19.4 | | 24 | | |
| paused | 12.8 | | 19 | | |

67 loops (5%) took over 25 ms and 3 over 40 ms. INIT loops averaged 10.7 ms: the follower already runs there.

| section | mean ms | max ms |
|---|---|---|
| `follower.update` | 9.9 (64%) | 46.3 |
| `updateTelemetry` | 2.7 | 16.6 |
| `writeModules` | 0.6 | 17.4 |
| everything else | ≈0.9 | |
| unprofiled: the SDK between loops ([below](#what-the-sdk-costs-every-loop)) | 1.4 | |

- **Pedro is the loop.** `follower.update` costs 7.7 ms with the robot paused, mostly the Pinpoint read (5.8 ms on the
  bench), and 13.4 ms while driving, adding path math and four drive writes, two of them on the Expansion Hub.
- **`write.BettaShooter` spikes to 5–17 ms every ~0.4 s** and is 0.03 ms otherwise: the SDK re-reading each motor's
  mode and re-sending its power and enable, not our code ([below](#how-often-a-write-re-send-reaches-the-hub)).
- **`updateTelemetry` passes 4 ms about 6 times a second**: the DS frame and its field map.
- **Garbage collection:** 17 collections took 1.1 s of 19.6 s. Some 20–46 ms `follower.update` spikes match nothing
  else and are probably collector pauses; the log can't prove it.

## What the SDK costs every loop

Every `EnhancedOpMode` loop spends about 1.4 ms in the SDK between `loop()` calls, the Ivy log's unprofiled 1.4 ms:
count it in every loop budget. A `LinearOpMode` loop doesn't pay it, so driving the framework loop from one should save
about 1.2 ms; not tried.

- After each `init_loop()` or `loop()` call, SDK 12.0.0's `OpMode.internalRunOpMode` calls `telemetry.update()` and
  then `Thread.sleep(1)`, and it copies both gamepads before each call (read from source, RobotCore 12.0.0
  `OpMode.java`). `EnhancedOpMode` extends `OpMode` (`architecture/core/EnhancedOpMode.java:35`), so every framework
  loop pays it.
- Measured on the bench Control Hub (setup in [`hub-bench-findings.md`](hub-bench-findings.md)) on 2026-10-09, and
  again in a 2026-10-24 rerun of the same bench:

  | loop | p50 | p95 |
  |---|---|---|
  | iterative `OpMode` whose `loop()` does nothing | 1.42 ms | 1.68–1.69 ms |
  | `LinearOpMode` running Pedro's follower (software localizer), Pedro's Ivy command scheduler and `telemetry.update()` (Bench: Raw Loop, idle) | 0.20–0.22 ms | 0.35–0.38 ms |

- The `LinearOpMode` loop does real work and is still ≈1.2 ms faster: about 8% of the Ivy log's 15.3 ms mean loop.
  `LoopProfiler` reports the SDK's share as `unprofiled`: loop time minus the top-level sections
  (`architecture/telemetry/LoopProfiler.java:51`).

## Per-command costs

Bench p50s from [`hub-bench-findings.md`](hub-bench-findings.md). A write costs only in loops where its value
changes; a skipped (cached) write costs about 0.

| | Control Hub | Expansion Hub (RS485) |
|---|---|---|
| bulk read (all encoder, analog and digital ports of that hub) | 2.1–2.5 ms | 4.0 ms |
| motor or servo write, changed value | 1.55–1.64 ms | ≈2.5 ms, estimated |

- The Expansion Hub write is not measured. It is the Control Hub write plus the 0.9 ms RS485 hop measured on ADC and
  bulk reads. On USB, an Expansion Hub write measured about 2.6 ms ([Photon](#photon), Photon off).
- With manual bulk caching a hub's bulk read runs only in loops where one of its inputs is read: any input on the
  Expansion Hub adds its bulk read to every loop (both hubs together 5.7 ms against 2.1 ms for the Control Hub alone).
- Where each device of the BIOBUZZ robot should plug in, and the loop period that gives, is in
  [`wiring-findings.md`](wiring-findings.md): about 19 ms parked and 28 to 37 ms driving and aiming (estimated).
  `scripts/looptimes/wiring_budget.py` adds the costs up.

## How often a write re-send reaches the hub

`WriteCache` re-sends an unchanged value every 250 ms (`RESEND_NANOS`, `architecture/hardware/WriteCache.java:4`), but
for motor powers and servo positions only every second re-send reaches the hub, about every 0.5 s. So a dropped or
NACKed write is repaired about 0.5 s later, and lowering `RESEND_NANOS` doesn't make that faster. Keep the re-send
([`CLAUDE.md`](../CLAUDE.md), Module pattern).

- The SDK caches each commanded motor power and servo position in a `LastKnown` that stays fresh for 500 ms, and skips
  a write equal to a fresh value. It stores the new value before sending and keeps it when the send fails or is
  NACKed, so it skips our re-sends of a lost value until 500 ms have passed (read from source, SDK 12.0.0: `LastKnown`,
  `LynxDcMotorController.internalSetMotorPower`, `LynxServoController.setServoPosition`).
- The SDK caches each motor's run mode and each motor's and servo's enable the same way. `DcMotorImpl.setPower` asks for the mode on
  every call, and the SDK re-reads it from the hub once its copy is over 500 ms old; each write that gets through
  re-sends the enable once that copy is over 500 ms old. So a motor held at one power costs about three hub commands
  every 0.5 s (mode read, power, enable) and a servo two (position, enable), and the calls in between ≈0.03 ms each.
  A write costs ≈1.6 ms on the Control Hub (bench p50, [`hub-bench-findings.md`](hub-bench-findings.md)) and an
  estimated ≈2.5 ms on the Expansion Hub ([above](#per-command-costs)). The same expiry, on a motor written every
  loop, is the ~500 ms loop spike in [`CLAUDE.md`](../CLAUDE.md) (Dependencies). Read from source, SDK 12.0.0: `DcMotorImpl.setPower`,
  `LynxDcMotorController.internalSetMotorEnable`, `LynxServoController.internalSetPulseWidth`.
- `setVelocity` is the exception: the SDK doesn't cache it (`LynxDcMotorController.setMotorVelocity` sends on every
  call), so `EnhancedMotor.setVelocity`'s 250 ms re-send (`architecture/hardware/EnhancedMotor.java:63`) reaches the
  hub every time.

## Fixed-period loops

Intake Transfer Shooter Test runs `loop()` on a fixed period, `loopMs` (15 ms by default), and shows `Loop (ms)` and
`Late loops (over loopMs + 1 ms)`. None of it is measured on the robot yet. Expect late loops about every 0.5 s
whatever the budget: the test sets every device every loop through the plain SDK classes, so as the SDK's 500 ms
caches expire it re-reads four motor modes and re-sends the unchanged powers, the servo positions and every enable,
about 18 hub commands every 0.5 s, in one loop or a few ([above](#how-often-a-write-re-send-reaches-the-hub); read
from source). By the per-command costs below, 15 ms is also tight between those loops with both flywheels in closed
loop: if `Late loops` climbs faster than a few every 0.5 s, read the motor currents less often first.

How it works (`opmodes/test/IntakeTransferShooterTest.java:120–128`):

- At the top of `loop()`, park until the deadline: `while ((waitNs = nextLoopNs - System.nanoTime()) > 0)
  LockSupport.parkNanos(waitNs)`. The `while` covers early or spurious wake-ups.
- If the loop starts more than 1 ms (`LATE_TOLERANCE_NS`) after its deadline, count it as late and restart the schedule
  from now, instead of running short loops to catch up.
- Then move the deadline on by one period.
- Waiting at the top puts the SDK's time between `loop()` calls (1.42 ms p50 for an empty loop, above) inside the
  period instead of on top of it.

Estimated cost per loop, from the bench p50s (unverified on the robot; `intake_transfer_shooter.xml` puts every device
on the Control Hub):

| | ms |
|---|---|
| one bulk read, covering both flywheel encoders | 2.2–2.5 |
| four motor current reads | ≈5.4 |
| battery voltage | ≈1.3 |
| two flywheel power writes, changing every loop in closed loop | ≈3.3 |
| unchanged powers and positions (the SDK skips them) | ≈0.03 each |
| total, before telemetry and the SDK's time between loops | ≈12 |

## Other teams' Decode code (2026-10-25)

| team / repo | relevant to loop time |
|---|---|
| FTC-23511 `Decode-2026` | PhotonCore with 8 parallel commands; caching tolerance 0.01 on every motor, 0.02 on CR servos, 0.001 on turret and hood servos; launcher and turret encoders read through drive motor ports; OctoQuad localizer (`readLocalizerData` only); voltage read on a timer; per-loop profiler to CSV |
| Meta-Infinity `DecodePublicRelease` | per-device write thresholds: drive, intake, transfer 0.02, shooter 0.005, hood 0.002, turret and latch 0.001; Pinpoint |
| kleongf `FTC_Decode` | caching thresholds 0.005–0.03; reads the voltage sensor in every subsystem every loop (≈1.3 ms each: avoid) |
| FTC11329 `11329-2025-repo` (Into the Deep) | skips writes only on exactly equal values; no bulk caching; reads its color sensor several times per loop |

- We already do bulk caching, throttled voltage reads and the write re-send; Pedro's own `CachedMotor` skips drive
  changes under 0.01.
- SDK 12's `GoBildaPinpointDriver` already runs I2C at 400 kHz, so forcing it (as 11329's old driver did) gains nothing.

## Photon

Photon (`architecture/hardware/photon/`, the team's fork of SolversLib Photon) sends motor and servo writes without
waiting for each one's ACK. Only a `@Photon` OpMode's INIT swaps the hubs; OpModes after it keep the swapped hubs,
disabled, until the hardware map is rebuilt (config change or Restart Robot). See [`CLAUDE.md`](../CLAUDE.md),
Framework layout.

Measured on 2026-10-28 with stock SolversLib Photon 0.3.6 (before the fork), Control Hub plus an Expansion Hub on USB,
nothing on the ports; 4 motors and 2 servos changing on each hub every loop, p50 (p95):

| | write block, both hubs | Expansion Hub | Control Hub | loop period |
|---|---|---|---|---|
| Photon off | 25.9 (41.0) ms | 15.9 ms, ≈2.65 ms a write | 9.35 ms, ≈1.56 ms a write | 31.8 ms |
| 8 parallel | 6.0 (12.3) ms | 3.95 ms | 2.06 ms | 11.5 ms |
| 2 parallel | 6.8 ms | 4.39 ms | 2.31 ms | 12.4 ms |

That run also showed why stock Photon can't go on the robot: at INIT it registered the Control Hub's replacement module
in the Expansion Hub's USB device, so the Control Hub's own replies went to the replaced object, and a timed-out command
would have left that hub marked unresponsive for good. Its read-back failures there were the bench's own sign error
(goBILDA 5202 motors are CCW, so the hub's register holds minus the power), and its STOP check could not fail.

The fork as first committed, on the same hubs, 2026-10-29, same workload, p50 (p95):

| | write block, both hubs | Expansion Hub | Control Hub | loop period |
|---|---|---|---|---|
| stock (an OpMode Photon never touched) | 24.96 (41.7) ms | 15.43 ms | 9.28 ms | 30.7 ms |
| fork, 8 parallel | 10.84 (14.2) ms | 6.20 ms | 4.66 ms | 16.0 ms |
| stock SolversLib Photon, 8 parallel (2026-10-28, above) | 6.0 (12.3) ms | 3.95 ms | 2.06 ms | 11.5 ms |

So the fork cut the write block 2.3x and the loop period from 30.7 to 16.0 ms, against 4.2x and 11.5 ms for stock
Photon, which skips the fork's safety checks and is unsafe on the robot. Each parallel write cost the fork about 0.4 ms
more than stock Photon (motor write p50 0.91 against 0.40 ms); a desktop comparison ranked its ping-timer reset on every
write as the biggest part of that. The current tree resets it at most every 100 ms and can time each step of the write
path (`timing on8 profiled` in the Photon block), but neither is measured on a hub yet; its write block also leaves out
the bench's own bookkeeping, at most about 0.05 ms a write, so compare its numbers with these loosely.

The fork changes that, but only the rebuilt Photon block (`scripts/bench/run_bench.py --photon`, see
`scripts/bench/README.md`) decides it: exact read-backs with Photon off and on, a stock baseline from an OpMode Photon
never touched, routing and not-responding recovery over 8-16 app restarts, real lost writes, STOP at random points of the
loop with negative controls, STOP inside a write, crashes and consecutive OpModes, deploySloth, a hub reset, the RS485
wiring, cable pulls, I2C reads on both hubs, and a soak that bounds the per-write loss rate. Use Photon on the robot only
once its summary says SAFE TO TRY ON THE ROBOT for the robot's wiring: only the go run for that wiring (the soak and
`--photon-i2c`, plus the cable pulls on USB; see the README's table) can say it, and a shorter run says PASSED at best.

## Still unknown

- **Intake Transfer Shooter Test's real period**: set both flywheels to a target RPM (closed loop) on the dashboard,
  close its tab ([`CLAUDE.md`](../CLAUDE.md), Dependencies), and watch `Loop (ms)` and `Late loops` on the DS; check
  whether the late loops come about every 0.5 s.
- **How accurately `LockSupport.parkNanos` wakes on the Control Hub**: in that OpMode with the loop body emptied, log
  how far past its deadline each loop starts.
- **What a `LinearOpMode`-driven framework loop saves** (≈1.2 ms estimated): compare Bench: Framework Loop's period with
  the same loop driven from a `LinearOpMode`.
- **The ≈0.5 s repair time** (read from source): on the bench, while an `EnhancedMotor` holds a power, overwrite that
  port's power with a raw Lynx command that bypasses the SDK, and poll the hub's power register until it comes back;
  expect within about 0.5 s, not 0.25 s.
