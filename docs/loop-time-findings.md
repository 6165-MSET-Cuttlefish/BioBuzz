# Loop-time findings

What costs loop time on the Betta bot, and what to change. Hub command costs come from `docs/hub-bench-findings.md`;
re-measure with Ivy Path Test's log (`scripts/looptimes/analyze_ivy.py`) after any hardware or layout change.

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
| unprofiled (SDK between loops) | 1.4 | |

- **Pedro is the loop.** `follower.update` costs 7.7 ms with the robot paused, mostly the Pinpoint read (5.8 ms on the
  bench), and 13.4 ms while driving, adding path math and four drive writes, two of them on the Expansion Hub.
- **`write.BettaShooter` spikes to 5–17 ms every ~0.4 s** and is 0.03 ms otherwise: the SDK re-reading each motor's
  mode and re-sending its enable, not our code.
- **`updateTelemetry` passes 4 ms about 6 times a second**: the DS frame and its field map.
- **Garbage collection:** 17 collections took 1.1 s of 19.6 s. Some 20–46 ms `follower.update` spikes match nothing
  else and are probably collector pauses; the log can't prove it.

## Estimated per-loop costs of the planned hardware

Per-command bench costs added together; expect a loaded robot to cost more. A write costs only in loops where its
value changes, a skipped (cached) write about 0.

| | Control Hub | Expansion Hub (RS485) |
|---|---|---|
| bulk read (all encoder, analog and digital ports of that hub) | 2.4 ms | 4.7 ms |
| motor or servo write, changed value | 1.55–1.64 ms | ≈2.5 ms |

- **Inputs** (2 Brushland rangefinders on analog, 2 flywheel Through Bores): all on the Control Hub they share one bulk
  read; any input on the Expansion Hub adds its bulk read every loop (both hubs together ≈6.3 ms).
- **Outputs that change every loop**: 4 turret servos while auto-aiming (≈6.2 ms), 2 shooter motors on a velocity PID
  (3.2–5 ms), 4 drive motors while driving (≈6.6 ms, already in today's 19 ms), 2 hood servos if set from distance
  (≈3.1 ms). Hardstop, intake servo, intake and transfer motors change rarely (≈0).
- **Worst case while driving and auto-aiming**: ≈36 ms (28 Hz) with the layout below, ≈44 ms with inputs split across
  hubs and the turret on the Expansion Hub; about 22 ms with turret and hood still and fixed-power shooters.

### Layout

| | Control Hub | Expansion Hub |
|---|---|---|
| motors | both flywheels on ports 0 and 3, two drive motors on 1 and 2 | two drive motors, intake, transfer |
| encoders | the flywheels' Through Bores on encoder ports 0 and 3 | none |
| servos | 4 turret, 2 hood | hardstop, intake servo |
| analog | both rangefinders | none |
| I2C | odometry | none |

- Flywheels go on ports 0 and 3 of one hub with their encoders on the same ports: ports 1 and 2 count in software and
  drop counts at flywheel speed (CLAUDE.md, Flywheel RPM). One hub keeps a single bulk read.
- Leave the Expansion Hub with no inputs, so its bulk read never runs, and give it outputs that rarely change.
- Two drive motors stay on the Expansion Hub, as today (+≈1.8 ms while driving).
- Through Bores on the OctoQuad instead: its full read is 6.6 ms against 3.2 ms for odometry alone (+3.4 ms every
  loop), while on the Control Hub they ride the bulk read the rangefinders already need. Use the OctoQuad only for its
  on-board velocity or if the hub's encoder ports run out.

## What saves more than the layout

1. **Write tolerances**: we use `withCachingTolerance(0.001)` on 16 devices; other teams skip far more (below). Try
   0.005 on the shooter PID output, 0.02 on intake and transfer, 0.002–0.005 on the hood; keep 0.001 on the turret.
2. **OctoQuad for odometry**: 3.2 ms against the Pinpoint's 5.8 ms every loop, with far less heading drift. Needs the
   guarded localizer `docs/hub-bench-findings.md` describes (a missing OctoQuad hangs INIT).
3. **Fewer per-loop allocations**, to cut collector pauses.
4. **A rangefinder poller thread** costs about 3 ms while driving: use one only to read faster than the loop.

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

SolversLib Photon (`org.solverslib:photon`) sends motor and servo writes without waiting for each one's ACK, so it
could remove most of the per-write cost above. It is unstable, and installing it alone changes every OpMode (see
CLAUDE.md, Dependencies). Bench: Photon (`scripts/bench/run_bench.py --photon --usb-expansion`) checks write costs,
lost or reordered writes, bursts, reads, a poller thread, STOP with writes in flight, and the framework loop with it on.
It hasn't run yet: decide on Photon only from its results.
