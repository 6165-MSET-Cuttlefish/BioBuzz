# Control Hub bench findings

What the Control Hub costs per command, and how it behaves at STOP, on a crash, on an app restart and on deploySloth.
Read it before changing the planner, hub or I2C traffic, odometry, the Limelight, background threads or stop paths;
after such a change, rerun `scripts/bench/run_bench.py --ports-empty` (on a bare hub, `scripts/bench/bare_hub.sh`) and
update the numbers here.

Java paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Times are p50 / p95 / max in ms unless
noted. Numbers without a date are from 2026-10-09; the hardware for each run is under [Setup](#setup).

## Design around these

- **Route planner: one `RouteOptimizer.Search.step()` per loop; rerun Bench: Hub and Bench: Planner Stop on a hub
  before trusting a STOP during a 4-ball plan.** Ball Collection Auto, Vision Ball Collection and Ball Collection plan
  4 balls (`BallCollection.BALLS`) on the OpMode thread, one step per loop. A step shapes visit orders for
  `VisibilityGraphPlanner.routeSearchSliceMs` (20 ms) and overruns it by at most one `shape()` call
  (`architecture/planner/RouteOptimizer.java:143-154`), so STOP waits for one step, not a plan
  ([`ball-planner-findings.md`](ball-planner-findings.md)). The sliced search hasn't run on a hub: every hub number
  below was measured planning in one call.
  - Estimate for the sliced search (simulated: a desktop replay of the 2026-10-24 layouts that drops the same balls as
    the hub and ran 28–35× faster, median 32×, scaled back by that): the longest step about 110 ms (100–115) for
    random 4-ball layouts and about 240 ms (210–260) for the 4-ball Planner Stop layout. From request to done in an
    `EnhancedOpMode`, whose loop runs between steps, a 4-ball plan takes about 1.3× as long as in one call (random
    layouts p50 about 730 ms, worst about 3.3 s). STOP would wait about 255 ms (up to about 275) in Vision Ball
    Collection and Ball Collection; in Ball Collection Auto, one step plus the loop's tail plus a webcam close of up to
    581 ms (below) comes to about 830 ms (up to about 855), 45–70 ms inside the 900 ms window.
  - Why it's slow (read from source): it bounds every order of the full ball set (n!) and shapes them best bound
    first, dropping to n − 1 balls only when no full order is drivable
    (`architecture/planner/RouteOptimizer.java:171-214`). One awkward ball makes it shape every full order first, so
    the layout matters as much as the count.
  - Planning time in one call on 2026-10-24, for the 16 × 15 in robot with its intake 8 in ahead that the code uses
    now, in one half of the field with the HIVE rails and 1 in gaps (`opmodes/test/bench/PlannerWorkload.java`; the
    two both-sides rows are Bench: Planner Stop's replans, the rest Bench: Hub's):

    | balls and layout | p50 | p95 | max | n |
    |---|---|---|---|---|
    | 3, random | 141 | 333 | 465 | 60 |
    | 4, random | 558 | 1888 | 2434 | 48 |
    | 4, on both sides of the HIVE rail | 1452 | 1728 | 1728 | 15 |
    | 5, random | 1934 | 13067 | 14769 | 40 |
    | 5, on both sides of the rail | 902 | 969 | 969 | 15 |
    | 5, in a line behind the rail | 4369 | 4450 | 4450 | 5 |
    | 5, in a tight cluster | 2.0 | 2.3 | 2.3 | 5 |

  - The first 4-ball plan in a fresh app process took 2046 ms, on its first random layout. On the fixed 4-ball Planner
    Stop layout, the first plan in a fresh app process took 1581 and 1728 ms against 1343–1529 for later plans.
  - STOP during a plan made in one call (Bench: Planner Stop, 2026-10-24: 4 and 5 balls, replanning back to back,
    STOP sent 0.05, 0.3 or 0.6 s after a plan ended): 1 of 6 trials restarted the app. That STOP landed 474 ms into a
    4-ball plan; `stop()` began 1068 ms after STOP, and the SDK logged `stuck in stop()` 1.006 s after
    `auto-stopping` and restarted the Robot Controller app. In the other 5, `stop()` began at most 744 ms after STOP.
  - A restart loses dashboard edits and the alliance. The dashboard answered again about 16 s after STOP, but no
    OpMode could run until 44.9 s after it (`Robot Status: running` in logcat).
- **A missing or faulting Pinpoint raises nothing.** Before any Pedro-driven run, check that the status telemetry's
  `Robot Position` line reads the start pose after INIT: (0, 0, 0) means the Pinpoint isn't answering.
  - Missing (2026-10-24, nothing on I2C bus 1): `BettaConstants.create` built the follower in 571 ms with no error,
    `follower.update()` ran 4.25 / 5.01 / 19.64 (n=200) with no error, and after `follower.setPose(56, 8, 90°)` every
    sampled pose read (0, 0, 0) (`opmodes/test/bench/PinpointAbsentBench.java`). `hardwareMap.get` only needs the
    config to name the device. With no device version read, SDK 12's `GoBildaPinpointDriver.update()` reads nothing
    (read from source), so the pose stays at (0, 0, 0) however the robot moves, and Pedro drives without ever seeing
    progress.
  - Faulting (read from source): on a read with a NaN or an implausible value (its default `LOCAL_TEST` check; ours
    reports device version 4), the driver keeps the previous value and reports `FAULT_BAD_READ`. A failed I2C read
    returns zero bytes (`TimestampedI2cData.makeFakeData`), which pass that check, so a Pinpoint that stops answering
    mid-run should snap the pose to (0, 0, 0) with status `NOT_READY` rather than freeze it (not yet seen on a hub).
    Pedro 3.0.2's `PinpointLocalizer.update()` never reads `getDeviceStatus()`, and only the test OpModes Pinpoint
    Localizer Test, Pinpoint Drive Test and Ball Field Drive show it.
  - A guard, like `GuardedOctoQuadLocalizer` for the OctoQuad (below), must check at INIT that the Pinpoint answers
    and stop with an error naming it and its port, then check the status on every read, counting non-READY only after
    the first READY. The Pinpoint reported `FAULT_BAD_READ` right after `BettaConstants.create`, then READY 71 ms
    later.
- **Every app start takes about 37 s; budget 40–45 s for a config switch or a STOP-time app restart at a match.**
  On 14 config switches on a bare hub (2026-10-24), process start to `INIT FINISH` in logcat took 37.1–38.1 s.
  - A config switch over adb, timed by the bench runner from the switch to the dashboard reporting the robot ready,
    took 42.4–44.6 s on 2026-10-24.
  - 12.1–12.3 s of it is one class scanner, Pedro AutoTune's `com.pedropathing.tuning.autotune.TunerScanner`
    (`com.pedropathing:tuning:1.0.1`, `TeamCode/build.gradle:86`), which calls `getDeclaredMethods()` on each class it
    is handed. Sinister, the class-scanning library under Sloth, runs it at app start, and the main thread waits on
    Sinister's scanners (`running scanners for load`) until it ends.
  - Every start logs an ANR (Android's "application not responding" report), `ANR in com.qualcomm.ftcrobotcontroller`
    (reason: executing a service, `FtcRobotControllerWatchdogService` in 26 of 27), about 26 s in: 13 of 13 starts on
    2026-10-09, 14 of 14 on 2026-10-24. The same process went on to finish INIT every time, so it isn't a crash.
- **The SDK's stop window is 900 ms** (`OpModeInternal.MS_BEFORE_FORCE_STOP_AFTER_STOP_REQUESTED`), plus 100 ms of
  grace, covering whatever `init_loop()`/`loop()` is running and `stop()`. Overrunning it restarts the app.
  - In the 100 ms of grace every hub command throws `ForceStopException`, a `finally`'s too, so a loop still running
    at 900 ms can't put hardware state back (read from source: SDK 12.0.0 `OpModeStuckCodeMonitor` calls
    `setThrowOnNetworkLockAcquisition(true)`; `MessageKeyedLock.acquire` throws). An iterative OpMode sees STOP
    mid-loop only through STOP's gate, `OpModeManagerImpl.shouldPreventDangerousHardwareAccess()`, raised as STOP
    begins: Bench: OctoQuad's bank phase checks it before each read.
  - Webcam close during STOP: `CellTipCamera.stop()` took 574–581 ms when STOP came right after INIT, and 61–319 ms
    later in the run.
  - A raw `WebcamSession.close()` made 200 ms after opening took 685 ms.
  - So webcam teardown can use most of the window.
  - It doesn't have to be in the window (read from source, untried on a hub). SDK 12's
    `OpModeManagerImpl.callActiveOpModeStop` fail-safes the hubs, runs `stop()` under the 900 ms watchdog, then
    calls the post-stop listeners with no watchdog. EasyOpenCV 1.7.3 (SDK 12 Vision's dependency) registers every
    camera when it's built and closes it from that post-stop call (`OpenCvCameraBase`). `CellTipCamera.stop()`
    (`biobuzz/modules/CellTipCamera.java:88-94`) closes it inside the window instead, through
    `WebcamSession.close()` (`biobuzz/modules/vision/WebcamSession.java:108-116`). To win the time back, leave the
    close to EasyOpenCV or use `closeCameraDeviceAsync`, keeping the dashboard `stopCameraStream()`. The
    `CellTipPipeline.release()` call can stay in `stop()`: it frees only the native AprilTag detector, under the lock
    `processFrame` holds, so a frame that arrives after it skips detection
    (`biobuzz/modules/vision/CellTipPipeline.java:94-95` and `:161-167`).
- **deploySloth: wait for the `Processed Sloth Load` toast before INIT ([`CLAUDE.md`](../CLAUDE.md), Build &
  deploy).** A fixed 5 s wait is marginal; without the toast, allow 6 s.
  - The jar loaded 4.3–4.8 s (2026-10-09) and 4.7–5.2 s (2026-10-24) after the push, which comes about 0.1–0.2 s
    before deploySloth returns.
  - INIT sent the moment the jar was pushed, or right as deploySloth returned, restarted the app (both dates).
  - INIT sent 0.5–2 s after it returned didn't report within 30 s (both dates).
  - INIT 5 s after ran the new code (both dates); on 2026-10-24 the load had ended only about 0.2 s before.
  - A deploySloth sent while an OpMode runs leaves it running; the load is staged and lands after STOP, processed
    11.25 s after the push (2026-10-24).
  - A hot-loaded jar survives an app restart.
  - The OpMode list stayed correct throughout a load.
- **After a hub fail-safe, `EnhancedMotor` and `EnhancedServo` reach the hub again only on `WriteCache`'s re-send: up
  to 250 ms plus a loop.** Nothing clears the cache on a fail-safe, and an unchanged value goes out again only once
  `RESEND_NANOS` (250 ms) has passed (`architecture/hardware/WriteCache.java:4` and `:18-26`). Don't lengthen it.
  - On 2026-10-24 raw SDK devices were back 1.9 ms after `failSafe()` and the wrapped ones 21.5 ms after, in one
    trial each. The SDK forgets its own last-written values on a fail-safe (`LynxModule.failSafe`), so a raw device
    re-sends on its next write.
  - A command the hub never answers is resent every 100 ms and abandoned at 250 ms. The hub is then marked not
    responding, and every command to it fails at once until it sends something back. User code never sees the
    dropped write; logcat shows `timeout: abandoning waiting 250ms for` (read from SDK 12.0.0 source:
    `LynxRespondable.awaitAndRetransmit`, `LynxCommExceptionHandler`).
- **Expansion Hub: every command costs more, not just encoders.** The RS485 hop added about 0.9 ms to each ADC read
  sent to the Expansion Hub, and 1.5 ms to a bulk read. Every Lynx command crosses the same link, so expect the same
  on motor and servo writes and on each command of an I2C transaction; those were not measured over RS485.

  | command | Control Hub | Expansion Hub |
  |---|---|---|
  | input voltage (ADC) | 1.72 / 2.37 / 7.47 | 2.61 / 3.32 / 8.69 |
  | current (ADC) | 1.69 / 2.43 / 4.58 | 2.61 / 3.49 / 5.75 |
  | bulk read | 2.47 / 3.26 / 11.87 | 4.01 / 4.67 / 7.64 |

  - With manual bulk caching, using any Expansion Hub input means one more bulk read per loop:
    - Control Hub bulk read alone: 2.10 / 2.43 / 4.21.
    - Both hubs' bulk reads: 5.72 / 6.27 / 7.08.
    - Both hubs plus the Pinpoint and OctoQuad reads: 14.62 / 17.33 / 22.35.
  - An I2C read takes several Lynx commands, so I2C devices pay the hop several times per read. The Pinpoint and
    OctoQuad weren't measured on the Expansion Hub. Keep odometry and other per-loop I2C sensors on the Control Hub.
- **Odometry, measured on the Control Hub:**

  | | Pinpoint | OctoQuad MK2 |
  |---|---|---|
  | one read | `update()` 5.79 / 6.73 / 9.33 | `readLocalizerData` 3.20 / 3.77 / 8.22 (3.66 / 4.49 / 5.27 before Pedro's localizer was built) |
  | building the localizer | 585 ms | 1666 ms (Pedro waits for RUNNING) |
  | heading drift while still | **6.1° in about 5 s** | 0.01° |

  - 500 ms of the Pinpoint build is a fixed sleep in Pedro's `PinpointLocalizer.reset()`, whatever the reset mode
    (read from source; the same in 3.0.1, which the 585 ms ran on, and 3.0.2).
  - The OctoQuad had 0 bad reads in 383 loops. `readLocalizerDataAndAllEncoderData` costs 6.58 / 8.51 / 17.67.
  - Pinpoint drift: the 0.7 in position error that came with it is the heading error times the 6.5 in pod offset.
  - Rerun the Pinpoint step with the board untouched before blaming the Pinpoint. If the drift holds, a start pose set
    in INIT won't survive it, and neither would a re-seed at START.
- **Pedro's `OctoQuadLocalizer` hangs INIT when the OctoQuad is missing: give a follower that uses the OctoQuad
  `GuardedOctoQuadLocalizer` (`architecture/hardware/`) instead.** No robot's follower uses either yet; only the
  OctoQuad benches build the guarded one. Pedro's constructor (measured on 3.0.1 on 2026-10-09, unchanged in 3.0.2)
  waits for RUNNING in a loop with no timeout and no STOP check.
  - With nothing on the bus, or an MK1 reporting `FAULT_NO_IMU`, INIT hangs with no message.
  - STOP gets out: the SDK force-stopped the OpMode in about 1.3 s without restarting the app.
  - Pedro's `update()` also silently skips invalid reads (CRC error or not RUNNING) and keeps the last pose.
  - `GuardedOctoQuadLocalizer` with nothing on the bus (Bench: OctoQuad Absent, 2026-10-24): the bench robot built in
    75 ms, the `OctoQuad` fault came 2979 ms into INIT, INIT kept looping (7.26 / 10.06 / 17.68), and START stopped the
    OpMode with an error naming the OctoQuad. It hasn't run with an OctoQuad plugged in.
- **Limelight ball pipeline (pipeline 4): slow with balls in view.** Design vision-dependent timing for about 4–5 new
  results a second and about 0.26 s from capture to the hub.
  - Pipeline latency (`tl`) by the ball count it reported: every count from 3 to 10, p50 209–225 (n=95), flat rather
    than rising per ball. 0 balls, p50 46 (41–61, n=5), but all five were the first frames after switching to the
    pipeline, so an empty view being that fast is unconfirmed. 1 and 2 balls were never in view. The SnapScript's
    detection steps (`limelight/ball_contour_snapscript.py`) haven't changed since 2026-10-09.
  - With balls in view: about 4.7 new results per second; `tl` 227 / 244 / 254; capture to arrival on the hub
    263 / 284 / 352; new results 229 / 246 / 266 apart.
  - Pipeline 0 (fiducial) runs at about 44 fps, `tl` about 25.
  - Ball velocities and "fresh frame" logic have to tolerate about a quarter second of age, so `staleFrameSeconds`
    (0.5 s) counts from a result's arrival on the hub, not its capture.
  - `LimelightSync` takes about 1.7 s, and INIT to the first ball frame about 1.8 s.
  - The poller's rate (100–250 Hz) doesn't change the frame rate. It costs CPU, not loop time: the RC process used 81%
    of a core with the poller off, 124% at 100 Hz, 163% at 200 Hz and 165% at 250 Hz, and loop p50 rose 0.15 ms at
    most.
- **A Limelight that never answers a POST holds back only the sync and its fault, for one 15 s POST timeout** (Bench:
  Limelight Hung, 2026-10-24: the real `Camera` against a fake Limelight that serves results and holds every POST
  open).
  - The `Limelight` fault came 15.2 s into INIT. INIT kept looping (3.28 / 5.22 / 16.99), and after START the loop
    ran at 267 loops/s.
  - One POST was sent, not retried.
  - STOP during a hung POST isn't held up: the stop pass took 1.2–2.9 ms.
  - The next INIT doesn't wait for it (first INIT loop 142–172 ms), but its sync waits out the stopped run's POST,
    which ended 15.0 s after it was sent, then took about 1.6 s more: done 13.1–13.6 s after INIT, the old POST having
    been sent 3.0–3.5 s before it. So a re-INIT right after STOPping on a hung Limelight has ball vision about 16.6 s
    later at worst (estimate from those times).
  - Two `LimelightSync` workers, as a Sloth reload would leave, never had POSTs open at once
    ([`limelight-findings.md`](limelight-findings.md)).
- **Webcam controls:** `WebcamControls.update()` with a write (exposure or gain) blocks for 72 / 87 / 87. Write them
  only in INIT or when they change.
- **A rangefinder poller thread gains no readings at the default 20 ms timing budget.** Configure Laser Rangefinder
  sets `timingBudgetMs = 20` with `measurementPeriodMs = 0` (`opmodes/test/ConfigureLaserRangefinder.java:18-19`), and
  a period of 0 starts each range as the last one ends, so each sensor gives a new distance about every 20 ms (50 Hz);
  a 14–17 ms driving loop already reads every one. Read the rangefinders in the loop's bulk read, or set both to a
  10 ms budget with Configure Laser Rangefinder before adding a poller. The vendored driver takes 5–1000 ms
  (`vendor/brushland/LaserRangefinder.java:74`).
- **Analog refresh and poller threads** (Bench: Analog Poll, run by hand with `laser_rangefinder.xml`: 10 ms on
  2026-10-14, then 5, 7 and 10 ms on 2026-10-16 with both rangefinders plugged in):
  - The hub refreshes every ADC channel about every 10 ms. Back-to-back reads saw a new value every 9.7–11.1 ms
    (p50), never under about 9 ms, on both rangefinders, the battery monitor and the 5 V monitor, through GetADC or
    bulk reads. Hub ADC noise changes the reading at most refreshes, so this can't show the rangefinder's own update
    rate. Both were at a 20 ms timing budget (50 Hz) in every run.
  - A poller thread doing one bulk read per poll against a stand-in loop (MANUAL bulk read, 0, 6 or 8 GetADC commands
    for odometry and drive writes, 2.7 ms of CPU), mean loop period added over the same run's no-poller loop:

    | loop (no poller) | 10 ms poll | 7 ms poll | 5 ms poll |
    |---|---|---|---|
    | idle (6.4–6.5) | +0.3 | +0.6 | +0.4 |
    | OctoQuad driving (13.8–14.0) | +3.0 | +3.4 | +6.6 |
    | Pinpoint driving (16.4–16.9) | +3.2 | +4.4 | +8.7 |

    The 2026-10-14 10 ms run agreed (+2.6 and +3.7 driving). Two GetADC reads per poll cost more than one bulk read
    at every rate (+3.6–3.8 at 10 ms, +9.8–11.8 at 5 ms, driving).
  - New `analog0` values per second while driving: the loop alone 59–71; the poller 97–99 at 5, 7 and 10 ms alike.
    The extra values are ADC noise at each refresh, not new distances. A 10 ms poller already sees almost every
    refresh, so polling faster only costs loop time.
  - The poller kept its schedule at every rate (p99 within 0.9 ms of the period); its bulk read took 2.0–2.7 ms
    (p50) under load.

## Hub costs (Control Hub, bulk caching off unless noted)

| operation | 2026-10-09 | 2026-10-24 |
|---|---|---|
| empty OpMode loop | 1.42 / 1.69 / 6.08 | 1.42 / 1.68 / 4.92 |
| bulk read (clear + first read) | 2.22 / 2.97 / 21.55 | 2.34 / 2.93 / 5.10 |
| cached read after a bulk read | 0.03 / 0.12 / 0.59 | 0.04 / 0.11 / 0.68 |
| `getCurrentPosition` or analog read, uncached | 1.27 / 1.55 / 3.52 | 1.28–1.31 / 1.66–1.74 / 5.23 |
| `getVoltage` / hub input voltage | 1.30 / 1.56 / 2.94 | 1.26–1.28 / 1.64–2.17 / 4.10 |
| hub current / motor current | 1.35–1.37 / 1.67–1.81 / 3.05 | 1.25–1.28 / 1.54 / 3.34 |
| motor `setPower`, changed value | 1.64 / 1.96 / 6.13 | 1.65 / 2.18 / 4.01 |
| motor `setPower`, same value (SDK skips it) | 0.03 / 0.04 / 5.27 | 0.03 / 0.03 / 3.84 |
| four motor writes | 5.66 / 7.73 / 16.31 | 5.69 / 7.57 / 14.05 |
| servo `setPosition`, changed | 1.55 / 2.02 / 2.90 | 1.54 / 1.85 / 4.34 |
| IMU yaw/pitch/roll (BNO055) | 5.12 / 6.21 / 7.29 | 5.26 / 6.51 / 9.49 |
| dashboard packet GSON (about 5.6 k chars) | 2.48 / 3.59 / 10.09 | 2.52 / 3.24 / 10.41 |
| LLResult parse, ball script result (`JSONObject`) | about 0.8 | 0.73–0.82 (p50) |
| LLResult parse, 24 tags + 64 detections | 28.6 / 32.1 / 36.2 | 29.4 / 34.5 / 41.3 |
| Limelight POST over USB (`updatePythonInputs` etc.) | 4.4–5.6 / 6.2–7.8 / 10.3 | not measured (no Limelight) |
| `pipelineSwitch` to a Limelight that hangs | blocks 15 s | blocks 15 s |
| `pipelineSwitch`, configured Limelight unplugged | not measured | 3.76 (3 calls) |

## EnhancedOpMode loop (Betta bench robot, Drivetrain + Pedro, idle follower)

| setting | 2026-10-09 | 2026-10-24 |
|---|---|---|
| defaults | 6.32 / 9.44 / 25.57 | 6.34 / 9.11 / 21.58 |
| DS telemetry every loop | 8.49 / 11.62 / 28.23 | 7.67 / 12.14 / 27.29 |
| dashboard telemetry off | 4.21 / 7.24 / 25.54 | 4.29 / 6.61 / 25.52 |
| loop profiler off | 5.00 / 8.06 / 14.81 | 5.49 / 7.68 / 20.65 |
| Drivetrain current telemetry on | **12.13 / 16.60 / 26.74** | **12.14 / 16.46 / 24.59** |
| hub current telemetry on | 7.58 / 10.70 / 26.43 | 7.51 / 11.01 / 19.69 |
| following a path | 4.02 / 5.88 / 33.32 | 4.09 / 6.13 / 28.61 |
| Close Flower's loop without the framework, following | 1.03 / 1.33 / 27.76 | 1.00 / 1.27 / 28.01 |
| a bare LinearOpMode running Pedro, idle | 0.22 / 0.38 / 3.84 | 0.20 / 0.35 / 3.06 |

- Telemetry is the biggest piece: `updateTelemetry` takes about 2 ms of a 6 ms loop, and the loop allocates about
  63 kB.
- Keep Drivetrain current telemetry off outside tuning.
- No Driver Station was connected, so DS frames weren't serialized or sent. A match costs more than these numbers.
- The Limelight fault fallback loop ran at 135 loops/s, 6.79 / 11.05 / 41.87 (2026-10-09), and at 134 loops/s,
  6.89 / 11.00 / 32.42 (2026-10-24).

## Confirmed on the hub (don't "fix" these back)

- **Crashes in an `EnhancedOpMode`:**
  - Every lifecycle stage stops all modules, within about 15 ms of the throw (2026-10-24).
  - A second failure is attached as Suppressed.
  - The SDK reports the crash (`User code threw an uncaught exception`) 0.43–0.50 s after a throw in INIT,
    `init_loop`, a module's `init()`, `onStart` or `loop`, and 0.02 s after a throw in `stop()` (2026-10-24). The
    dashboard Error line showed it 0.12–0.27 s after the bench runner received that report (both dates): outside
    `stop()`, expect over half a second from a throw to the Error line. The line clears at the next INIT.
  - After a throw in `loop()`, the OpMode thread lived another 537 ms; a `TransferCounter` thread tied to its
    `isAlive()` ended at the same moment (2026-10-24).
- **Webcam camera-thread exceptions** stop the OpMode and reach the Error line.
- **Cell-tip frame age** (`captureTimeNanos` against `System.nanoTime()`) runs 6–13 ms, never negative.
- **Dashboard gamepads** reach both iterative and linear OpModes. A held press gives one edge, and the sticks rest
  500 ms after the last message (498.8 and 501.4 ms on 2026-10-24).
- **Edges through START** (2026-10-24): with LB held from INIT through START, an `EdgeBooleanSupplier` first read after
  START fires on the first running loop. One built and invalidated in INIT gives no edge at START and one per later
  press.
- **Turret PWM** (2026-10-24, `BettaTurret`, read from the hub's registers): off at INIT; on at the commanded pulse from
  the first loop (1500 µs centred, 780 µs for TurretR forward); off right after `stop()` and as long as writes are off;
  with writes on, on again the next loop.
- **`TransferCounter`'s poller** (Bench: Thread Lifecycle, 2026-10-24): 100 polls/s, 2 late of 552, longest gap
  12.5 ms, 2.45 ms per bulk read. `stop()` returned in 0.03–9.0 ms over 16 runs. No poller was left at the next INIT
  after a normal STOP, a STOP from INIT, a crash in `loop()`, a `stop()` that throws, 10 back-to-back runs or a Sloth
  reload. A poller that dies stopped the OpMode thread with its cause 8.2 ms (4 loops) later; the SDK reported it about
  0.5 s after that.
- **Drive hand-back:** Drivetrain gets the motors back from Pedro within 2 loops in every hand-back case.
- **Hardware view:** a stale Power edit doesn't drive a motor under the next OpMode.
- **Telemetry survives:** a NaN in a field overlay doesn't kill dashboard telemetry.
- **Limelight proxy:** slothboard's proxy keeps query strings. Every Limelight POST was accepted.
- **Limelight fault:**
  - Raised 456 ms (2026-10-09) and 490 ms (2026-10-24) after INIT.
  - Drawn red on the overlay.
  - Leads every packet.
  - Ball collection ends SKIPPED and hands the Drivetrain back (2026-10-24).
- **Floodgate:** an unwired floodgate pin reads 0.001 V, so Drivetrain's current limiter stays at 1.0.
- **Ivy timeout:** `PathCommands.timeout(15000)` ended 15.008 s (2026-10-09) and 15.006 s (2026-10-24) after START.
- **Config baseline:** slothboard's config baseline lists every `@Config` class.

## Setup

Two runs of `scripts/bench/run_bench.py --ports-empty`, the 2026-10-24 one through `scripts/bench/bare_hub.sh`:

- **2026-10-09:**
  - REV Control Hub, firmware 1.8.2, Android 7.1.2, FTC SDK 12.0.0, BNO055 IMU.
  - REV Expansion Hub at address 2, connected over RS485.
  - Limelight 3A and the Arducam on one USB hub in the Control Hub's USB 3.0 port.
  - goBILDA Pinpoint (firmware 4) on I2C bus 1, with both pods attached and still.
  - OctoQuad MK2 (firmware 3.1.0) on I2C bus 2, with its pods on ports 3 and 4.
  - No Driver Station connected, and nothing on the motor or servo ports.
- **2026-10-24:** a bare Control Hub (not the 2026-10-09 one) with the same firmware, Android and SDK, on a battery
  that read 14.01–14.35 V: no Expansion Hub, nothing on the USB, I2C, motor or servo ports, no Driver Station. From
  Bench: Thread Lifecycle's deploySloth on, the benches ran from Sloth jars the runner built from the same checkout,
  not from the installed APK. Its planner planned in one call (the sliced search came after); its `WriteCache` is
  this build's. Its Pinpoint localizer added a status check that fires only after a READY and sends no hub command,
  so with no Pinpoint it behaved as the plain Pedro `PinpointLocalizer` this build uses.

Not run on either date (they need `--interactive`):
- Pinpoint, OctoQuad and Limelight unplug/replug;
- odometry moved to the Expansion Hub;
- driving the Gamepad view by hand.

## Still unknown

- **Whether a run over the USB cable (`run_bench.py --usb`) measures what a Wi-Fi run does:** `summary.md`'s `Link:`
  line labels every run, so cable numbers stay out of the Wi-Fi ones above until the default bench, run both ways on
  one hub, shows the same Bench: Hub bus call costs and Bench: Framework Loop loop periods.
- **Where the ball pipeline's slowdown starts, and which step costs it:** time each SnapScript stage (mask,
  distance-transform split, homography, drawing) with 0, 1, 2 and 3 balls in view, well after the pipeline switch,
  reporting the stage times in spare llpython slots.
- **Whether dropping AutoTune removes the 12 s:** build without `com.pedropathing:tuning` (or with `TunerScanner`
  excluded) and time process start to `INIT FINISH` in logcat. AutoTune runs from that artifact
  ([`CLAUDE.md`](../CLAUDE.md), Tuning).
- **Whether a Pinpoint unplugged mid-run reads (0, 0, 0) as the source says, and whether single bad reads happen while
  driving:** run the Pinpoint unplug and replug in `run_bench.py --interactive`. It decides whether a guard stops on
  one non-READY read or on a streak.
- **Whether a rangefinder in SHORT mode keeps its accuracy at a 10 ms timing budget:** log a fixed target's distance
  at 20 and 10 ms budgets and compare the spread and the mean.
- **The sliced route search on a hub:** its longest step, plan time from request to done, the loop period while it
  plans, and STOP during it: run Bench: Hub and Bench: Planner Stop. The estimate above is from a desktop replay.
- **How long `EnhancedMotor` and `EnhancedServo` stay off after a fail-safe that lands just after a write:** both
  dates' single trial landed just before a re-send. Bench: Write Recovery fails safe seven times, 0–235 ms after a
  write, and judges each against `WriteCache`'s re-send period plus 2 loops; run it, and Bench: Write Recovery Photon
  with `--photon`.
- **`GuardedOctoQuadLocalizer` with an OctoQuad plugged in:** read cost, valid reads after the first RUNNING, the
  start pose through START, and a cable pull: run Bench: OctoQuad with `--interactive`.
- **Whether closing the webcam after the stop window works on a hub:** move the close out of `CellTipCamera.stop()`,
  run Bench: Webcam Stop, and check `stop()`'s time and that the next INIT opens the webcam.
