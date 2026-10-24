# Control Hub bench findings

Measured with `scripts/bench/run_bench.py --ports-empty` on 2026-10-09, and on 2026-10-24 on a bare hub with
`scripts/bench/bare_hub.sh`. Rerun the bench after a change that touches any of this, and update this file with the
new numbers. A line that names one date was measured only that day.

**Setup:**
- Both days: REV Control Hub, firmware 1.8.2, Android 7.1.2, FTC SDK 12.0.0, BNO055 IMU, no Driver Station connected.
- 2026-10-09, battery 13.0–13.35 V:
  - REV Expansion Hub at address 2, connected over RS485.
  - Limelight 3A and the Arducam on one USB hub in the Control Hub's USB 3.0 port.
  - goBILDA Pinpoint (firmware 4) on I2C bus 1, with both pods attached and still.
  - OctoQuad MK2 (firmware 3.1.0) on I2C bus 2, with its pods on ports 3 and 4.
  - Nothing on the motor or servo ports.
- 2026-10-24: a second Control Hub, not the 2026-10-09 one, battery 14.0–14.4 V, with nothing plugged into any port
  (USB, I2C, RS485, motors, servos). So the Expansion Hub, Limelight, webcam, Pinpoint and OctoQuad numbers here are
  2026-10-09's. From Bench: Thread Lifecycle's deploySloth on, that run's benches ran from hot-loaded Sloth jars the
  runner built from the same checkout, not from the installed APK.

Times are p50 / p95 / max in ms unless noted. Not run on either day (they need `--interactive`):
- Pinpoint, OctoQuad and Limelight unplug/replug;
- odometry moved to the Expansion Hub;
- driving the Gamepad view by hand.

## Design around these

- **Route planner on the OpMode thread.** Vision Ball Collection, Ball Collection Auto and Ball Collection plan 4
  balls there (`BallCollection.BALLS`; Ball Collection its four slots), one `RouteOptimizer.Search.step()` per loop. A
  step shapes orders for `VisibilityGraphPlanner.routeSearchSliceMs` (20 ms) and overruns it by at most one `shape()`,
  so STOP waits for one step, not a plan. Rerun Bench: Hub and Bench: Planner Stop after any planner change. Both runs
  below planned in one go, before slicing: on the field, with the HIVE rails, for an 18 × 18 in robot with its intake
  9 in ahead (2026-10-09) and the Betta bot's 16 × 15 in with a 15 in wide intake 8 in ahead (2026-10-24):

  | balls | 2026-10-09 | 2026-10-24 |
  |---|---|---|
  | 3, random layouts (60) | 120 / 330 / 437 | 141 / 333 / 465 |
  | 4, random layouts (48) | 794 / 1979 / 2498 | 558 / 1888 / 2434 |
  | 4, on both sides of the HIVE rail (Planner Stop) | 6584 / 6990 / 6990 | 1452 / 1728 / 1728 |
  | 5, random layouts (40) | 6704 / 13669 / 16526 | 1934 / 13067 / 14769 |
  | 5, behind the rail | 10998 / 11148 / 11148 | 4369 / 4450 / 4450 |
  | 5, on both sides of the rail | 40939 / 41134 / 41134 | 875 / 909 / 909 |
  | 5, on both sides of the rail (Planner Stop) | 41805 / 42906 / 42906 | 902 / 969 / 969 |

  - Layout matters as much as ball count: on 2026-10-24 the 5-ball Planner Stop layout planned faster than the 4-ball
    one.
  - The cold first 4-ball plan took 227 ms (2026-10-09) and 2046 ms (2026-10-24), each on its run's first random
    layout, which changes with the robot shape, so it doesn't isolate warm-up. On the fixed 4-ball Planner Stop
    layout, the first plan in a fresh app process took 1581 and 1728 ms against 1343–1529 for later plans.
  - STOP pressed during a plan in one go, judged on the hub's clock:
    - 2026-10-09: 6 of 6 trials restarted the Robot Controller app.
    - 2026-10-24: 1 of 6. STOP landed mid-plan in 5 trials. The ones that survived reached `stop()` at most 744 ms
      after STOP (the SDK's `STOP - OPMODE` 30–777 ms after its `auto-stopping`); the one that restarted reached it
      after 1068 ms.
    - In all 7 restarts the SDK logged `stuck in stop()` 1003–1006 ms after `auto-stopping`, and the robot reported
      `Robot Status: running` again 44.9–45.1 s after STOP. The relaunched app's dashboard answers at about 16 s,
      half a minute before it can run anything. A restart loses dashboard edits and the alliance.
  - 3 balls planned in under 0.47 s in both runs (the worst of 60 random layouts). No 3-ball STOP trial was run.
  - Estimate for the sliced planner, not yet run on a hub, from a desktop replay of the 2026-10-24 layouts that drops
    the same balls as the hub and runs 28–35× faster (median 32×). The longest step: about 110 ms (100–115) for random
    4-ball layouts and about 240 ms (210–260) for the 4-ball Planner Stop layout. From request to done in an
    `EnhancedOpMode`, whose loop runs between slices, a 4-ball plan takes about 1.3× as long as in one go (random
    layouts p50 about 730 ms, worst about 3.3 s). STOP waits about 255 ms (up to about 275) in Vision Ball Collection
    and Ball Collection; in Ball Collection Auto, one step plus the loop's tail plus a webcam close of up to 581 ms
    (below) is about 830 ms (up to about 855), 45–70 ms inside the 900 ms window.
- **The SDK's stop window is 900 ms**, plus 100 ms of grace, covering whatever `init_loop()`/`loop()` is running and
  `stop()`.
  - Webcam close during STOP (2026-10-09): `CellTipCamera.stop()` took 574–581 ms when STOP came right after INIT,
    and 61–319 ms later in the run.
  - A raw `WebcamSession.close()` made 200 ms after opening took 685 ms (2026-10-09).
  - So webcam teardown can use most of the window.
- **A dashboard hardware-config change at idle freezes the app** (slothboard 6165.6, which this build pins).
  - `SET_HARDWARE_CONFIG` from the dashboard with no OpMode running froze the Robot Controller app for 10 s, then the
    app was killed and relaunched, ready 53 s after the switch (2026-10-24). `WRITE_HARDWARE_CONFIG` and
    `DELETE_HARDWARE_CONFIG` ended the same way, the app gone about 10 s after the command, on the 2026-10-01 and
    2026-10-05 runs: 6 of 6 across the three.
  - Cause, from the 2026-10-24 thread dumps and slothboard's source: slothboard runs the robot restart on its
    websocket thread while holding its OpMode lock (`activeOpMode`). The SDK's shutdown waits up to 10 s for the event
    loop thread, whose teardown (`FtcDashboard.onOpModePostStop`) is blocked on that same lock. The SDK's `deadline
    expired waiting for service termination: EventLoop` and the Control Hub watchdog's `Relaunching the RC` both came
    10.0 s after the switch.
  - slothboard 6165.7 restarts after releasing the lock, on the UI thread; not yet run on a hub. The bench runner's
    14 config switches over adb (2026-10-24) never froze.
- **Every app start takes about 38 s** (2026-10-24, the 16 starts whose logs cover it):
  - Process start to `INIT FINISH`: 37.1–38.3 s. Sinister's boot holds the main thread from about 5 s to 25–26 s
    (20.3–21.1 s), 12.1–12.3 s of it Pedro's autotune `TunerScanner`.
  - Every one logged an ANR on `FtcRobotControllerWatchdogService` about 25 s in; none was killed for it. The same ANR
    is in every config switch's log from 2026-10-05 (1) and 2026-10-09 (13). Each one rewrites `/data/anr/traces.txt`.
  - A config switch over adb, as the runner times it: 42.4–44.6 s.
- **deploySloth: wait at least 5 s after it returns before pressing INIT; tell the team 6 s.**
  - The jar loaded 4.3–4.8 s after the push on 2026-10-09; on 2026-10-24 4.74–5.15 s after the push, which was
    4.64–4.83 s after deploySloth returned.
  - INIT 5 s after it returned ran the new code, with 0.51 s to spare on 2026-10-09 and 0.18 s on 2026-10-24.
  - INIT sent the moment the jar was pushed, or right as deploySloth returned, restarted the app (both days).
  - INIT sent 0.5–2 s after it returned didn't report within 30 s (both days).
  - A hot-loaded jar survives an app restart. The OpMode list stayed correct throughout a load.
  - A deploySloth while an OpMode runs is staged and lands after STOP: the OpMode kept running, and the load was
    processed 11.25 s after the push (2026-10-24).
- **After a hub reset or a dropped write, `EnhancedMotor` and `EnhancedServo` come back only on WriteCache's 250 ms
  re-send.** Nothing clears WriteCache on a `failSafe()`, so they skip the unchanged value until the re-send.
  - Raw SDK devices reached the hub again on the next write, about 2 ms after a `failSafe()` (both days).
  - The wrapped ones came back 1.9 ms after it on 2026-10-09 and 21 ms after it (one loop after the raw devices) on
    2026-10-24; both times the failSafe happened to land just before a re-send.
  - By the code, that is up to 250 ms plus a loop with the motor off or the servo limp. Neither run measured the
    long case; Bench: Write Recovery's failSafes at 0–235 ms after a write cover it.
- **Expansion Hub: every command costs more, not just encoders** (2026-10-09). The RS485 hop adds about 0.9 ms to
  every Lynx command sent to the Expansion Hub, and 1.5 ms to a bulk read. That covers:
  - motor and servo writes;
  - analog, digital and encoder reads;
  - every I2C transaction.

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
- **Odometry, measured on the Control Hub (2026-10-09):**

  | | Pinpoint | OctoQuad MK2 |
  |---|---|---|
  | one read | `update()` 5.79 / 6.73 / 9.33 | `readLocalizerData` 3.20 / 3.77 / 8.22 (3.66 / 4.49 / 5.27 before Pedro's localizer was built) |
  | building the localizer | 585 ms | 1666 ms (Pedro waits for RUNNING) |
  | heading drift while still | **6.1° in about 5 s** | 0.01° |

  - The OctoQuad had 0 bad reads in 383 loops. `readLocalizerDataAndAllEncoderData` costs 6.58 / 8.51 / 17.67.
  - Pinpoint drift: the 0.7 in position error that came with it is the heading error times the 6.5 in pod offset. The
    Pinpoint reported `FAULT_BAD_READ` right after `BettaConstants.create`, then READY 71 ms later.
  - Rerun the Pinpoint step with the board untouched before blaming the Pinpoint. If the drift holds, a start pose set
    in INIT won't survive it, and neither would a re-seed at START.
- **A missing Pinpoint raises no fault and reads (0, 0, 0)** (2026-10-24, nothing on I2C bus 1).
  `BettaConstants.create` returned in 571 ms and `update()` ran 4.25 / 5.01 / 19.64 without throwing, but after
  `setPose(56, 8, 90°)` every pose read (0, 0, 0). Nothing in the framework checks for a missing Pinpoint, so an auto
  would follow its paths from the field origin with no warning.
- **The OctoQuad hangs INIT when it's missing.** Pedro's `OctoQuadLocalizer` constructor (measured on 3.0.1 on
  2026-10-09, unchanged in 3.0.2) waits for RUNNING in a loop with no timeout and no STOP check.
  - With nothing on the bus, or an MK1 reporting `FAULT_NO_IMU`, INIT hangs with no message.
  - STOP gets out: the SDK force-stopped the OpMode in about 1.3 s without restarting the app.
  - Pedro's `update()` also silently skips invalid reads (CRC error or not RUNNING) and keeps the last pose.
  - `GuardedOctoQuadLocalizer` with nothing on the bus (2026-10-24): the bench robot built in 75 ms, the `OctoQuad`
    fault came 2979 ms into INIT, INIT kept looping (7.26 / 10.06 / 17.68), and START stopped the OpMode with an error
    naming the OctoQuad. No robot's follower uses it yet, and it hasn't been run with an OctoQuad plugged in.
- **Limelight ball pipeline (pipeline 4): slow** (2026-10-09).
  - About 4.7 new results per second.
  - Pipeline latency (`tl`) 227 / 244 / 254.
  - Capture to arrival on the hub 263 / 284 / 352.
  - Pipeline 0 (fiducial) runs at about 44 fps.
  - New results come 229 / 246 / 266 apart. Ball velocities and "fresh frame" logic have to tolerate about a quarter
    second of age, so `staleFrameSeconds` (0.5 s) counts from a result's arrival on the hub, not its capture.
  - `LimelightSync` takes about 1.7 s, and INIT to the first ball frame about 1.8 s.
  - The poller's rate (100–250 Hz) doesn't change the frame rate. It costs CPU, not loop time: the RC process used 81%
    of a core with the poller off, 124% at 100 Hz, 163% at 200 Hz and 165% at 250 Hz, and loop p50 rose 0.15 ms at
    most.
- **A Limelight that never answers a POST** (2026-10-24: the real Camera against a fake Limelight that serves results
  and holds every POST open):
  - The `Limelight` fault came 15.2 s into INIT, one 15 s POST timeout. INIT kept looping (3.28 / 5.22 / 16.99), and
    after START the loop ran at 267 loops/s.
  - One POST was sent, not retried.
  - STOP during a hung POST isn't held up: the stop pass took 1.2–2.9 ms.
  - The next INIT doesn't wait for it (first INIT loop 142–172 ms), but its sync waits out the stopped run's POST,
    which ended 15.0 s after it was sent, then took about 1.6 s more: done 13.1–13.6 s after INIT, the old POST having
    been sent 3.0–3.5 s before it. Estimate: when the POST was sent just before that STOP, ball vision is up about
    16.6 s after the re-INIT.
  - Two LimelightSync workers, as a Sloth reload would leave, never had POSTs open at once.
- **Webcam controls** (2026-10-09): `WebcamControls.update()` with a write (exposure or gain) blocks for 72 / 87 / 87.
  Write them only in INIT or when they change.
- **Crash reports lag the throw by about half a second** (2026-10-24). The SDK logged `User code threw an uncaught
  exception` 0.43–0.50 s after a hook threw (INIT, `init_loop`, a module's `init()`, `onStart`, `loop`) and 0.02 s
  after a `stop()` threw. The dashboard Error line had it at the bench runner's first or second status read after that.
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
    A 10 ms poller already sees almost every refresh, so polling faster only costs loop time.
  - The poller kept its schedule at every rate (p99 within 0.9 ms of the period); its bulk read took 2.0–2.7 ms
    (p50) under load.
  - `TransferCounter`'s poller on a bare hub (Bench: Thread Lifecycle, 2026-10-24): 100 polls/s, 2 late of 552,
    longest gap 12.5 ms, 2.45 ms per bulk read. `stop()` returned in 0.03–9.0 ms over 16 runs. No poller was left at
    the next INIT after a normal STOP, a STOP from INIT, a crash in `loop()`, a `stop()` that throws, 10 back-to-back
    runs or a Sloth reload.
  - After a crash in `loop()` the OpMode thread ended 537 ms after the throw (the SDK's delay: user code had
    returned), and the poller ended with it. A poller that dies stopped the OpMode thread with its cause 8.2 ms
    (4 loops) later; the SDK reported it about 0.5 s after that.
  - **Never `interrupt()` a thread that sends hub commands** (same bench): of 1176 interrupts landing mid-command, 7
    hit the lock release and abandoned the hub's lock (`#### abandoning lock`). Each held the OpMode's next hub read
    about 505 ms (reads otherwise 2.68 / 3.25 / 4.96). Stop a poller with a flag and `join()`.

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
- The bulk read (about 2.2 ms) counts under `readModules` since the floodgate current read moved into
  `Drivetrain.read()`; it used to count under `gameLoop`.
- Keep Drivetrain current telemetry off outside tuning.
- No Driver Station was connected, so DS frames weren't serialized or sent. A match costs more than these numbers.
- The Limelight fault fallback loop ran at 135 loops/s, 6.79 / 11.05 / 41.87 (2026-10-09), and at 134 loops/s,
  6.89 / 11.00 / 32.42 (2026-10-24).

## Confirmed on the hub (don't "fix" these back)

- **Crashes in an `EnhancedOpMode`** (both days):
  - Every lifecycle stage stops all modules.
  - A second failure is attached as Suppressed.
  - The dashboard Error line shows the crash once the SDK reports it (see the crash-report lag above), and clears at
    the next INIT (seen on 2026-10-24 wherever a status caught that INIT before the run's own crash).
- **Webcam camera-thread exceptions** stop the OpMode and reach the Error line (2026-10-09).
- **Cell-tip frame age** (`captureTimeNanos` against `System.nanoTime()`) runs 6–13 ms, never negative (2026-10-09).
- **Dashboard gamepads** reach both iterative and linear OpModes. A held press gives one edge, and the sticks rest
  500 ms after the last message (498.8 and 501.4 ms on 2026-10-24).
- **Edges through START** (2026-10-24): with LB held from INIT through START, an `EdgeBooleanSupplier` first read after
  START fires on the first running loop. One built and invalidated in INIT gives no edge at START and one per later
  press.
- **Drive hand-back:** Drivetrain gets the motors back from Pedro within 2 loops in every hand-back case (both days).
- **Turret PWM** (2026-10-24, BettaTurret, read from the hub's registers): off at INIT; on at the commanded pulse from
  the first loop (1500 µs centred, 780 µs for TurretR forward); off right after `stop()` and as long as writes are off;
  with writes on, on again the next loop.
- **Hardware view:** a stale Power edit doesn't drive a motor under the next OpMode (both days).
- **Telemetry survives:** a NaN in a field overlay doesn't kill dashboard telemetry (both days).
- **Limelight proxy:** slothboard's proxy keeps query strings. Every Limelight POST was accepted (2026-10-09).
- **Limelight fault:**
  - Raised 456 ms (2026-10-09) and 490 ms (2026-10-24) after INIT.
  - Drawn red on the overlay.
  - Leads every packet.
  - Ball collection ends SKIPPED and hands the Drivetrain back (it didn't hand it back on 2026-10-09).
- **Floodgate:** an unwired floodgate pin reads 0.001 V, so Drivetrain's current limiter stays at 1.0 (both days).
- **Ivy timeout:** `PathCommands.timeout(15000)` ended 15.008 s (2026-10-09) and 15.006 s (2026-10-24) after START.
- **Config baseline:** slothboard's config baseline lists every `@Config` class (both days).
