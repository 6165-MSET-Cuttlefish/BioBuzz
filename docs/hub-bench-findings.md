# Control Hub bench findings

Measured with `scripts/bench/run_bench.py --ports-empty` on 2026-10-09. Rerun the bench after a change that touches
any of this, and update this file with the new numbers.

**Setup:**
- REV Control Hub, firmware 1.8.2, Android 7.1.2, FTC SDK 12.0.0, BNO055 IMU.
- REV Expansion Hub at address 2, connected over RS485.
- Limelight 3A and the Arducam on one USB hub in the Control Hub's USB 3.0 port.
- goBILDA Pinpoint (firmware 4) on I2C bus 1, with both pods attached and still.
- OctoQuad MK2 (firmware 3.1.0) on I2C bus 2, with its pods on ports 3 and 4.
- No Driver Station connected, and nothing on the motor or servo ports.

Times are p50 / p95 / max in ms unless noted. Not run that day (they need `--interactive`):
- Pinpoint, OctoQuad and Limelight unplug/replug;
- odometry moved to the Expansion Hub;
- driving the Gamepad view by hand.

## Design around these

- **Route planner: the worst problem.** It runs on the OpMode thread in Vision Ball Collection and Ball Collection,
  whose `maxBalls` default is 4.
  - Planning time by ball count:

    | balls | p50 | p95 | p99 or worst |
    |---|---|---|---|
    | 3 | 120 | 330 | 437 (p99) |
    | 4, varied layouts | 794 | 1979 | 2498 |
    | 4, on both sides of the HIVE rail | 6584 | | 6990 |
    | 5, varied layouts | 6704 | 13669 | 16526 |
    | 5, on both sides of the rail | about 41000 | | 42906 |

  - The cold first 4-ball plan took 227 ms.
  - STOP pressed during a plan restarted the Robot Controller app in 6 of 6 trials, about 16 s to come back. A restart
    loses dashboard edits and the alliance.
  - The fix isn't chosen yet: plan on a worker thread with a time budget, or cap the ball count. 3 balls stays under
    0.5 s.
- **The SDK's stop window is 900 ms**, plus 100 ms of grace, covering whatever `init_loop()`/`loop()` is running and
  `stop()`.
  - Webcam close during STOP: `CellTipCamera.stop()` took 574–581 ms when STOP came right after INIT, and 61–319 ms
    later in the run.
  - A raw `WebcamSession.close()` made 200 ms after opening took 685 ms.
  - So webcam teardown can use most of the window.
- **deploySloth: wait about 5 s after it returns before pressing INIT.**
  - The jar loads about 4.3–4.8 s after the push.
  - INIT sent the moment the jar was pushed, or right as deploySloth returned, restarted the app.
  - INIT sent 0.5–2 s after it returned didn't report within 30 s.
  - INIT 5 s after ran the new code.
  - A hot-loaded jar survives an app restart.
  - The OpMode list stayed correct throughout a load.
- **Expansion Hub: every command costs more, not just encoders.** The RS485 hop adds about 0.9 ms to every Lynx
  command sent to the Expansion Hub, and 1.5 ms to a bulk read. That covers:
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
- **Odometry, measured on the Control Hub:**

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
- **The OctoQuad hangs INIT when it's missing.** Pedro 3.0.1's `OctoQuadLocalizer` constructor waits for RUNNING in a
  loop with no timeout and no STOP check.
  - With nothing on the bus, or an MK1 reporting `FAULT_NO_IMU`, INIT hangs with no message.
  - STOP gets out: the SDK force-stopped the OpMode in about 1.3 s without restarting the app.
  - Pedro's `update()` also silently skips invalid reads (CRC error or not RUNNING) and keeps the last pose.
  - Using the OctoQuad needs a guarded localizer: a timeout, an error naming the OctoQuad, and a check on every read.
- **Limelight ball pipeline (pipeline 4): slow.**
  - About 4.7 new results per second.
  - Pipeline latency (`tl`) 227 / 244 / 254.
  - Capture to arrival on the hub 263 / 284 / 352.
  - Pipeline 0 (fiducial) runs at about 44 fps.
  - Ball velocities and "fresh frame" logic have to tolerate about a quarter second of age.
  - `LimelightSync` takes about 1.7 s, and INIT to the first ball frame about 1.8 s.
  - The poller's rate (100–250 Hz) doesn't change the frame rate.
- **Webcam controls:** `WebcamControls.update()` with a write (exposure or gain) blocks for 72 / 87 / 87. Write them
  only in INIT or when they change.

## Hub costs (Control Hub, bulk caching off unless noted)

| operation | p50 / p95 / max |
|---|---|
| empty OpMode loop | 1.42 / 1.69 / 6.08 |
| bulk read (clear + first read) | 2.22 / 2.97 / 21.55 |
| cached read after a bulk read | 0.03 / 0.12 / 0.59 |
| `getCurrentPosition` or analog read, uncached | 1.27 / 1.55 / 3.52 |
| `getVoltage` / hub input voltage | 1.30 / 1.56 / 2.94 |
| hub current / motor current | 1.35–1.37 / 1.67–1.81 / 3.05 |
| motor `setPower`, changed value | 1.64 / 1.96 / 6.13 |
| motor `setPower`, same value (SDK skips it) | 0.03 / 0.04 / 5.27 |
| four motor writes | 5.66 / 7.73 / 16.31 |
| servo `setPosition`, changed | 1.55 / 2.02 / 2.90 |
| IMU yaw/pitch/roll (BNO055) | 5.12 / 6.21 / 7.29 |
| dashboard packet GSON | 2.48 / 3.59 / 10.09 (about 5.6 k chars) |
| LLResult parse, ball script result | about 0.8 (`JSONObject`) |
| LLResult parse, 24 tags + 64 detections | 28.6 / 32.1 / 36.2 |
| Limelight POST over USB (`updatePythonInputs` etc.) | 4.4–5.6 / 6.2–7.8 / 10.3 |
| `pipelineSwitch` to a Limelight that hangs | blocks 15 s |

## EnhancedOpMode loop (Betta bench robot, Drivetrain + Pedro, idle follower)

| setting | loop p50 / p95 / max |
|---|---|
| defaults | 6.32 / 9.44 / 25.57 |
| DS telemetry every loop | 8.49 / 11.62 / 28.23 |
| dashboard telemetry off | 4.21 / 7.24 / 25.54 |
| loop profiler off | 5.00 / 8.06 / 14.81 |
| Drivetrain current telemetry on | **12.13 / 16.60 / 26.74** |
| hub current telemetry on | 7.58 / 10.70 / 26.43 |
| following a path | 4.02 / 5.88 / 33.32 |
| Close Flower's loop without the framework, following | 1.03 / 1.33 / 27.76 |

- Telemetry is the biggest piece: `updateTelemetry` takes about 2 ms of a 6 ms loop, and the loop allocates about
  63 kB.
- Keep Drivetrain current telemetry off outside tuning.
- No Driver Station was connected, so DS frames weren't serialized or sent. A match costs more than these numbers.
- The Limelight fault fallback loop ran at 135 loops/s: 6.79 / 11.05 / 41.87.

## Confirmed on the hub (don't "fix" these back)

- **Motor and servo writes:** after a `failSafe()`, `EnhancedMotor` and `EnhancedServo` reach the hub again within
  about 2 ms.
- **Crashes in an `EnhancedOpMode`:**
  - Every lifecycle stage stops all modules.
  - A second failure is attached as Suppressed.
  - The dashboard Error line shows the crash within 0.2 s and clears at the next INIT.
- **Webcam camera-thread exceptions** stop the OpMode and reach the Error line.
- **Cell-tip frame age** (`captureTimeNanos` against `System.nanoTime()`) runs 6–13 ms, never negative.
- **Dashboard gamepads** reach both iterative and linear OpModes. A held press gives one edge, and the sticks rest
  500 ms after the last message.
- **Drive hand-back:** Drivetrain gets the motors back from Pedro within 2 loops in every hand-back case.
- **Hardware view:** a stale Power edit doesn't drive a motor under the next OpMode.
- **Telemetry survives:** a NaN in a field overlay doesn't kill dashboard telemetry.
- **Limelight proxy:** slothboard's proxy keeps query strings. Every Limelight POST was accepted.
- **Limelight fault:**
  - Raised within 0.5 s of INIT.
  - Drawn red on the overlay.
  - Leads every packet.
  - Ball collection ends SKIPPED.
- **Floodgate:** an unwired floodgate pin reads 0.001 V, so Drivetrain's current limiter stays at 1.0.
- **Ivy timeout:** `PathCommands.timeout(15000)` ended 15.008 s after START.
- **Config baseline:** slothboard's config baseline lists every `@Config` class.
