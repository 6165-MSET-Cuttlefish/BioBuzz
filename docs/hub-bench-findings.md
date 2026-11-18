# Control Hub bench findings

What the Control Hub costs per command, and how it behaves at STOP, on a crash, on an app restart and on deploySloth.
Read it before changing the planner, hub or I2C traffic, odometry, the Limelight, background threads or stop paths;
after such a change, rerun `scripts/bench/run_bench.py --ports-empty` (on a bare hub, `scripts/bench/bare_hub.sh`) and
update the numbers here.

Java paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Times are p50 / p95 / max in ms unless
noted. Numbers without a date are from 2026-10-09; the hardware for each run is under [Setup](#setup).

## Design around these

- **Route planner: one `RouteOptimizer.Search.step()` per loop. Its longest step on a hub is 325 ms. Bench: Planner
  Stop's webcam trials, which closed the webcam inside `stop()` as Ball Collection Auto then did, took up to 839 ms
  of the 900 ms window in 20 trials, and that worst case, added up from the measured parts, was over the window.
  Main now leaves the webcam close to EasyOpenCV after the OpMode, outside the window (from the code). That close was
  tried on a hub on 2026-11-17 at 18:50, in the bench build: `stop()` took 0.26–2.3 ms, the stop pass at most 26 ms,
  the next INIT saw a frame in about 1 s, nothing restarted (five STOPs, no planner). Main's own change hasn't run
  on a hub: don't trust a STOP during Ball Collection Auto's 4-ball plan until Bench: Planner Stop's webcam trials
  pass on main's build.** Ball
  Collection Auto, Vision Ball Collection and Ball Collection plan 4 balls (`BallCollection.BALLS`) on the OpMode thread, one step per loop. A step shapes visit
  orders for `VisibilityGraphPlanner.routeSearchSliceMs` (20 ms) and overruns it by at most one `shape()` call
  (`architecture/planner/RouteOptimizer.java:143-154`), so STOP waits for one step, not a plan
  ([`ball-planner-findings.md`](ball-planner-findings.md)).
  - The sliced search on a hub (Bench: Hub's planner phase, 2026-11-14, over USB, on the 2026-10-24 hub; that build's
    planner code is main's):

    | | p50 | p95 | max | n |
    |---|---|---|---|---|
    | every step | 45.7 | 81.5 | 273.4 | 6186 |
    | time past the 20 ms slice | 25.7 | 61.5 | 253.4 | 6186 |
    | longest step of a plan, 4 balls, random | 45.3 | 87.1 | 92.5 | 48 |
    | longest step of a plan, 4 balls on both sides of the rail (Planner Stop's layout) | 263.6 | 273.4 | 273.4 | 5 |
    | longest step of a plan, 5 balls, random | 76.6 | 107.8 | 131.4 | 40 |
    | plan, request to done, 4 balls, random | 588 | 1980 | 2583 | 48 |
    | plan, request to done, 4 balls on both sides of the rail | 1423 | 1448 | 1448 | 5 |

    - The desktop estimate (below) was right for random layouts (about 110 ms) and low for Planner Stop's layout
      (about 240 ms, 210–260): the hub's 263–273 ms is above its top, and Bench: Planner Stop's 290–321 ms (below)
      further above.
    - Plan times are close to planning in one call (table below). This bench's loop does almost nothing between
      steps: its period was 47.6 / 83.8 / 277.2, about 2 ms over the step. The 1.3× estimate for an
      `EnhancedOpMode` loop is untested.
    - The first 4-ball plan in a fresh app process took 2183 ms, longest step 124 ms.
    - It allocated 7.0 GB in 303 s, 1.14 MB a loop: 809 collections, none blocking, the collector thread using 14.6% of
      a core.
  - STOP during the sliced search (Bench: Planner Stop, 2026-11-14, a later run on the same hub over USB, no webcam:
    the 4-ball layout on both sides of the rail and a 5-ball one, replanning back to back in INIT as Vision Ball
    Collection does, STOP sent 0.05, 0.3 or 0.6 s after the runner saw a plan end, 9 trials each):
    - No STOP restarted the app. The hub logged STOP to idle in 29–282 ms with 4 balls and 41–107 ms with 5.
    - The longest step of each 4-ball trial was 289.8–321.4 ms, over the bench's 300 ms limit in 4 of 9 (its check
      FAILs); steps ran 66–71 ms at p50. With 5 balls the longest was 93.6–107.8 ms.
    - Plans from request to done: 4 balls 1527 / 1655 / 1767 (n=36), 5 balls 952 / 1013 / 1016 (n=37).
    - The unsliced control, each 4-ball plan in one step (longest 1448–1673 ms), overran the window on 3 of 3 STOPs:
      two restarted the app, the third reached idle 1030 ms after STOP.
  - The same again, now with the webcam trials (Bench: Planner Stop, 2026-11-17, the same hub over USB, an Arducam
    OV9782 straight in the Control Hub's USB 3.0 port; [Setup](#setup)). Measured:
    - No sliced STOP restarted the app. The hub logged STOP to idle in 29–111 ms with 4 balls and 31–118 ms with 5.
    - The longest step of each 4-ball trial was 292.3–325.4 ms, over 300 ms in 7 of 9 (the check FAILs again); steps
      ran 68.9–70.6 ms at p50. With 5 balls the longest was 91.7–113.1 ms.
    - Plans from request to done: 4 balls 1577 / 1759 / 1929 (n=36), 5 balls 991 / 1071 / 1089 (n=36), 3–9% slower
      than on 2026-11-14.
    - The unsliced control (longest step 1483–1728 ms) overran on 3 of 3 STOPs, and all three restarted the app.
    - With the webcam open (`CellTipCamera` as Ball Collection Auto's robot opens it, 20 trials): no restart, every
      STOP idle inside the window. The slowest took 839 ms, 61 ms inside it: STOP right after INIT, 56 ms waiting for
      a slice, 16 ms of loop tail, then a 729 ms webcam close. The case the bench aims for, START while the webcam is
      still opening and STOP 0.05 s later, landed in a slice in 3 of 5 trials: 771–782 ms to idle, each with 32–44 ms
      of slice and a 667–683 ms close.
    - The webcam close in `stop()` took 50–137 ms while replanning with frames coming in, 437–654 ms with STOP 0.3 s
      after INIT, and 667–729 ms with STOP right after INIT or START when it landed in a plan (45–52 ms when it landed
      between plans). Later that evening Bench: Webcam Close measured a close in `stop()` of 783 ms with STOP 0.6 s
      into INIT, before the first frame, more than 729, and 90–151 ms at its other four STOPs (below).
    - The bench build's `CellTipCamera` closed the webcam in `stop()` as main's did then (from the code: its close
      option was left at the default, a plain `close()`); its planner code differs from main's only in comments.
  - What it means for STOP: in Vision Ball Collection and Ball Collection, which close no webcam, the longest step
    plus the loop's tail is about 350 ms, well inside the window (inferred). For Ball Collection Auto with the close
    inside `stop()`, no webcam trial met the longest step and the slowest close at once: 325 ms of step, about 30 ms
    of tail (the most measured) and a 783 ms close add up to about 1138 ms, over the window (inferred). The trials
    passed because the slow closes came with planner waits of 56 ms or less. The earlier estimates were too low:
    about 917 ms on this page (321 + 15 + 581), 830–855 ms in the bench's old texts. With the close after the window,
    as main now does, it is 325 + 30 + at most 2.3 ms of `stop()`, about 360 ms, or about 380 ms counting the whole
    26 ms stop pass (inferred from the 18:50 run's postStop STOPs). That fix was tried on a hub on 2026-11-17 in
    Bench: Webcam Close (below), not yet with the planner running.
  - The estimate (simulated: a desktop replay of the 2026-10-24 layouts that drops the same balls as the hub and ran
    28–35× faster, median 32×, scaled back by that): the longest step about 110 ms (100–115) for random 4-ball
    layouts and about 240 ms (210–260) for the 4-ball Planner Stop layout; in an `EnhancedOpMode`, a 4-ball plan
    about 1.3× as long as in one call (random layouts p50 about 730 ms, worst about 3.3 s).
  - Why it's slow (read from source): it bounds every order of the full ball set (n!) and shapes them best bound
    first, dropping to n − 1 balls only when no full order is drivable
    (`architecture/planner/RouteOptimizer.java:171-214`). One awkward ball makes it shape every full order first, so
    the layout matters as much as the count.
  - Planning time in one call on 2026-10-24, for the 16 × 15 in robot with its intake 8 in ahead that the code uses
    now (the Betta bot's, retired 2026-11-13; still in `RobotGeometry`), in one half of the field with the HIVE rails and 1 in gaps (`opmodes/test/bench/PlannerWorkload.java`; the
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
    OpMode could run until 44.9 s after it (`Robot Status: running` in logcat). The unsliced controls' restarts took
    longer: 47.6–47.7 s (2 restarts, 2026-11-14) and 47.7–48.0 s (3 restarts, 2026-11-17) from STOP to that line, on
    the bigger bench build.
- **A missing or faulting Pinpoint raises nothing.** The BIOBUZZ robot has no Pinpoint this year; if one goes back
  on, before any Pedro-driven run check that the status telemetry's
  `Robot Position` line reads the start pose after INIT: (0, 0, 0) means the Pinpoint isn't answering.
  - Missing (2026-10-24, nothing on I2C bus 1): a follower on Pedro's `PinpointLocalizer` with the values in
    `opmodes/test/bench/PinpointSetup` was built in 571 ms with no error,
    `follower.update()` ran 4.25 / 5.01 / 19.64 (n=200) with no error, and after `follower.setPose(56, 8, 90°)` every
    sampled pose read (0, 0, 0) (`opmodes/test/bench/PinpointAbsentBench.java`). `hardwareMap.get` only needs the
    config to name the device. With no device version read, SDK 12's `GoBildaPinpointDriver.update()` reads nothing
    (read from source), so the pose stays at (0, 0, 0) however the robot moves, and Pedro drives without ever seeing
    progress.
  - Missing again (2026-11-14, over USB, the same hub, nothing on I2C bus 1): the same result. The follower built in
    578 ms, `follower.update()` ran 4.09 / 5.61 / 21.82 (n=200), no error and no fault, and every sampled pose (one per
    50 updates) read (0, 0, 0) after `setPose(56, 8, 90°)`. Logcat showed the SDK's reads of the Pinpoint getting
    nothing back (`readStatusQuery: cbExpected=5 cbRead=0`, 201 times). The bench's follower is a plain
    `PinpointLocalizer` (`opmodes/test/bench/PinpointSetup.java`); the robot's follower reads the OctoQuad instead.
  - Faulting (read from source; not yet seen on a hub): a bad read keeps the previous pose, a failed one snaps it to
    (0, 0, 0), and Pedro never reads the status: [`sdk-hub-findings.md`](sdk-hub-findings.md#device-drivers). No
    robot OpMode shows the Pinpoint's status; Bench: Pinpoint does.
  - A guard, like `GuardedOctoQuadLocalizer` for the OctoQuad (below), must check at INIT that the Pinpoint answers
    and stop with an error naming it and its port, then check the status on every read, counting non-READY only after
    the first READY. The Pinpoint reported `FAULT_BAD_READ` right after the follower was built, then READY 71 ms
    later.
- **Every app start takes 37–41 s; budget 45 s for a config switch and 50 s for a STOP-time app restart at a match**
  (the restarts after STOP above took up to 48.0 s).
  On 14 config switches on a bare hub (2026-10-24), process start to `INIT FINISH` in logcat took 37.1–38.1 s; on 9
  on the same hub with a bigger build (2026-11-14, 127 OpModes against 60), 38.8–39.4 s.
  - A config switch over adb, timed by the bench runner from the switch to the dashboard reporting the robot ready,
    took 42.4–44.6 s on 2026-10-24. From the switch to the runner's dashboard reconnect, on its whole-second log
    times: 40–42 s (14 switches, 2026-10-24, over Wi-Fi), 42–43 s (10 switches, 2026-11-14, over USB). Each switch
    worked first time on both dates.
  - 12.1–12.3 s of it is one class scanner, Pedro AutoTune's `com.pedropathing.tuning.autotune.TunerScanner`
    (`com.pedropathing:tuning:1.0.1`, `TeamCode/build.gradle:86`), which calls `getDeclaredMethods()` on each class it
    is handed. Sinister, the class-scanning library under Sloth, runs it at app start, and the main thread waits on
    Sinister's scanners (`running scanners for load`) until it ends.
  - Every start logs an ANR (Android's "application not responding" report), `ANR in com.qualcomm.ftcrobotcontroller`
    (reason: executing a service, `FtcRobotControllerWatchdogService` in 26 of 27), about 26 s in: 13 of 13 starts on
    2026-10-09, 14 of 14 on 2026-10-24, 9 of 9 config switches on 2026-11-14. The same process went on to finish INIT
    every time, so it isn't a crash.
- **The SDK's stop window is 900 ms** (`OpModeInternal.MS_BEFORE_FORCE_STOP_AFTER_STOP_REQUESTED`), plus 100 ms of
  grace, covering whatever `init_loop()`/`loop()` is running and `stop()`. Overrunning it restarts the app.
  - In the 100 ms of grace every hub command throws `ForceStopException`, a `finally`'s too, so a loop still running
    at 900 ms can't put hardware state back (read from source: SDK 12.0.0 `OpModeStuckCodeMonitor` calls
    `setThrowOnNetworkLockAcquisition(true)`; `MessageKeyedLock.acquire` throws). An iterative OpMode sees STOP
    mid-loop only through STOP's gate, `OpModeManagerImpl.shouldPreventDangerousHardwareAccess()`, raised as STOP
    begins: Bench: OctoQuad's bank phase checks it before each read.
  - Webcam close during STOP, when `stop()` closed it (main before this change): `CellTipCamera.stop()` took
    574–581 ms when STOP came right after INIT, and 61–319 ms later in the run. On 2026-11-17 it took up to 729 ms
    right after INIT or START (Bench: Planner Stop's webcam trials, above), and 783 ms with STOP 0.6 s into INIT,
    before the first frame (Bench: Webcam Close, below).
  - A raw `WebcamSession.close()` made 200 ms after opening took 685 ms.
  - So webcam teardown can use most of the window.
  - It doesn't have to be in the window (from the source, and one hub run on 2026-11-17). SDK 12's
    `OpModeManagerImpl.callActiveOpModeStop` fail-safes the hubs, runs `stop()` under the 900 ms watchdog, then
    calls the post-stop listeners with no watchdog, on the event loop thread. EasyOpenCV 1.7.3 (SDK 12 Vision's
    dependency) registers every camera when it's built and closes it from that post-stop call (`OpenCvCameraBase`).
    The SDK starts the next INIT only after that call returns, so the next OpMode never opens the webcam while the
    old close runs.
  - What main does now (from the code): `CellTipCamera.stop()` (`biobuzz/modules/CellTipCamera.java:88-94`) calls
    `WebcamSession.closeAfterOpMode()`, which ends the session (the next `update()` throws) and stops the dashboard
    stream, and leaves the device to EasyOpenCV. `CellTipPipeline.release()` stays in `stop()`: it frees only the
    native AprilTag detector, under the lock `processFrame` holds, so a frame that still arrives skips detection
    (`biobuzz/modules/vision/CellTipPipeline.java:90-91` and `:157-163`). `WebcamSession.close()` still closes at
    once, for code that must free the device before the OpMode ends (Bench: Webcam). CameraTune, which never
    reopens the webcam, leaves its close to EasyOpenCV too.
  - Measured on a hub: Bench: Webcam Close, 2026-11-17, 18:50 to 18:56, in the bench build, whose `CellTipCamera`
    picks a close by its `closeMode`: in `stop()` (sync), on another thread with `closeCameraDeviceAsync` (async), or
    left to EasyOpenCV after the OpMode (postStop, what main now does). STOP at INIT + 0, 0.3, 1 and 2.5 s and 3 s
    after START, each followed at once by another INIT. Five STOPs per mode, once each:

    | | sync | async | postStop |
    |---|---|---|---|
    | `CellTipCamera.stop()` (ms) | 90–783 | 0.8–4.3 | 0.26–2.3 |
    | stop pass, first module's `stop()` to `onEnd()` (ms) | 112–814 | 21–41.5 | 19–26 |
    | hub STOP to '***STOP - OPMODE***' (ms) | 122–831 | 68–507 | 72–495 |
    | next INIT, `createRobot()` to the first fresh verdict (ms) | 971–1060 | 964–992 | 950–978 |

    - No STOP restarted the app or left a force-stop or stuck marker, in any mode.
    - The 783 ms and the 495–507 ms came from the STOP 0.6 s into INIT, before the first frame.
    - The hub's STOP to '***STOP - OPMODE***' still holds the close: the SDK logs that line after the post-stop
      listeners (from the code). The close moved past `stop()`, it didn't vanish (inferred). With async, EasyOpenCV's
      own post-stop close waits for the async thread's lock, so it holds the event loop too (from the code).
    - Five STOPs can't separate async from postStop. postStop's `stop()` was shorter and needs no extra thread, so
      main uses it.
    - It doesn't show: more than five STOPs per mode; a STOP while the planner runs; main's code (the bench build's); how long
      the close after the OpMode takes on its own (that build logs no `onEnd()` line); that the post-stop call really
      has no watchdog (still from the source); Ball Collection Auto itself.
  - The close after the OpMode has no watchdog (from the code). EasyOpenCV's `stopStreaming()` waits in a loop for a
    first frame and ignores interrupts (`OpenCvWebcamImpl`), so a webcam that started streaming and never sent a
    frame, e.g. unplugged right then, would hang the event loop with no app restart: no INIT works until Restart
    Robot. Not seen on a hub.
- **deploySloth: wait for the `Processed Sloth Load` toast before INIT ([`CLAUDE.md`](../CLAUDE.md), Build &
  deploy); without the toast, allow 12 s, not 5 or 6.** A hot load took about 9 s on 2026-11-14, and INIT sent during
  it restarts the app or is dropped.
  - Push to `Processed Sloth Load`: 4.3–4.8 s (2026-10-09, 52 OpModes, another hub), 4.7–5.2 s (2026-10-24, 60
    OpModes), 9.47, 8.98 and 9.09 s (2026-11-14, 127 OpModes, the 2026-10-24 hub). On the same hub the load time
    nearly doubled as the OpMode count more than doubled. That is a correlation, not a proven cause: the build
    changed in every other way too. The push comes about 0.1–0.2 s before deploySloth returns (2026-10-09 and
    2026-10-24).
  - INIT sent the moment the jar was pushed, or right as deploySloth returned, restarted the app (all three dates).
    On 2026-11-14 INIT 0.5 s after it returned did too, where on the earlier dates it was dropped. The SDK logged
    `OpMode '$Stop$Robot$' stuck in stop(). Restarting robot controller app.` 1.1–1.4 s after `Staged Sloth Load`;
    its thread dump shows the OpMode thread inside Sinister's unload (`SinisterImpl.scanUnload`), called from Sloth's
    own OpMode's `loop()` (`OnBotLoadEventHandler$OpMode.loop`).
  - INIT sent 1, 2 or 5 s after it returned was dropped on 2026-11-14 (logcat `initOpMode(): Was unable to find
    metadata for OpMode Bench: Sloth Probe`): no restart, and nothing ran. The 5 s INIT reached the hub 4.0 s before
    the load ended. On 2026-10-09 and 2026-10-24, INIT 0.5–2 s after was dropped the same way, and INIT 5 s after ran
    the new code; on 2026-10-24 the load had ended only about 0.2 s before.
  - So the 12 s is the longest load measured (9.5 s, three loads) plus margin. Not yet measured: an INIT sent after
    a load this long has ended (no run on 2026-11-14 sent one), and how long a load takes on main's build, which has
    77 `@TeleOp`/`@Autonomous` annotations against the bench build's 127 (counted in source, 2026-11-14).
  - A deploySloth sent while an OpMode runs leaves it running; the load is staged and lands after STOP, processed
    11.25 s (2026-10-24) and 15.4 s (2026-11-14) after the push, both counting the wait for STOP.
  - A hot-loaded jar survives an app restart. On 2026-11-14, after the restart at the push, the new process logged
    `sloth upload is newer`, then `Processed Sloth Load` during start-up; that start took 40.8 s to `INIT FINISH`.
  - The OpMode list stayed correct throughout a load (all three dates).
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
    in INIT won't survive it, and neither would a re-seed at START. Test: Bench: Pinpoint's soak
    (`python3 scripts/bench/run_bench.py --pinpoint-soak --ports-empty`, an hour untouched) judges the heading over the
    first 5 s and 5-120 s against the OctoQuad's beside it, and times a recalibration.
- **Pedro's `OctoQuadLocalizer` hangs INIT when the OctoQuad is missing: give a follower that uses the OctoQuad
  `GuardedOctoQuadLocalizer` (`architecture/hardware/`) instead.** The BIOBUZZ robot's follower uses the guarded one
  (`BioBuzzConstants.localizer`, [`odometry-tuning.md`](odometry-tuning.md)), as do the OctoQuad benches. Pedro's constructor (measured on 3.0.1 on 2026-10-09, unchanged in 3.0.2)
  waits for RUNNING in a loop with no timeout and no STOP check.
  - With nothing on the bus, or an MK1 reporting `FAULT_NO_IMU`, INIT hangs with no message.
  - STOP gets out: the SDK force-stopped the OpMode in about 1.3 s without restarting the app.
  - Pedro's `update()` also silently skips invalid reads (CRC error or not RUNNING) and keeps the last pose.
  - The vendored `pedro/procedures/OctoQuadTuner` builds Pedro's localizer, so it hangs INIT the same way.
  - What the guard does (read from `architecture/hardware/GuardedOctoQuadLocalizer.java` and
    `architecture/core/EnhancedOpMode.java:186-187` on 2026-11-12): it never waits in INIT, and raises the `OctoQuad`
    fault when the OctoQuad isn't RUNNING 3 s after its last reset or reports `FAULT_NO_IMU` on 3 reads in a row.
    `EnhancedOpMode`'s START calls `awaitRunning()`, which waits for RUNNING until those 3 s are up, then throws
    naming the problem; an OpMode that isn't an `EnhancedOpMode` calls `awaitRunning()` itself before driving. Once
    RUNNING, invalid reads for more than 250 ms in a row throw. A pose set before RUNNING is written to the OctoQuad at
    RUNNING.
  - `GuardedOctoQuadLocalizer` with nothing on the bus (Bench: OctoQuad Absent, 2026-10-24): the bench robot built in
    75 ms, the `OctoQuad` fault came 2979 ms into INIT, INIT kept looping (7.26 / 10.06 / 17.68), and START stopped the
    OpMode with an error naming the OctoQuad. The same on 2026-11-14 over USB: built in 78 ms, the fault 2982 ms into
    INIT, INIT 7.15 / 10.14 / 18.60, the START error naming the OctoQuad. It hasn't run with an OctoQuad plugged in.
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
- **Analog refresh and poller threads** (Bench: Analog Poll, run by hand with `laser_rangefinder.xml`, a rig config
  deleted on 2026-11-14 whose `analog0` and `analog1` were Control Hub analog ports 1 and 3: 10 ms on
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

## EnhancedOpMode loop (bench robot on the Betta bot's drive constants, Drivetrain + Pedro, idle follower)

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
- **Telemetry survives:** a NaN in a field overlay doesn't kill dashboard telemetry (again on 2026-11-14).
- **Limelight proxy:** slothboard's proxy keeps query strings. Every Limelight POST was accepted.
- **Limelight fault:**
  - Raised 456 ms (2026-10-09) and 490 ms (2026-10-24) after INIT.
  - Drawn red on the overlay.
  - Leads every packet.
  - Ball collection ends SKIPPED and hands the Drivetrain back (2026-10-24).
- **Floodgate:** an unwired floodgate pin reads 0.001 V, so Drivetrain's current limiter stays at 1.0.
- **Ivy timeout:** `PathCommands.timeout(15000)` ended 15.008 s (2026-10-09) and 15.006 s (2026-10-24) after START.
- **Config baseline:** slothboard's config baseline lists every `@Config` class.
- **Outputs come back after a hub fail-safe** (Bench: Write Recovery, 2026-11-14, over USB, 7 fail-safes): `EnhancedMotor`
  and `EnhancedServo` reached the hub again within 273.3 ms at worst, against a limit of 287.2 ms (`WriteCache`'s
  250 ms re-send period plus 2 loops at the run's 18.6 ms p95). A raw motor and a raw servo were back within one loop
  (41.5 ms at worst). None stayed off.
- **The same with Photon on** (Bench: Write Recovery Photon, 2026-11-17, over USB, 7 fail-safes): `EnhancedMotor` and
  `EnhancedServo` reached the hub again within 271.9 ms at worst, against a limit of 288.9 ms, at the same moment after
  every fail-safe, so the wrapped motor no longer lags the servo. The raw motor and servo were back within one loop
  (38.6 ms at worst) ([`photon-bench.md`](photon-bench.md)).
- **`adb shell getprop ro.serialno` names a Control Hub** (2026-11-14, over USB): the session read the hub's serial
  from it at its start, so `scripts/bench/session.py` can tell one hub from another by it.

## Known problems in the bench code

- **The deploySloth race's INIT delays are now too short to run the hot-loaded probe, so "the hot-loaded jar survives
  an app restart" FAILs without testing anything.** Its last INIT goes 5 s after deploySloth returns
  (`scripts/bench/run_bench.py:10524`), which landed mid-load on 2026-11-14 (above), and with no probe run from a hot
  jar the check reports False instead of not judged (`:10712-10713`). The jar did survive the restart in that run's
  logcat. Proposed fix (written, not on main yet): a seventh deploySloth that INITs once `Processed Sloth Load` is in
  logcat, with its own check that FAILs if that INIT restarts the app, gets no report or runs the APK's code, and the
  restart check reported as not judged when no probe ran from a hot jar. Delete this entry when it lands.

- **Bench: Limelight Fault can hold STOP past the 900 ms window in its replug step.** Its `onEnd()` puts the
  Limelight back on the runner's pipeline with `ll.pipelineSwitch(...)` on the OpMode thread during STOP
  (`opmodes/test/bench/LimelightFaultBench.java:330-343`), a POST that blocks up to 15 s on a hung Limelight
  ([Hub costs](#hub-costs-control-hub-bulk-caching-off-unless-noted)) and that [`CLAUDE.md`](../CLAUDE.md)
  (Conventions) keeps to `LimelightSync`'s thread. A Limelight that answered a poll in the last second but hangs on
  that POST would hold STOP past the window and restart the app. It runs only when the runner passes
  `restorePipeline`, which it does only in the interactive replug step (`scripts/bench/run_bench.py:7063`), not run
  on either date. Read from source, 2026-11-11; not seen on a hub, not fixed. Proposed fix: hand the restore to
  `LimelightSync`'s thread, or, as a bench-only exception to that rule, send it from a one-shot daemon thread; wait at
  most 300 ms for it and report it as pending at STOP otherwise; dropping the restore isn't a fix, since the next
  synced INIT selects the ball pipeline, not the one the runner saved.

## Setup

Runs of `scripts/bench/run_bench.py --ports-empty`, the 2026-10-24 one through `scripts/bench/bare_hub.sh`, the
2026-11-14 one through `scripts/bench/session.py --usb --install`, then `run_bench.py --usb` by itself for the benches
that session didn't reach, and on 2026-11-17 `run_bench.py --usb` by itself:

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
- **2026-11-14:** the 2026-10-24 hub, same firmware (1.8.2), Android and SDK, bare, on a battery that read
  14.11 V falling to 13.64 V, reached over a USB cable from a Mac (`--usb`), the hub's first bench run that way. The
  build was a bench branch with 127 OpModes; its planner code and its plain Pedro Pinpoint follower were main's then.
  - What worked over the cable: the session backed up the hub's app (its APK, configs and Sloth folder), installed
    the bench build and started the app, up about 42 s after launch; the probe (about 2 min); 11 config switches over
    adb, each first time; one `adb forward` to the dashboard port, made at the default suite's start, still carrying
    the dashboard after 8 config switches and 3 app restarts. Against the 2026-10-24 Wi-Fi run on this hub, Bench:
    Hub's bus costs matched within about 0.1 ms at p50 (bulk read 2.23 against 2.34, four motor writes 5.67 against
    5.69), its IMU reads within 0.25 ms (yaw/pitch/roll 5.49 against 5.26), and Bench: Framework Loop's settings within
    0.5 ms (defaults 6.33 against 6.34).
  - It stopped 27 minutes into the default suite, when the Mac restarted (out of memory, as reported by the person
    running it). The runner's text logs stop at 11:32:10, mid config switch, but its other files show the switch to
    `bench_octoquad_absent` finished (`INIT FINISH` in that switch's logcat, saved at 11:32:54) and Bench: OctoQuad
    Absent had started: its parameters were pushed at 11:32:55, and the power log runs to 11:32:52. Not run: Bench:
    OctoQuad Absent (interrupted), Webcam Stop with no webcam, Planner Stop, Overlay NaN, Config by name and Freeze
    Dump, then the session's Planner Stop with its control and Dashboard Off, and its put-back. What a Mac running a
    session needs: [`toolchain-findings.md`](toolchain-findings.md#laptop-and-adb).
  - After the restart, the same hub over USB, with no new install: `run_bench.py --usb --ports-empty --octoquad
    --absent --planner-stop --overlay-nan --config-by-name --freeze-dump`, twice. At 12:07 it stopped at once:
    Bench: OctoQuad Absent was still in INIT, in the app process whose `INIT FINISH` the session's last config switch
    logged, so the session had INITed it and the OpMode had held there since. The runner refuses to start while an
    OpMode is active; by 12:10 the app had been restarted over adb by hand, and the second run did all six benches.
    Bench: OctoQuad skipped its checks (no OctoQuad answered). Results: Bench: Planner Stop under Route planner above;
    OctoQuad Absent under the OctoQuad; Config by name in [`toolchain-findings.md`](toolchain-findings.md#deploying);
    Freeze Dump in [`dashboard-findings.md`](dashboard-findings.md); Webcam Stop with no webcam and Overlay NaN passed.
    That run took `bench_octoquad_absent` as the hub's own config and left the hub on it again.
  - Then Bench: Dashboard Off by itself, `run_bench.py --usb --ports-empty --dashboard-off` at 12:33: its switch, app
    restart, reboot and recovery checks passed
    ([`dashboard-findings.md`](dashboard-findings.md#enabledisable-dashboard)); its reinstall step hung in `adb install`
    for the runner's 600 s timeout, so its three reinstall checks read FAIL as not run
    ([`toolchain-findings.md`](toolchain-findings.md#laptop-and-adb)). It left `bench_octoquad_absent` active too. None
    of these runs did the session's put-back.
- **2026-11-17:** the 2026-11-14 hub (same firmware, 1.8.2), over a USB cable from a Mac, on one 12 V battery that
  read 13.11 V at the first run's start and 12.79 V when the second run's benches ended (12.74 V at 18:11, its
  put-back; the runner logs voltage only, no current). No new install: the bench build of 2026-11-14 was still on the
  hub. Its robot code is older than main's; its planner code matches main's apart from comments, and its Photon
  sources are main's. The runs:
  - `run_bench.py --usb --planner-stop --ports-empty`, 16:23 to 16:34: an Arducam OV9782 plugged straight into the
    Control Hub's USB 3.0 port, no USB hub and no power bank, nothing else plugged in. Results under Route planner
    above. It put the hub's config back and removed its adb forward.
  - The Photon USB block, `run_bench.py --usb --photon --usb-expansion --photon-i2c --ports-empty`, 16:58 to 18:12:
    an Expansion Hub powered from the Control Hub's second XT30 port and linked only by a mini-USB to USB-A cable (no
    RS485), a goBILDA Pinpoint (no pods) on its I2C port 1, the OctoQuad (no pods) on Control Hub I2C port 2, nothing
    on any motor or servo port. Results in [`photon-bench.md`](photon-bench.md). The benches ended at 17:44; the run
    then waited at its last prompt (put the Pinpoint back) until 18:10, put the hub's config back
    (`laser_rangefinder_i2c`), deleted the bench configs and removed its adb forward.
  - Later, `run_bench.py --usb --webcam-close --ports-empty`, 18:50 to 18:56: Bench: Webcam Close, the Arducam
    straight in the Control Hub's USB 3.0 port, the battery at 12.69 V at its start and 12.67 V at its end. The
    bench build on the hub had `CellTipCamera`'s three close modes (each result names its mode). Results under The
    SDK's stop window above.
  - The hub's logcat clock ran about 10 minutes ahead of the Mac's (the runner's log at 17:33 quotes hub lines from
    17:43): line up the two by events, not by time.
  - Expected noise (inferred): with `laser_rangefinder_i2c` active and no laser on I2C bus 1, the SDK logs I2C errors
    for `Laser` at each app start; and one ANR per app start, as under [Design around these](#design-around-these).

Not run on any of these dates (they need `--interactive`):
- Pinpoint, OctoQuad and Limelight unplug/replug;
- odometry moved to the Expansion Hub;
- driving the Gamepad view by hand.

## Still unknown

- **Each rangefinder's own analog update rate at its 10 ms budget, and whether analog0's 4-2926 mV swing of 2026-10-17
  was the sensor or its cable:** Bench: Analog Poll's still targets can't show the first (hub ADC noise changes the
  reading at every refresh), and nothing has re-run the second. Test: Bench: Rangefinders, `python3
  scripts/bench/run_bench.py --rangefinders --interactive --ports-empty`: a fast target's level steps with a 20 ms
  control on sensor A, and the still, flex and swap phases
  ([`scripts/bench/README.md`](../scripts/bench/README.md#bench-rangefinders)).
- **Whether adb's server reconnects a dropped `adb connect` device by itself:** its help text (platform-tools 37.0.1)
  doesn't say. If it does, over Wi-Fi an adb command a runner sends between two `ensure_connected` calls could reach
  another hub the Mac joined, before the next call reads `ro.serialno` (`--hub-serialno`, which `session.py` passes) and
  stops the run. Test: `adb connect 192.168.43.1:5555`, power-cycle the hub, and run `adb devices` every few seconds
  without `adb connect`: the hub coming back as `device` means adb reconnects by itself.
- **Whether a hub back from a reboot is ready for the next runner once the session's wait ends:** it waits for adb,
  the RC app's pid and port 8000 in the hub's socket table, not for the robot's setup. Test: pull the hub's battery
  lead for a few seconds late in a session's default suite, and check the next stage's runner connects.
- **Whether the cable changes anything the default suite measures beyond Bench: Hub's bus costs and Bench: Framework
  Loop's periods:** those matched on one hub (2026-10-24 over Wi-Fi, 2026-11-14 over USB, [Setup](#setup)), with
  builds three weeks apart. `summary.md`'s `Link:` line labels every run; compare the rest when a session over the
  cable finishes.
- **How long a hot load takes on main's build, and whether INIT after it ends runs the new code on a build this
  size:** on 2026-11-14 no INIT came after a load ended. Test: deploySloth main's build by hand on a bare hub, time the
  push to `Processed Sloth Load` in logcat a few times, and INIT once after each.
- **Where the ball pipeline's slowdown starts, and which step costs it:** time each SnapScript stage (mask,
  distance-transform split, homography, drawing) with 0, 1, 2 and 3 balls in view, well after the pipeline switch,
  reporting the stage times in spare llpython slots.
- **Whether dropping AutoTune removes the 12 s:** build without `com.pedropathing:tuning` (or with `TunerScanner`
  excluded) and time process start to `INIT FINISH` in logcat. AutoTune runs from that artifact
  ([`CLAUDE.md`](../CLAUDE.md), Tuning).
- **Whether a Pinpoint unplugged mid-run reads (0, 0, 0) as the source says, and whether single bad reads happen while
  driving:** run the Pinpoint unplug and replug in `run_bench.py --interactive`. It decides whether a guard stops on
  one non-READY read or on a streak. `python3 scripts/bench/run_bench.py --pinpoint --interactive --ports-empty` pulls
  the cable twice, first under the SDK's driver alone (no follower), then under the follower.
- **Whether a healthy Pinpoint at rest ever reads a status other than READY after its first READY, and whether the
  41.8 ms `follower.update()` peak logged on the Cuttle bot (2026-07-30; retired 2026-11-13) recurs as an I2C stall:** test: Bench:
  Pinpoint's soak (`python3 scripts/bench/run_bench.py --pinpoint-soak --ports-empty`). An hour with none bounds the
  rate at about 1e-5 per read. The driving half was never counted: the Betta bot was retired on 2026-11-13 first,
  and the BIOBUZZ robot runs no Pinpoint.
- **Whether a rangefinder in SHORT mode keeps its accuracy at a 10 ms timing budget:** log a fixed target's distance
  at 20 and 10 ms budgets and compare the spread and the mean.
- **A plan's time from request to done inside a collection OpMode's own loop:** Bench: Hub and Bench: Planner Stop
  time it in their own loops (above); no bench times it in an `EnhancedOpMode`.
- **`GuardedOctoQuadLocalizer` with an OctoQuad plugged in:** read cost, valid reads after the first RUNNING, the
  start pose through START, and a cable pull: run Bench: OctoQuad with `--interactive`. START before the first
  RUNNING, the cable plugged in during INIT, and the same pull under Pedro's `OctoQuadLocalizer` as the control:
  `python3 scripts/bench/run_bench.py --octoquad --octoquad-chain --interactive --ports-empty`.
- **Whether main's close after the OpMode holds with the planner running and over many STOPs** (the fix for Ball
  Collection Auto's STOP, [Design around these](#design-around-these)): Bench: Webcam Close answered it once, for the
  bench build's version, with no planner (2026-11-17). Test on main's bench build: Bench: Planner Stop's webcam
  trials, `python3 scripts/bench/run_bench.py --usb --planner-stop --ports-empty` (STOP to `onEnd()` within 900 ms,
  `stop()` under 100 ms, '***STOP - OPMODE***' after EasyOpenCV's close, the worst case), and Bench: Webcam Stop,
  `python3 scripts/bench/run_bench.py --usb --webcam --ports-empty` (each STOP followed by an INIT that must see a
  frame within 3 s); run each more than once.
- **What a webcam unplugged at STOP, or between `startStreaming` and its first frame, does to the close after the
  OpMode:** from the code it can hang the event loop with no watchdog (above). Test: pull the webcam's cable right
  at STOP in Bench: Webcam Stop, then INIT. Also untried: Restart Robot or a config switch while a slow close runs
  (the SDK's `stopEventLoop` waits up to 10 s, [`dashboard-findings.md`](dashboard-findings.md)).
