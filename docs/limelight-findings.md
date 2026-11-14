# Limelight findings

The Limelight 3A ball camera beyond what [`CLAUDE.md`](../CLAUDE.md) says: why the INIT sync works the way it does, SDK
12's `Limelight3A` timing, reaching the Limelight from a laptop, and where the camera-mount numbers come from. Read it
before changing `LimelightSync` or `LimelightBallSource`, moving the Limelight, or reaching it from a laptop.

## What to do

- **Keep the full upload at every INIT.** Never skip it because the Limelight reports a matching stamp (the build's
  checksum of the pipeline and the SnapScript, the Limelight's Python script): a matching stamp can't prove the
  Limelight runs the committed files ([below](#why-the-sync-uploads-every-init-and-runs-on-a-thread)). To tune in the
  web editor, turn `Limelight Balls → syncOnInit` off.
- **Keep the sync on its own thread**, even though SDK 12 has no INIT time limit: the thread is what lets STOP get out
  of a sync.
- **Don't let the hub's clock jump during an OpMode with ball vision.** Three SDK Limelight timings are wall-clock, and
  the hub's first Driver Station (DS) or browser connection after boot moves the wall clock by decades. A jump
  mid-OpMode can raise a false Limelight fault and wreck frame timing for about 27 s or until the next INIT
  ([below](#the-wall-clock-jump)). A match shouldn't hit it: the DS has connected before it can start an OpMode.
  Running with no DS, open `http://192.168.43.1:8080` once after boot so the jump comes before INIT, except before an
  `--interactive` hub bench run: its Bench: Clock step needs the clock unset (`scripts/bench/run_bench.py`).
- **Moving the Limelight at the same height, tilt and heading keeps `H_ARRAY`** (the SnapScript's homography from
  pixels to ground inches). Set `Mount.xIn` to the lens's forward offset from the robot centre plus 29.6, and
  `Mount.yIn` to its left offset minus 0.4 ([below](#where-the-camera-mount-numbers-come-from)), then check in Camera
  Module Test.
- **Back the Limelight up** with `GET /pipeline-atindex` for pipelines and the web UI's websocket for scripts: no REST
  endpoint downloads a script ([below](#reaching-the-limelight-from-a-laptop-over-usb)).

## Why the sync uploads every INIT and runs on a thread

Read from source on 2026-11-10: `LimelightSync`, `LimelightBallSource`, `TeamCode/build.gradle`, and SDK 12.0.0's
`OpMode`.

- Every INIT of an OpMode that keeps ball vision on uploads `limelight/ball_pipeline.vpr` and the SnapScript to the
  ball pipeline, with no check of what is already there (`LimelightSync.java:127-145`). It takes about 1.7 s
  ([hub-bench-findings.md](hub-bench-findings.md)).
- The results identify the running script only by the stamp in `llpython[0]` (the script's output array): 23 bits of
  a CRC32 of both files, which the build writes into the uploaded script's `SCRIPT_ID` line
  (`TeamCode/build.gradle:119-124`). A matching stamp can't prove the committed files run:
  - a copy edited in the web editor keeps that line, so it still reports the stamp;
  - pipeline settings such as exposure, gain and resolution never show in the script's output.

  Skipping the upload on a matching stamp would let web-editor tuning or settings drift run in a match.
- `syncOnInit` off makes the sync only switch pipelines and accept a web-editor copy's `SCRIPT_ID`
  (`LimelightBallSource.java:206`, `:266-269`). Like any dashboard edit, it turns back on at deploySloth or an app
  restart.
- **Why a thread.** SDK 12 gives `init()`, `init_loop()` and `loop()` no time limit (`OpMode`'s `msStuckDetect*` fields
  are deprecated: "OpModes no longer have a time limit"), so a slow INIT isn't killed. The reason is STOP: a
  `Limelight3A` POST can block 15 s ([`CLAUDE.md`](../CLAUDE.md), Conventions), which on the OpMode thread freezes INIT,
  and a STOP during it outlasts the SDK's 900 ms stop window ([hub-bench-findings.md](hub-bench-findings.md)). On its
  own thread, `LimelightBallSource.stop()` cancels the sync before its next POST (`LimelightBallSource.java:185-192`);
  one already sent finishes on that daemon thread (`LimelightSync.java:47-48`, `:80-82`). A crash in the thread is
  rethrown on the OpMode thread by `Request.rethrowIfCrashed()` (`LimelightBallSource.java:209`).
- **One POST at a time, across Sloth reloads.** Every POST goes through one semaphore kept in `OpMode.blackboard`
  (`LimelightSync.java:63-64`, `:156-164`), which a deploySloth doesn't replace, so a reloaded `LimelightSync` with
  its own statics waits for a POST the old class left open instead of landing on the Limelight before or after it.
  Bench: Limelight Hung checks it (`reinit_fresh`); its 2026-10-24 run is in
  [hub-bench-findings.md](hub-bench-findings.md).

## SDK 12 `Limelight3A` timing

Read from source: SDK 12.0.0 `Hardware` sources, `com.qualcomm.hardware.limelightvision.Limelight3A` and `LLResult`.
CLAUDE.md already covers the blocking POSTs, swallowed poll errors and `setPollRateHz` while polling.

| what | behaviour |
|---|---|
| poll interval | 10 ms (100 Hz) by default. `setPollRateHz` clamps to 1-250 Hz and sets `1000 / rate` ms, integer division. 100-250 Hz didn't change the ball pipeline's frame rate ([hub-bench-findings.md](hub-bench-findings.md)) |
| GET (the `/results` poll, `getStatus()`) | 100 ms connect timeout, 100 ms read timeout |
| POST and DELETE | 100 ms connect timeout, 15 s read timeout: a connection that can't be made fails within 100 ms; a Limelight that accepts it and doesn't answer holds the caller 15 s |
| `getTimeSinceLastUpdate()` | `System.currentTimeMillis()` minus the wall-clock time of the last successful poll. That time starts at 0, so until the first poll succeeds it returns the whole wall clock. `isConnected()` is this ≤ 250 ms, so it inherits the wall clock |
| `LLResult.getControlHubTimeStamp()` | wall clock when the poller parsed the result; `getStaleness()` is now minus that, on the wall clock |
| `LLResult.getControlHubTimeStampNanos()` | the same wall-clock time × 1,000,000, not `System.nanoTime()`: never compare the two |

### The wall-clock jump

The hub's clock reads about 1970 after boot until a DS connects or a browser opens `http://192.168.43.1:8080`, then
jumps to the real date ([`CLAUDE.md`](../CLAUDE.md), Build & deploy). If that happens while `LimelightBallSource` runs
(read from source; neither effect measured, and neither is guarded in code):

- **A false fault for one loop.** Until the next poll lands (about 10 ms), `getTimeSinceLastUpdate()` returns decades.
  `connectionProblem()` compares it with `noDataFaultMs` (250), so a loop that reads it then raises the Limelight
  fault ("no data for" about 1.8e9 s) and drops the ball tracks (`LimelightBallSource.java:136`, `:156-159`,
  `:194-197`). A ball collection that starts in that loop ends SKIPPED (`BallCollection.java:175-179`).
- **Frame timing off for 128 frames.** If the jump lands between the poller's parse of a new result and our read of it
  (up to one poll interval), that frame's `getStaleness()` is decades long, and so is the hub-clock offset taken from it
  (`LimelightBallSource.java:241-245`). The offset in use is the minimum over the last 128 frames (`recordOffset`), so
  that frame and the 127 after it, about 27 s at the ball pipeline's 4.7 results/s, get a capture time decades in the
  past:
  - `Camera` places balls with the oldest robot pose in its 64-loop history instead of the capture-time pose, so field
    positions are wrong while the robot moves (`Camera.java:82`; `RobotStateHistory.sampleAt` clamps);
  - frame age and Camera's `Latency (ms)` line read decades;
  - a ball collection waiting for vision never sees a fresh frame and ends SKIPPED after `visionWaitMs` (1 s by
    default; `BallCollection.java:204-208`).

  The offsets belong to `LimelightBallSource`, which every INIT builds anew, so the next INIT clears it.

## Reaching the Limelight from a laptop over USB

Measured on 2026-09-30 on the team's Limelight 3A, plugged into a Mac by USB, including a full backup of every
pipeline and script. Vendor pages fetched 2026-11-10:
[change log](https://docs.limelightvision.io/docs/docs-limelight/software-change-log),
[REST API](https://docs.limelightvision.io/docs/docs-limelight/apis/rest-http-api),
[Limelight 3A](https://docs.limelightvision.io/docs/docs-limelight/getting-started/limelight-3a).

| | address | how known |
|---|---|---|
| the Limelight | `172.29.0.1` | measured; vendor change log (the subnet) and 3A page (its port-forwarding example); hub configs (`betta_decode.xml:28`, `cuttle_decode.xml:49`) |
| the address it gives the host | `172.29.0.22` | measured on the Mac; the hub configs' serial number `EthernetOverUsb:eth0:172.29.0.22` |
| on Windows | the `172.28.0.0/24` subnet instead | vendor change log, Limelight OS 2024.9 |
| with a USB ID set | the ID becomes the third octet (`172.29.<id>.0/24`) | vendor change log |
| REST API | port 5807 | vendor REST page; `Limelight3A`'s base URL |
| web UI | port 5801 | vendor 3A page |
| the web UI's websocket | port 5805 | measured on our unit only |

Over the robot Wi-Fi, without the USB cable, the dashboard forwards these ports at `192.168.43.1`
([dashboard-findings.md](dashboard-findings.md#limelight-proxy)).

- **macOS: give the terminal app Local Network permission** (System Settings → Privacy & Security → Local
  Network), or every connection fails at once with `No route to host`. Some browsers need their own entry; Safari
  worked without one.
- **Pipelines:** `GET http://172.29.0.1:5807/pipeline-atindex?index=N` returns pipeline N's JSON, identical to the web
  UI's `.vpr` download. Diff it against `limelight/ball_pipeline.vpr`.
- **Scripts:** the vendor REST page lists no endpoint that returns a SnapScript's Python (`/getsnapscriptnames` gives
  names only). Over the web UI's websocket (port 5805), send `set_pipeline_index 0*N#`, then repeat
  `request_current_python_script_for_download 0*#` until the reply's script name matches the `desc` field of pipeline
  N's JSON: replies lag a pipeline switch. This switches the running pipeline, so don't do it during an OpMode.
  `/api/pipeline-bundle/download` failed on our unit.
- **Hub code against a real Limelight, without a hub:** the SDK's `Limelight3A` runs on a desktop JVM with the
  `classes.jar` from the SDK's `Hardware` and `RobotCore` AARs plus an `org.json` jar (measured 2026-09-30). Build it
  as `new Limelight3A(EthernetOverUsbSerialNumber.fromSerialNumber("EthernetOverUsb:eth0:172.29.0.22"), "limelight",
  InetAddress.getByName("172.29.0.1"))` (`EthernetOverUsbSerialNumber` is in
  `org.firstinspires.ftc.robotcore.internal.usb`). Re-test `LimelightSync` this way after a Limelight OS update.

## Where the camera mount numbers come from

`BallFieldTransform.Mount` (`BallFieldTransform.java:14-17`) is `xIn` 33.2, `yIn` -0.4, `headingDeg` 0, derived on
2026-10-03 rather than tape-measured. It places the `(0,0)` crosshair, the ground point `H_ARRAY` calls (0,0),
relative to the robot centre, the Pinpoint offsets' origin ([`CLAUDE.md`](../CLAUDE.md), Tuning):

- On the Betta bot the Limelight's lens sits 3.5 in ahead of the robot centre, on the centreline (measured 2026-10-03).
- Decomposing `H_ARRAY` with the Limelight's lens calibration scaled to `CALIBRATION_SIZE`, 640 × 480
  (`limelight/ball_contour_snapscript.py:28-41`), puts the camera 29.6 in behind and 0.39 in left of the crosshair,
  15.55 in above the ground, pitched 15.85° down, with its optical axis 1.9° right of the calibration board's x axis
  (computed 2026-11-10 with the code below; about ±0.1 in depending on which rotation column sets the scale).
- So `xIn` = 3.5 + 29.6 = 33.1 (33.2 in code) and `yIn` = 0 - 0.39 = -0.4.

Using it:

- `headingDeg` is 0 only if the board's x axis was square to the robot's forward when `H_ARRAY` was fitted.
- Sanity-check any new homography by decomposing it: its height and pitch should match a tape measure on the mount.
  CLAUDE.md's Camera Mount check in Camera Module Test stays the final word.

The decomposition, in Python with numpy. Above the code, paste `H_ARRAY`, `CALIBRATION_SIZE`, `LENS_CALIBRATION_SIZE`
and `LENS_FX`/`LENS_FY`/`LENS_CX`/`LENS_CY` from the SnapScript. It assumes `UNDISTORT = True` (`H_ARRAY` fitted to
undistorted pixels). Today's values print `[-29.64 0.39 15.55]`, `15.85` and `-1.91`.

```python
import numpy as np

sx = CALIBRATION_SIZE[0] / LENS_CALIBRATION_SIZE[0]
sy = CALIBRATION_SIZE[1] / LENS_CALIBRATION_SIZE[1]
K = np.array([[LENS_FX * sx, 0, LENS_CX * sx], [0, LENS_FY * sy, LENS_CY * sy], [0, 0, 1]])
M = np.linalg.inv(K) @ np.linalg.inv(np.array(H_ARRAY))
M /= np.sqrt(np.linalg.norm(M[:, 0]) * np.linalg.norm(M[:, 1])) * np.sign(M[2, 2])
U, _, Vt = np.linalg.svd(np.column_stack([M[:, 0], M[:, 1], np.cross(M[:, 0], M[:, 1])]))
R = U @ Vt
camera = -R.T @ M[:, 2]
axis = R.T @ [0, 0, 1]
print("camera from (0,0), in (forward, left, up):", camera.round(2))
print("pitch down, deg:", np.degrees(np.arcsin(-axis[2])).round(2))
print("axis yaw from the board's x, deg (+ left):", np.degrees(np.arctan2(axis[1], axis[0])).round(2))
```

## Still unknown

- **Whether the clock jump does what the source says.** Power-cycle the hub, confirm `adb shell date` still reads
  about 1970, start an OpMode with ball vision without a DS or the RC web page, then connect a DS. Logcat (tag
  `Fault`) showing `Limelight: no data for` then `Limelight cleared` confirms the first effect; Camera's
  `Latency (ms)` reading decades on the DS for about 27 s confirms the second, which needs the jump inside a window of
  about 10 ms, so repeat the test.
- **Whether one sender holds across a real deploySloth.** Bench: Limelight Hung's `reinit_fresh` starts a second
  worker by reflection, against a fake, not from a reloaded class. Never measured with a real Limelight either: a STOP
  in the middle of the INIT sync, and a SnapScript edit reaching the Limelight through deploySloth alone. Test:
  `python3 scripts/bench/run_bench.py --limelight-reload` (Bench: Limelight Reload,
  [`scripts/bench/README.md`](../scripts/bench/README.md#bench-limelight-reload)).
- **Whether the decomposed height and pitch are real.** Tape-measure the Limelight's lens height (should be about
  15.5 in) and tilt (about 15.9° down) on the Betta bot.
- **The websocket commands, port 5805 and the bundle endpoint on other Limelight OS versions.** The unit's OS version on
  2026-09-30 isn't recorded. Re-run the script download and `/api/pipeline-bundle/download` after any Limelight OS
  update, and record the version here.
