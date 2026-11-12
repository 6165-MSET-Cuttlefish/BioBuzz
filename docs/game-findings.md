# BIOBUZZ rules and field findings

What the BIOBUZZ rules let the robot send over Wi-Fi and plug into USB, what our code sends today, and the HIVE's
geometry from FIRST's field CAD. Read it before an event, before adding anything that talks over Wi-Fi or USB, and
before changing HIVE numbers in `BioBuzzField` or planning routes under the HIVE frame.

## What to do

- **Turn the dashboard off before the event and keep it off until you leave.** R704(D) bans FTC Dashboard-style
  streaming, and the FTC Q&A bans Wi-Fi streaming anywhere in the venue, pit and practice field included (quotes
  below).
  - On the Driver Station (DS), INIT `Enable/Disable Dashboard` (TeleOp list, group `dash`). If it says the dashboard
    is enabled, press START; INIT it again to see "disabled", then STOP. Off survives app restarts and power cycles
    ([dashboard-findings.md](dashboard-findings.md#enabledisable-dashboard)).
  - Check it again at load-in: INIT it, read the line, press STOP without START.
  - `OptimizationToggles.dashboardTelemetry = false` is not an off switch: `EnhancedOpMode` still queues a dashboard
    packet every loop.
  - No dashboard tuning at the event: bring tuned values in the source ([CLAUDE.md](../CLAUDE.md), Tuning).
- **Choose the alliance some other way before the first event.** `Context.allianceColor` is set only on the dashboard
  (CLAUDE.md) and defaults to RED (`architecture/core/Context.java:10`), so with the dashboard off everything runs as
  RED: in a blue match, BioBuzz Tele's cell-tip camera watches RED's HIVE (`biobuzz/modules/CellTipCamera.java:62`).
  CLAUDE.md's "on the dashboard, never from an OpMode" rule has to change with it.
- **Keep laptops off the robot's Wi-Fi during matches** (R704(C)). At events, deploy over a USB cable: R704(B) lets no
  device but the Robot Controller and DS connect to the robot's network ([Still unknown](#still-unknown)).
- **USB on the robot takes only** UVC webcams (standard USB webcams; the manual's example is the Logitech C270), the
  Limelight 3A, a USB hub or switch, and one REV Expansion Hub (R707, R708, R701). An Expansion Hub on USB instead of
  RS485, which Photon's parallel writes need ([photon-bench.md](photon-bench.md)), is legal.
- **The A-frame legs lean in over the lane between the HIVE rails**, and `BioBuzzField.hiveRails()`, the planner's
  only obstacles on the field, leaves them out. Before routing a robot under the frame, add the legs for its height
  ([HIVE geometry](#hive-geometry-field-cad)). Clear of the legs, any legal robot fits under a HIVE at rest.
- **`BioBuzzField`'s HIVE numbers stand:** they match the CAD and the manual within 0.13 in. Close the TODO at
  `biobuzz/BioBuzzField.java:65` (keep 21.455 or use the CAD's 21.39) and the opening-position item in
  [sotm-tuning.md](sotm-tuning.md#not-done-yet).

## Sources

All fetched 2026-11-10:

- Competition Manual: <https://ftc-resources.firstinspires.org/ftc/game/cm-html>, which redirected to the revision its
  file name calls TU08. Rules are quoted from it.
- FTC Q&A: <https://game-qa.firstinspires.org/boards/2027/FTC/QA>.
- Field CAD: <https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad-step>, one STEP file, last
  modified 2026-09-15.

Code paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. slothboard (our dashboard, a fork of
FTC Dashboard) and AutoTune (Pedro's tuning web app) are read from the versions `TeamCode/build.gradle` pins:
`com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0-6165.7` and `com.pedropathing:tuning:1.0.1`.

## Wi-Fi and streaming (R704)

Competition Manual §12.7, R704:

> B. Teams may not interfere with the ROBOT networks. All communication signals must originate from only the ROBOT
> CONTROLLER device or the DRIVER STATION device using the ROBOT CONTROLLER Wi-Fi network. No other devices may
> attempt to connect to, interfere with, or alter the ROBOT CONTROLLER Wi-Fi network.
>
> C. Ensure all programming laptops and other devices (other than the DRIVER STATION device) are disconnected from the
> ROBOT CONTROLLER Wi-Fi network during MATCH play.
>
> D. Software with access to the ROBOT CONTROLLER Wi-Fi network must limit the amount of data being streamed (i.e.,
> continuous transmission of data) over Wi-Fi. Software may only stream robot control data, debugging data, and
> telemetry to and from the ROBOT using the FTC Driver Station Application. Additional logging/streaming services,
> such as those hosted by third party plugins and tools such as FTC Dashboard, FTControl Panels, and others are
> prohibited. No continuous video stream is allowed.

FTC Q&A #26 (answered 2026-10-19, on a HuskyLens 2 vision sensor, which streams over its own Wi-Fi):

> Wi-Fi streaming is not allowed anywhere in a competition venue (per R704.D), so Wi-Fi streaming (even in the pit and
> practice field) is prohibited.

What our code serves on the robot's Wi-Fi today (read from source):

| what | when | what stops it |
|---|---|---|
| dashboard packet: field overlay, pose, faults, plus telemetry lines while `dashboardTelemetry` is on (`architecture/core/EnhancedOpMode.java:492`, `updateDashboard`) | every INIT and run loop of every `EnhancedOpMode` (`:168`, `:235`) | slothboard off: `sendTelemetryPacket` returns at once |
| dashboard camera stream from the cell-tip webcam, 30 fps (`biobuzz/modules/vision/WebcamSession.java:67`) | from INIT of every OpMode on `BioBuzzRobot`, which always builds `CellTipCamera` (`biobuzz/BioBuzzRobot.java:32`); BioBuzz Tele included | slothboard off: `startCameraStream` returns at once. `setCameraStreamEnabled(false)` exists; only `WebcamBench` calls it |
| Limelight proxy: the Limelight's MJPEG video (port 5800), web UI (5801) and APIs (5805, 5807), forwarded from 172.29.0.1 (slothboard `LimelightProxyManager`) | while slothboard is on | slothboard off |
| slothboard's web server (port 8000) | always; while off it answers a client only with status, plus the config, OpMode and hardware config lists when the client connects | nothing ([Still unknown](#still-unknown)) |
| AutoTune's web server (10158) and websocket (12649), started at every app start (`com.pedropathing.tuning.autotune.Hooks`) | always; sends only to a connected client | nothing ([Still unknown](#still-unknown)) |
| DS camera preview (`WebcamSession.java:54`, SDK 12.0.0 `CameraStreamServer.handleRequestFrame`) | one JPEG per request from the DS app | nothing needed: it goes through the DS app, which (D) allows (our reading) |

The slothboard and AutoTune rows send nothing without a connected client, so in a match, with laptops off the Wi-Fi as
(C) requires, nothing goes out. The Q&A ruling extends the ban to the pit and practice field, where a laptop would
connect.

## USB (R707, R708)

> R707 USB is for vision. Only the following devices may be connected to the ROBOT control system using USB:
> A. webcams and optical vision sensors per R708, B. USB hub or USB switch, and C. a REV Expansion Hub.

> R708 Use only supported USB vision. Only single image sensor vision devices that are natively supported by the ROBOT
> CONTROLLER app are allowed to connect to USB (stereoscopic cameras are not allowed). This includes the following:
> A. all UVC compatible USB webcams (Logitech C270, and related), and B. Vision coprocessors allowed per R702.
>
> UVC compatible USB webcams may only use the UVC provided stream / data. No other interfaces or data provided by the
> webcam may be used.

- **Limelight:** R702's Table 12-9 lists the Limelight 3A as the supported programmable vision coprocessor, and R702
  says "reprogramming them is permitted", so `LimelightSync` uploading our ball pipeline at INIT
  (`biobuzz/modules/vision/LimelightSync.java`) is allowed. R702 Example 6 names the Limelight 3G as prohibited.
- **Webcam:** the cell-tip webcam opens through the SDK's UVC driver (`WebcamName`; SDK 12.0.0's
  `internal.camera.libuvc`), the R708(A) case.
- **Expansion Hub:** R701(C) allows "no more than one additional REV Expansion Hub"; R707(C) lets it connect over USB.
- **USB hub power:** Q&A #1 (answered 2026-10-07): a powered USB hub may power the allowed USB devices whichever legal
  way the hub itself is powered (the question asked about a COTS USB battery).
- **Not UVC means not USB:** Q&A #26: a vision sensor that isn't UVC-compatible (there, a HuskyLens 2) may not use
  USB, even where R702 allows it on I2C.

## HIVE geometry (field CAD)

The CAD's origin is the field centre, x toward the audience's right (blue's side), y away from the audience, in
inches. Pedro coordinates are CAD + 70.75 on both axes (`FieldConfig.fieldWidthInches` = 141.5). CAD values are
measured from the STEP file above; `am-...` are its part names.

| | CAD | manual | `biobuzz/BioBuzzField.java` |
|---|---|---|---|
| rails: foot bars (`am-5878`) | x ±22.75 to ±24.733, y ±19.4723, top 2.15 in up | frame base 49.46 in wide, 38.95 in deep (Fig 9-8) | same x and y (`:152`-`154`) |
| HIVE pivots | x ±12.75, 43.95 in up | 25.5 in centre to centre (Fig 9-10), 43.95 in up (§9.6.1) | 12.75 (`:31`) |
| CELL tilt when up | 30° | 30° (Fig 9-10) | 30° (`:64`) |
| CELL opening from the pivot, along the bar | 21.39 in (the CELL's far end) | half the 42.91 in bar (Fig 9-9): 21.455 | 42.91 / 2 (`:65`, with the TODO) |
| CELL opening height | 53.38 to 65.50 in | 53.5 to 65.6 in (Fig 9-10) | bottom 53.5 (`:66`) |
| lowest part at rest, legs aside | 30.65 in (a goal rib, `am-5866`); a down CELL's floor 31.98 in | bottom of HIVE 30.6 in (Fig 9-10) | not used |

- **Only the rails reach the floor.** The frame feet (`am-5879`, top 2.13 in) sit on the bars, and the A-frames stand
  on them (Fig 9-8). The rails' outer edges span 49.47 × 38.94 in, the manual's frame base.
- **The A-frame legs (`am-5876`) lean in over the lane.** Each runs from a rail's end (x ±23.8, y ±18.6) up to the
  frame's top corner (x ±12.2 to ±13.2, y within 1 in of 0, 41 in up); Fig 9-10's end view shows the lean. Where a
  leg's inner edge is, per height (the lane is x inside ±22.75, Pedro x 48.0 to 93.5):

  | height above the tiles (in) | 6 | 12 | 18 | 24 | 29 |
  |---|---|---|---|---|---|
  | leg's inner edge, x from the centre (in) | 21.7 | 20.1 | 18.5 | 16.8 | 15.5 |
  | inside the rail's inner edge by (in) | 1.0 | 2.7 | 4.3 | 5.9 | 7.3 |
  | leg's y from the centre (in) | 15.6–16.7 | 12.9–14.0 | 10.2–11.3 | 7.6–8.7 | 5.3–6.4 |

  `hiveRails()` is the two foot bars only, so a planned route along a rail near either end can drive a tall robot
  into a leg.
- **Clear of the legs, any legal robot fits under a HIVE at rest.** R105(A): a robot "must remain within a 18 in.
  (45.70 cm) by 24 in. (61.0 cm) by 29 in. (73.65 cm) tall sizing volume when fully expanded", 1.6 in under the
  lowest part. No leg comes within x ±12.2, so a robot that stays within 12 in of the field's centre line (Pedro x
  58.75 to 82.75) passes under at any legal height. The shot solver still refuses shots from under the frame
  ([sotm-tuning.md](sotm-tuning.md)).
- **The code's numbers stand.** Every value in the table agrees within 0.13 in, too little to matter for aiming.

## Still unknown

- **Whether a server that is up but sending nothing breaks R704(D):** slothboard's server while off, and AutoTune's
  servers, which have no off switch. Ask the FTC Q&A, or the event's FIRST Technical Advisor (FTA) at inspection. If
  it does, competition builds must leave out slothboard and `com.pedropathing:tuning`.
- **Whether a laptop may join the robot's Wi-Fi in the pit:** R704(C) names only match play, but (B) lets no device
  but the Robot Controller and DS connect. Ask the FTC Q&A; until then use a USB cable at events.
