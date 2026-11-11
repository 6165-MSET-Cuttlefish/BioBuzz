# AprilTag webcam findings

The cell-tip check (`CellTipCamera`) reads AprilTags on our alliance's HIVE to tell whether it is scorable or
tipped. Its webcam, config name `aprilTagDetector`, is an Arducam B0385 with an OmniVision OV9782 colour
global-shutter sensor (the whole frame is exposed at once). Read this before changing the webcam, its mount, cable,
USB port, resolution or exposure, or before trusting a cell-tip verdict.

Code paths are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. Vendor facts come from the Arducam
B0385 [datasheet][ds] (V1.0) and [quick-start manual][qs], both read 2026-11-10, unless another source is named.
Prices and stock change, so each one carries the date it was seen.

## Do this

- **The cell-tip verdict hasn't been checked on a real HIVE.** When tip verdicts look wrong, suspect the logic first
  ([below](#the-verdict-is-unproven-on-a-real-hive)).
- **Anything that hides the tags reads as TIPPED**, including distance, motion blur and the HIVE leaving the frame.
  On the dashboard's Webcam Controls group, lower `exposureMs` from 8 toward 3–4 ms and raise `gain` until tags stop
  dropping out while the robot turns (computed, not yet tried on the robot).
- **640×480 probably sees about 61° wide, not the lens's 70°** (from goBILDA's calibration of the same board). If
  the HIVE leaves the frame at the sides, stream 1280×800 or 1280×720 MJPEG for the full 70°, which also makes each
  tag about 1.67× wider in pixels. Its Control Hub CPU cost per frame is not measured.
- **Keep this camera.** A wider one shrinks every tag, so the check reads TIPPED from shorter range. If a wider view
  is ever needed, fit a 90–100° low-distortion M12 lens, not a fisheye.
- **Plug it into the Control Hub's USB 3.0 port.**
- **Spare cable: Arducam CB043**, $1.99 at UCTRONICS (2026-11-10). Check its pinout with a multimeter before
  plugging it in ([below](#spare-and-shorter-cables)).
- **The camera is hard to buy in the US** (2026-11-10). Ask UCTRONICS or Arducam for a restock date, or buy goBILDA's
  cased version when it's back and take the board out of the case.

## The camera

| | B0385 |
|---|---|
| sensor | OV9782, 1/4 in, colour, global shutter, 1280×800 (16:10), 3 µm pixels, 68 dB dynamic range |
| output | USB 2.0 UVC (USB Video Class: no driver); shows up as "Arducam OV9782 USB Camera" (manual) |
| modes | MJPEG (compressed) 100 fps at 1280×800, 1280×720, 800×600, 640×480 and 320×240; YUY2 (uncompressed) 10 fps at 1280×800 and 1280×720 only |
| lens | 2.8 mm focal length, 70° horizontal field of view (HFOV) on this sensor, distortion <1%, M12 × 0.5 mount, 650 nm IR-cut ([Mounting](#mounting)) |
| power | 5 V, 200 mA max; −20 to +70 °C |
| board | 38 × 38 mm |

Some resellers list 120 fps or "without IR filter". The datasheet and manual say 100 fps with an IR filter, so trust
those. 640×480 exists only as MJPEG, which is why `WebcamSession` asks for MJPEG
(`biobuzz/modules/vision/WebcamSession.java:30-31, 64-66`).

## The cell-tip check

### What it needs from a camera

Read from source on 2026-11-10:

- `CellTipPipeline` (`biobuzz/modules/vision/CellTipPipeline.java`) runs OpenFTC's AprilTag detector
  (`org.openftc.apriltag.AprilTagDetectorJNI`; tag36h11, decimation 2) on the grayscale frame. It uses only each
  tag's corners, never its pose.
- It drops tags under `minTagAreaPx` = 120 px², about 11 px square (`:123`).
- Each CELL carries a cluster of 4 tags (see [below](#the-verdict-is-unproven-on-a-real-hive)). For each of our
  alliance's two clusters, it averages the roll of the tags' bottom edges. A cluster with at least `minVisibleTags`
  (1) tags and |roll| < `scorableMaxRollDeg` (90°) counts as upright, and any upright cluster makes the alliance
  scorable (`:126-131`, `:176-178`).
- Anything else (tags upside-down, or no tags at all) is TIPPED. The verdict starts TIPPED and changes only after
  the new reading has held for `holdSeconds` (0.25 s) (`:79`, `:133-141`).
- `CellTipCamera.isTipped()` returns that verdict if it is at most `maxStalenessMs` (250 ms) old, and false
  otherwise: a camera that stops delivering frames reads neither TIPPED nor scorable
  (`biobuzz/modules/CellTipCamera.java:77-81`). `RobotActions.checkTip()` ends on the first TIPPED reading, or after
  `checkTipTimeoutMs` (10 s) (`biobuzz/RobotActions.java:19-24`). SOTM Test's `flipOnCameraTip` flips the aimed CELL
  on each new TIPPED reading ([sotm-tuning.md](sotm-tuning.md)).

What follows from that:

- **Lens distortion barely matters.** The B0385's is under 1%, nowhere near enough to move a tag's roll close to
  the 90° margin.
- **Anything that makes tags vanish gives a false TIPPED**: tags shrinking under 120 px² with distance, motion blur,
  or the HIVE leaving the frame. A global shutter removes rolling-shutter skew, but it doesn't remove blur.
- **Blur** (computed; the turn rate is an assumption, so scale it to the robot): at the default 8 ms exposure
  (`biobuzz/modules/vision/WebcamControls.java:25`) and a 180°/s turn, the image moves 1.44° during each exposure.
  At 640×480 (9.5 px/° at the image centre, 10.5 averaged across the width) that is about 14–15 px, more than the
  width of an 11 px tag. Lower `exposureMs` toward 3–4 ms with more `gain`, and check in Cell Tip Test while
  turning. Change them in INIT: each change blocks the OpMode loop while it is written
  ([hub-bench-findings.md](hub-bench-findings.md)).
- **Not built: frames captured during a fast turn could count as "no verdict"** instead of TIPPED. The verdict is
  stamped on the same clock as `RobotStateHistory.nowSeconds()` (`CellTipPipeline.Verdict`), so a
  `RobotStateHistory` recorded in `CellTipCamera`, as `Camera` records one, would give the turn rate at each frame's
  capture time.
- **A CELL lying on its side** would read roll ≈ ±90°, right on the threshold. It needs its own rule if BIOBUZZ
  lets that happen.

### The verdict is unproven on a real HIVE

The rule "right side up = scorable" has been checked only on a mock-up, never on a real or full-size HIVE. What
the BIOBUZZ Competition Manual says:

- §9.6 and §9.6.2: a HIVE is two CELLs on one pivot, and in each stable state exactly one CELL faces up.
- §9.9: each CELL has a 4-tag cluster on its bottom face, facing the tiles, with its bottom edge toward the field
  centre. The pipeline's clusters match: RED 30–33 and 34–37, BLUE 42–45 and 38–41
  (`biobuzz/modules/CellTipCamera.java:27-29`).
- §10.5.1 sets the TIP criteria.

Whether both of a HIVE's clusters can be in view at once is unverified. If they can, an upright reading from the
other CELL can hide a tip, because any upright cluster means scorable. An empty frame reads TIPPED. The test that
settles this is under [Still unknown](#still-unknown).

## Resolution and field of view

By goBILDA's lens calibration of its own OV9782 camera, 640×480 and 800×600 are 4:3 crops of the 16:10 sensor,
which cut off its sides. The numbers below come from that calibration (USB vid 0x0C45, pid 0x0366;
`teamwebcamcalibrations.xml` in [goBILDA's calibration zip][gbcal], read 2026-11-10). fx is the focal length in
pixels, HFOV = 2·atan(width / 2 / fx), and VFOV is the vertical equivalent.

| mode | fx (px) | HFOV | VFOV | px per degree (width ÷ HFOV) |
|---|---|---|---|---|
| 1280×800 | 908.758 | 70.3° | 47.5° | 18.2 |
| 1280×720 | 908.785 | 70.3° | 43.2° | 18.2 |
| 800×600 | 684.509 | 60.6° | 47.3° | 13.2 |
| 640×480 (what `WebcamSession` streams) | 545.584 | 60.8° | 47.5° | 10.5 |

- fx at 640×480 is 0.600 of its 1280×800 value. A 4:3 centre crop of the 800-row sensor, scaled to 480 rows, gives
  exactly 0.600; scaling the full width would give 0.500. The unchanged 47.5° VFOV agrees.
- 640×400 (full width) isn't in the datasheet's mode list. Treat it as unavailable until someone tries it.
- Ignore the file's 480×320 entry (fx 270.26, about 83°). It doesn't fit the rest.
- `CellTipPipeline`'s FX 578, CX 320, CY 240 (`:60`) only feed the detector's pose solve, which nothing uses. They
  are not a calibration.
- These numbers come from goBILDA's unit. Whether the bare B0385 reports the same vid/pid and crops the same way is
  unverified.

## Lens distortion, calibration and pose

This matters only if VisionPortal AprilTag pose is ever used. The cell tip doesn't use pose.

- **The SDK has no built-in calibration for this camera.** FTC SDK 12.0.0's built-ins
  (`res/xml/builtinwebcamcalibrations.xml` in `RobotCore-12.0.0.aar`, read 2026-11-10) cover the Microsoft LifeCam
  HD-3000, the Logitech C310, C920 and C270, and goBILDA 3122-0003-0001 (vid 0xBDA, pid 0x5805). There is nothing for
  vid 0x0C45 / pid 0x0366. In the FTC docs' [webcam table][ftcwebcams] (2026-11-10), the global-shutter rows (Arducam,
  Kayeton, ELP) are all monochrome 70° cameras with no built-in calibration, and the colour OV9782 isn't listed.
- **goBILDA's file should work as a drop-in** (not tried). At 640×480 it gives fx = fy = 545.584, principal point
  (350.588, 223.150), k1 0.0290195, k2 −0.0238128, k3 −0.0378927. That is small distortion, consistent with the
  datasheet's "<1%". To use it, confirm the bare board's vid/pid, then copy the file unchanged into
  `TeamCode/src/main/res/xml/`. That is a `res/` change, so it needs a full install (CLAUDE.md, Build & deploy).
- **SDK pose ignores distortion anyway.** In Vision 12.0.0, `AprilTagProcessorImpl.poseFromTrapezoid` and
  `poseFromNGrt4Pts` call `Calib3d.solvePnP` with a new, empty `MatOfDouble` as the distortion coefficients, and
  never undistort the corners. `AprilTagProcessor.Builder.setLensIntrinsics` takes only fx, fy, cx and cy (javap,
  2026-11-10). So pose assumes a perfect pinhole lens, and on a distorted lens its error grows toward the image
  edges. To get pose on such a lens, calibrate with OpenCV (its fisheye model for a fisheye), undistort each
  detection's 4 corners, then call `solvePnP` yourself. The board to print is
  [calibration-chessboard-3in-letter.pdf](calibration-chessboard-3in-letter.pdf).

## Mounting

From the datasheet's drawings (unit mm):

- The board is 38.00 × 38.00, 1.60 thick (from goBILDA's STEP). The sensor's optical centre is at the board centre
  (19.00, 19.00).
- There are two square patterns of 4 holes: 34.00 × 34.00 and 28.00 × 28.00. The drawing marks them "R=4* 1.30mm"
  and "R=4* 1.40mm". Read as radii, those are holes of about 2.6 and 2.8 mm (M2.5 clearance). Measure one, or check
  the STEP, before printing a mount.
- The USB connector's footprint is 7.56 long × 3.81 deep, starting 8.24 from the left edge and 3.47 from the bottom
  edge as drawn.
- The stock lens is the M27280M07S: effective focal length 2.8, F2.8, M12 × 0.5 thread, head Ø14 ± 0.05, barrel
  13.2 long (4.3 head, 8.9 thread), 15.86 from the lens front to the image plane, lens holder height 7, 4 g,
  minimum object distance 0.3 m.
- The 650 nm IR-cut filter is part of the lens (construction 2G2P+IR), not the board. A replacement lens must carry
  its own, and the camera needs refocusing after any lens change.
- **CAD:** Arducam's official STEP of the bare board is [B0385.STEP][step] (26 MB; HTTP 200 on 2026-11-10), linked
  as "3D Drawing" on [Arducam's product page][arducam]. The fallback is goBILDA's [STEP of the cased version][gbstep],
  which has the board as its own part: delete the case, stickers and screw. No community model turned up on GrabCAD
  or Onshape.

## Wiring and cables

- **Connector:** JST ZH-series 4-pin, 1.5 mm pitch (datasheet "Connector No." S4B-ZR(LF)(SN)). The drawing labels
  the pins left to right VCC, DM (D−), DP (D+), GND, but doesn't mark which is pin 1. The numbering pin 1 = +5 V,
  2 = D−, 3 = D+, 4 = GND comes from other Arducam UVC datasheets and the Delock 96001 cable, so it is unverified
  for the B0385. The mating housing is probably JST ZHR-4, a name given for a different Arducam board (unverified).
  PicoBlade and JST GH cables (1.25 mm pitch) look alike but don't fit.
- **Stock cable:** 1 m, to USB-A. The datasheet lists 2, 3 and 5 m as options.
- **USB port:** use the Control Hub's USB 3.0 port. FTC's [Control Hub ports page][ports] (2026-11-10) says:
  - the USB 3.0 port "is primarily used for connecting USB video device class (UVC) cameras";
  - "Ensure that you plug USB devices, such as a Camera, into the USB 3.0 Port";
  - "An electrostatic discharge (ESD) event on the USB 2.0 port on the Control Hub can cause Wi-Fi disconnects";
  - the USB-C port "can also be used with a UVC Camera".

  The page gives no current limit for either port. If the webcam or the Limelight drops out on a shared USB hub,
  switch to a powered USB hub. The 2026-10-09 bench ran both on one USB hub in the USB 3.0 port
  ([hub-bench-findings.md](hub-bench-findings.md)).

### Spare and shorter cables

- **The replacement is Arducam CB043**, "4pin 1.5mm Pitch USB Cable for UVC Cameras - 1m/3.3ft". It costs
  [$1.99 at UCTRONICS][cb043] (Arducam's parent company's US store; stock not stated on 2026-11-10). CB028, the same
  connector at 2 m, is [$2.99][cb028] ("Only 30 left" on 2026-11-10), but its page doesn't say it is for UVC cameras.
  Neither page names the USB end or the pinout. Overseas sellers of CB043, with lead times of weeks to months:
  [Antratek][antratek] (NL, 2026-11-08) and Welectron (DE, 2026-11-07).
- **CB043 is very probably the stock cable** (photos compared 2026-11-08). The B0385's product photos on UCTRONICS
  and Antratek's CB043 photo show the same black cable, USB-A plug, small white 4-pin connector, and red, black, green
  and white wires. Both match the datasheet's connector and 1 m length. UCTRONICS's own CB043 photo is a generic stock
  image, so it proves nothing.
- **Check the pin order before plugging a new cable into a hub.** Photos can't show it. With a multimeter, find
  which pins of the small connector USB-A pin 1 (+5 V) and pin 4 (GND) reach, and check they mate with the board's
  VCC and GND, the two outer pins of its header (datasheet drawing). The connector is keyed, so it can't go in
  backwards, but a cable wired the other way round would still swap them.
- **Nothing ready-made is shorter than 1 m.** UCTRONICS sells only 1 m and 2 m, and no other JST ZH 4-pin to USB-A
  cable under 1 m was found (2026-11-08). Options, best first:
  1. Cut a CB043 to length and resplice it with heat shrink. Keep the green and white data pair twisted right up to
     the joint, since USB 2.0 high speed needs it.
  2. Build one from a pre-crimped JST ZH 4-pin lead and a USB-A pigtail, wiring 5 V to VCC, D− to DM, D+ to DP and
     GND to GND. Loose single wires carry high-speed USB less reliably than a twisted pair.
  3. Coil the extra length and tie it down.

  Entaniya sells a [100 mm cable][entaniya] with a ZH1.5-4P connector, "Tested with cameras from Arducam", but its
  page doesn't say what is on the other end, the pinout or the price. Delock 96001 is a ready-made 4-pin 1.5 mm
  USB 2.0 camera cable, but it is 1.5 m long.

## Buying the camera

| where | price | stock |
|---|---|---|
| [UCTRONICS][uc-b0385] | $59.99 | "Contact US", no stock statement (2026-11-10); out of stock on 2026-11-08 |
| [Arducam][arducam] | $59.99 | "Contact Us" button, no cart (2026-11-08) |
| [goBILDA 3122-0004-0001][gb] (same board, cased) | $74.99 | Sold Out (2026-11-10); in stock 2026-11-06 and 2026-11-08 |
| DigiKey, SparkFun, Amazon US | | not carried (searched 2026-11-08) |
| Core Electronics (AU), Welectron, Fabtolab, Desertcart | | out of stock or overseas (2026-11-08) |

Arducam's [distributor page][dist] lists DLS Corporation, SparkFun, Percept Systems (Canada) and Vilros for North
America, plus Amazon, UCTRONICS, DigiKey and RobotShop.

**goBILDA's cased version** ([product page][gb], 2026-11-10) is the same 70° OV9782 board. It is USB 2.0 UVC with a
USB-A plug and an 875 mm cable, weighs 76 g with hardware and cable, and has case holes on an 8 mm pattern. The
case is about 70.3 × 51.1 × 36.7 mm (from its STEP), against the bare board's 38 × 38. That is too big for the
robot, so the decision on 2026-11-06 was to keep the bare board. goBILDA also publishes a [STEP][gbstep], a
[calibration][gbcal] and a [user guide][gbguide].

## Wider cameras compared

Verdict on 2026-11-06: keep the 70° B0385. The reasons are computed from datasheet FOVs and resolutions, not tested:

- **A wider camera puts fewer pixels on each tag.** Averaged across the width, the B0385 gets 18.2 px/° at
  1280×800 and 10.5 px/° at our 640×480. A 1080p fisheye spread over about 170° gets about 11 px/°.
- **Tags shrink and the usable range drops by the same factor.** For a rectilinear lens at the image centre, going
  from 70° to 100° at the same resolution shrinks a tag to 0.59× its size (0.35× the area), and going to 96°
  shrinks it to 0.63×. The distance at which tags still pass the 120 px² filter shrinks by the same factor, and
  beyond it the HIVE reads TIPPED.
- **A fisheye distorts tags near the edges**, so fewer of them are detected there, and SDK pose ignores the
  distortion ([above](#lens-distortion-calibration-and-pose)).
- **Raising the resolution to win pixels back** costs Control Hub CPU per frame.
- **The one gain** is keeping the HIVE in frame from more positions, which matters because a missing HIVE reads
  TIPPED. If that is ever needed, aim for 90–100° with low distortion.

### Lens swap: the best route to 90–100°

Of the complete global-shutter UVC cameras checked on 2026-11-06 (Arducam, ELP, Innomaker, Spinel, e-con, Kayeton
and Waveshare), none has a published low-distortion 90–100° lens at a high frame rate out of the box; the closest,
Spinel's UC20MPA, runs 30 fps on stock firmware ([table below](#complete-cameras-checked)). The best option is to
swap the B0385's lens for Commonlands [CIL018-F2.8-M12B650][cil018] (1.8 mm, f/2.8, 650 nm IR-cut), $39.00 and in
US stock on 2026-11-10.

- Commonlands' [OV9782 table][cl9782] gives it 96° H / 68° V / 107° D on this sensor, marked "Covers". An ideal
  1.8 mm rectilinear lens would give 93.7° H.
- Its distortion is −14.0% over a 6.8 mm image circle (type not stated). Back-calculated from Commonlands' FOV
  figures, that is about −4% at the OV9782's horizontal edge and −7% in the corners, so calibrate it before
  trusting tag shape near the edges.
- Buy the M12B650, not the M12BNIR: the IR-cut has to come with the lens.
- Fit risks:
  - its chief ray angle is 14°, against the OV9782's 26.78° ([OmniVision brief][ovpb]), so expect some corner
    shading or colour shift;
  - by Commonlands' drawing it is about 11 mm longer and 3.5 mm wider than the stock lens;
  - Commonlands says it needs a 9–13 mm tall holder, and the stock lens's spec gives a 7 mm holder, so whether it
    reaches focus in the stock holder is unverified;
  - check whether the stock lens is glued in.
- Refocus and recalibrate after the swap. Tags shrink to about 0.63×, so range drops by about a third at the same
  resolution.
- Other lenses on the same table: CIL023 (2.2 mm, low distortion) is 84° H, too narrow. CIL821 (2.1 mm, 96°) and
  CIL822 (2.4 mm, 92°) are sold as wide-angle, not low-distortion. Arducam LN014 (2.1 mm, <1%) computes to about
  85° H.

### Complete cameras checked

From maker pages on 2026-11-06. HFOV is computed unless noted. None of these was bought or tested.

| camera | sensor | HFOV | price | catch |
|---|---|---|---|---|
| [ELP-USBGS1200P01-H100][elp1200] | AR0234 colour, 1920×1200, MJPEG 90 fps in every mode | maker 101°; 91.6° for a rectilinear 2.8 mm lens | $74.80 | no distortion or IR-cut stated; ELP's FOV numbers disagree across pages; 4:3 modes likely crop to 81–90° |
| [ELP-USBGS800P03][elp800], H120 1.8 mm lens | OV9281 mono, MJPEG only | 94–98° | $48.29 | its 640×480 at 210 fps is probably a crop |
| [Spinel UC20MPA][spinel], low-distortion 2.1 mm lens (UC20MPA_LD2124) | OG02B10 colour | 98–101° | $85 | stock firmware 30 fps; high-fps firmware is a custom order |
| [Waveshare OV9281 USB Camera (A)][waveshare], SKU 31671 | OV9281 mono | 99–105° at 1280×720 | $26.99 | <10% distortion; UVC not stated |
| [e-con See3CAM_24CUG][econ] | AR0234 | maker 104.6° | $199 ($249 enclosed) | stock lens visibly barrel-distorted |

Rejected outright:

- Arducam B0332 (OV9281) and B0322 (OV2311) are 70–75° H.
- Arducam's 130° D OV9281 is wide-angle, not low-distortion.
- Arducam B0495 (AR0234) runs only 960×600 at 10 fps on USB 2.0.
- Innomaker U20CAM-9281M is a 148° H fisheye.
- Kayeton KYT-U100-GS1ML is about 85°.

The FTC docs' "ELP Global Shutter" row is a mono 70° camera, not the colour H100, so that listing says nothing about
the H100.

## Still unknown

- **Whether the cell-tip verdict works on a real HIVE.** Test: on a real or full-size HIVE, record each cluster's
  visibility and roll from the stream overlay in Cell Tip Test. Do it in both stable states, from every planned
  shooting spot, while tipping the HIVE by hand. Note whether a CELL can come to rest on its side.
- **Which exposure stops tags dropping out while turning.** Test: run Cell Tip Test with the robot turning at match
  speed, stepping `exposureMs` down from 8 and `gain` up until the verdict holds.
- **The bare B0385's vid/pid, and whether its 640×480 crops the same way.** Test: read its USB vendor and product
  IDs with it plugged into a computer, then grab the same scene at 640×480 and 1280×800 and compare the edges.
- **The Control Hub's per-frame cost at 1280×800.** Test: Bench: Webcam times `CellTipPipeline` at 640×480 only, so
  rerun it with its own and `WebcamSession`'s `WIDTH`/`HEIGHT` set to 1280×800.
- **Whether 640×400 MJPEG exists.** Test: ask `startStreaming` for it.
- **The connector's pin 1 and CB043's pinout.** Test: the continuity check above.
- **The mounting-hole diameter.** Test: measure a hole, or read it from the Arducam STEP.
- **Whether the CIL018 lens focuses in the stock 7 mm holder.** Test, only if a wider view is wanted: buy one and try.

[ds]: https://core-electronics.com.au/attachments/localcontent/B0385_OV9782_Global_Shutter_UVC_Camera_Datasheet_19190316e78.pdf
[qs]: https://core-electronics.com.au/attachments/localcontent/B0385_Manual_218018a0904.pdf
[step]: https://download.arducam.com/3D-Drawing/B0385.STEP
[arducam]: https://www.arducam.com/100fps-global-shutter-color-usb-camera-board-1mp-ov9782-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-for-computer-laptop-android-device-and-raspberry-pi-arducam.html
[dist]: https://www.arducam.com/distributors/
[uc-b0385]: https://www.uctronics.com/arducam-global-shutter-color-usb-1mp-ov9782-uvc-webcam-module.html
[cb043]: https://www.uctronics.com/4pin-1-5mm-usb-cable-1m.html
[cb028]: https://www.uctronics.com/4pin-1-5mm-usb-cable-2m.html
[antratek]: https://www.antratek.com/arducam-cb043
[entaniya]: https://e-products.entaniya.co.jp/en/?p=9098
[gb]: https://www.gobilda.com/global-shutter-usb-camera-with-gobilda-case-100-fps/
[gbstep]: https://www.gobilda.com/content/step_files/3122-0004-0001.zip
[gbcal]: https://www.gobilda.com/content/downloads/3122-0004-0001_calibrations.zip
[gbguide]: https://www.gobilda.com/content/user_manuals/3122-0004-0001_user_guide.pdf
[ports]: https://ftc-docs.firstinspires.org/control_hard_compon/rc_components/hub/ports/ch-ports.html
[ftcwebcams]: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_webcams/visionportal-webcams.html
[cil018]: https://commonlands.com/products/low-distortion-m12-lens
[cl9782]: https://commonlands.com/pages/image-sensors/ov9782
[ovpb]: https://ovt.com/wp-content/uploads/2025/03/OV9782-PB-v1.0-WEB.pdf
[elp1200]: https://www.webcamerausb.com/elp-high-frame-rate-90fps-1080p-global-shutter-usb-camera-23-megapixel-1920x1200-ar0234-color-cmos-distortion-free-usb20-camera-module-100degree-lens-p-436.html
[elp800]: https://www.webcamerausb.com/elp-global-shutter-usb-camera-module-800p-120fps-480p-210fps-240p-420fps-120p-640fps-high-speed-usb-camera-board-with-no-distortion-lens-monochrome-usb-webcam-plug-paly-for-industrial-machines-p-450.html
[spinel]: https://www.spinelelectronics.com/product/uc20mpa/
[waveshare]: https://www.waveshare.com/ov9281-1mp-usb-camera-a.htm
[econ]: https://www.e-consystems.com/industrial-cameras/ar0234-usb3-global-shutter-camera.asp
