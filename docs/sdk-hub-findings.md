# SDK and hub findings

What FTC SDK 12.0.0 and the REV hubs do underneath motor, servo and sensor calls, bulk reads, threads and the OpMode
lifecycle. Read it before writing a hardware wrapper, a sensor thread or voltage compensation, or before trusting an
encoder velocity read from a hub.

"Source" below means the SDK 12.0.0 `Hardware` and `RobotCore` sources jars (`org.firstinspires.ftc` on Maven Central,
the version `TeamCode/build.gradle` pins), cited by class and method because SDK line numbers move between releases.
Recheck these after an SDK bump. Our code is cited by path under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

## Act on these

- **Every OpMode starts from reset hardware.** Set bulk caching, run mode, zero-power behaviour, direction and any servo
  PWM range in each OpMode's INIT: nothing carries over, auto to TeleOp included. Servos are limp and motors float
  from INIT until the first write, and `EnhancedOpMode` doesn't write in INIT
  (`architecture/core/EnhancedOpMode.java:83`). An OpMode that must hold a mechanism or preload through INIT overrides
  `shouldWriteDuringInit()`; writes then start about 500 ms into INIT (`:159`).
- **Hub encoder velocity can't report more than 32,767 counts/s**, 240 rpm on an 8192 counts/rev Through Bore, and
  `setVelocity()` clips its target to that without an error. Compute fast speeds from position (CLAUDE.md, Flywheel
  RPM).
- **A failed hub read returns zeros instead of throwing**: bulk data, ADC reads (battery reads 0.0 V) and I2C reads
  alike. Guard anything that divides by a reading or acts on a sudden zero. `WriteCache` already ignores a battery
  reading under 6 V (`architecture/hardware/WriteCache.java:16`).
- **Hub commands on one USB link go one at a time**, whatever thread sends them through the SDK. A Control Hub and an
  Expansion Hub on RS485 are one link, so a second thread can't overlap their traffic; it only adds waiting.
- **The SDK skips an identical write for 500 ms, even when the earlier send failed**, so a lost write reaches the hub
  again only after 500 ms, whatever a wrapper's re-send period.
- **Every motor in our hardware configs is the SDK's 99.5:1 motor type.** On any other motor, angle-unit velocities
  and RUN_USING_ENCODER `setPower()` are scaled wrong; tick counts and `getVelocity()` in ticks/s are right.

## Encoder velocity

| | value | how known |
|---|---|---|
| hub velocity format | signed 16-bit counts/s per port, in the bulk read | source: `LynxGetBulkInputDataResponse` |
| range | −32,768 to 32,767 counts/s | same |
| 8192 counts/rev (REV Through Bore) | correct only below 240 rpm | arithmetic |
| 28 counts/rev (goBILDA 1:1 motor encoder) | limit at 70,215 rpm, never reached | arithmetic |
| `setVelocity()` target | rounded, then clipped to ±32,767 counts/s, no error | source: `setMotorVelocity` |

- `setVelocity()` (`LynxDcMotorController.setMotorVelocity`) also switches a motor in RUN_WITHOUT_ENCODER or
  STOP_AND_RESET_ENCODER to RUN_USING_ENCODER, and every call sends a command: it is never deduplicated.
- In RUN_USING_ENCODER or RUN_TO_POSITION, `setPower(p)` becomes a velocity target of p times the motor type's
  achievable maximum, with no clip. If that target is over 32,767 counts/s, `setPower()` throws
  `IllegalArgumentException` ("illegal velocity") to the caller: the command is built outside the SDK's try/catch
  (source: `internalSetMotorPower`, `LynxSetMotorTargetVelocityCommand`). Only a motor type whose achievable maximum
  is over 32,767 counts/s can hit this.
- Above 32,767 counts/s, whether the hub's value wraps modulo 65,536 or saturates is unverified. Road Runner's
  `OverflowEncoder` (`inverseOverflow` in road-runner-ftc v0.1.25's
  [`Encoders.kt`](https://github.com/acmerobotics/road-runner-ftc/blob/v0.1.25/RoadRunner/src/main/java/com/acmerobotics/roadrunner/ftc/Encoders.kt),
  fetched 2026-11-10) assumes it wraps and that the value is always a multiple of 20 counts/s "due to Expansion Hub's
  50ms measurement window". No REV or FIRST page says either. gm0 says only that the hub "calculates velocity by using
  a 5 value “ring buffer”, which has a new value added to it every 10 ms"
  ([gm0](https://gm0.org/en/latest/docs/software/adv-control-system/sdk-motors.html), fetched 2026-11-10).
- Road Runner's unwrap snaps to the nearest multiple of 5 × 65,536, so its position-based estimate must be within
  ±163,840 counts/s of the true speed, and both assumptions above must hold.
- Don't copy SolversLib 0.3.6's `Motor.Encoder.getCorrectedVelocity()`: it is wrong even with a perfect estimate. Every
  correctly reported velocity below −16,384 counts/s comes back positive (−20,000 becomes +45,536), overflowed values
  are often left uncorrected, and its estimate updates only inside `getPosition()`, so without that call nothing is
  corrected (read from `com.seattlesolvers.solverslib.hardware.motors.Motor`, 0.3.6 sources jar). Neither Road Runner
  nor SolversLib is a dependency here.
- OctoQuad velocity is signed 16-bit too, but in counts per sample interval: the SDK's `OctoQuad` driver allows
  1–255 ms, and the firmware default is 50 ms (OctoQuad FTC Edition specification Rev 3.0C, sections 6.5.12 and
  8.5.4, in [DigitalChickenLabs/OctoQuad](https://github.com/DigitalChickenLabs/OctoQuad)). At 8192 counts/rev a 50 ms
  interval overflows above about 4800 rpm; 6000 rpm needs 39 ms or less. The interval is lost at power-off unless
  `saveParametersToFlash()` is called. Its read cost: [`wiring-findings.md`](wiring-findings.md).

## Encoder ports

- "Encoder ports 0 and 3 are hardware-counted, but ports 1 and 2 are software-counted"
  ([ftc-docs hub tips](https://ftc-docs.firstinspires.org/tech_tips/tech-tips/tech-tip-hub-tips/tech-tip-hub-tips.html),
  fetched 2026-11-10). gm0 adds that encoders over about 4000 counts/rev should not go on ports 1 and 2, which can
  lose counts at high speed
  ([gm0](https://gm0.org/en/latest/docs/software/adv-control-system/control-system-internals.html), fetched
  2026-11-10). Where to put a flywheel encoder: CLAUDE.md, Flywheel RPM.
- No REV or FIRST page found gives a maximum count rate for any port. The DECODE shooter's shot table goes to 3275 rpm
  (`decode/modules/ShooterInterpolation.java:169`) on an 8192 counts/rev encoder (`decode/modules/Shooter.java:21`) on
  Expansion Hub encoder port 3 (`TeamCode/src/main/res/xml/cuttle_decode.xml`), about 447,000 counts/s; lost counts
  there were never checked.

## Bulk reads

- In MANUAL mode the first bulk-eligible read after `clearBulkCache()` does one bulk read, and every later read returns
  that snapshot until the next clear. Bulk-eligible: encoder position and velocity, `isBusy()`, `isOverCurrent()`,
  analog inputs and digital inputs. Motor current, battery voltage and I2C are always separate commands (source:
  `LynxModule.recordBulkCachingCommandIntent` and its callers in the `Lynx*Controller` classes).
- With caching OFF, which the per-OpMode reset sets, every `getVelocity()` is a full bulk read (source:
  `LynxDcMotorController.internalGetMotorTicksPerSecond`). `EnhancedOpMode` sets MANUAL every INIT
  (`architecture/core/EnhancedOpMode.java:396-401`).
- A failed bulk read (timeout, NACK or exception) doesn't throw: the SDK caches an all-zero placeholder, so encoders,
  velocities and analog inputs read 0, digital inputs read false and `isBusy()` reads true until the next clear. Only
  `ForceStopException` is rethrown (source: `LynxModule.getBulkData`, `LynxCommExceptionHandler`).
- A timeout logs `timeout: abandoning waiting` once and marks the hub unresponsive. Until it answers again, every
  command to it is NACKed at once, logcat gets "... is currently unresponsive." at most every 2 s, and the Driver
  Station shows a global warning that the hub is "not currently responding to commands". Battery-low NACKs and
  interrupts leave no trace (source: `LynxRespondable.awaitAndRetransmit`, `LynxModuleWarningManager`,
  `LynxCommExceptionHandler`).
- In code, `BulkData.isFake()` is the only sign, and only `LynxModule.getBulkData()` hands out a `BulkData`. To check
  it, call `hub.getBulkData()` at the start of the loop instead of `clearBulkCache()`: it clears, reads once and fills
  the cache the motor and sensor reads then use, for the same single bulk read. Do it only on hubs whose inputs the
  loop reads, since it always reads: an unneeded Expansion Hub bulk read costs about 4.7 ms
  ([`loop-time-findings.md`](loop-time-findings.md)). `EnhancedOpMode` clears
  (`architecture/core/EnhancedOpMode.java:403`) and never checks.

## What writes send, skip and repeat

- `setPower()` in RUN_WITHOUT_ENCODER is clipped to [−1, 1], scaled to ±32,767 and truncated toward zero (source:
  `LynxDcMotorController.internalSetMotorPower`).
- The SDK skips a `setPower()` equal to its last-known value while that value is under 500 ms old (`LastKnown`, 500 ms
  default). After that the same value goes out again with `DcMotorImpl.setPower()`'s mode query and the channel-enable
  command: up to three commands. That is the ~500 ms loop spike in CLAUDE.md, Dependencies.
- The last-known value is recorded before the send and kept when the send fails, so after a NACKed or timed-out write
  the same value is skipped until 500 ms have passed. `WriteCache` re-sends an unchanged value once 250 ms have passed
  since its last write (`architecture/hardware/WriteCache.java:4`), so a lost motor write reaches the hub again after
  500 ms plus up to two loop periods; no shorter re-send period gets it there before 500 ms.
- A `setMode()` that changes the mode, or repeats it more than 500 ms later, re-sends the current power at once; the
  per-OpMode reset makes the SDK forget its last-known values. `setVelocity()` is never skipped.
- Servos: `setPosition()` is skipped the same way. Every pulse-width write re-enables PWM
  (`LynxServoController.internalSetPulseWidth`), and `setPwmDisable()` makes the SDK forget the last position, so the
  next `setPosition()` goes out even if unchanged (source: `LynxServoController.internalSetPwmEnable`). A wrapper's
  own write cache must forget its value too, as `EnhancedServo.setPwmDisable()` does
  (`architecture/hardware/EnhancedServo.java:63-67`).

## Battery voltage and other ADC reads

- `VoltageSensor.getVoltage()` and `LynxModule.getInputVoltage()` each send one ADC command per call, never cached and
  not part of the bulk read; so do motor current, hub current, I2C and GPIO bus current, auxiliary voltage and
  temperature (source: `LynxVoltageSensor`, `LynxModule`). A failed one returns 0. Costs:
  [`hub-bench-findings.md`](hub-bench-findings.md), Hub costs.
- `EnhancedOpMode` reads the battery every 50 loops (`architecture/core/EnhancedOpMode.java:38`, `:407-410`).

## Timestamps

- A bulk snapshot has no hub sample time: the payload has no time field, and the host receive time on the response
  (`LynxMessage.getPayloadTimeWindow()`) is hidden inside `BulkData`. To time samples, send your own
  `LynxGetBulkInputDataCommand` (as hub-reading threads already do) and read its response's `getPayloadTimeWindow()`.
- `LynxMessage.getNanotimeLastTransmit()` is `System.nanoTime()` just after the USB write returns, overwritten on a
  retransmit, and only reachable on command objects you build yourself (source: `LynxUsbDeviceImpl.transmit`).
- I2C reads are already stamped with their USB receive time (`TimestampedData.nanoTime`, set in
  `LynxI2cDeviceSynch`).

## Threads and the USB link lock

- Every Lynx command holds its USB link's transmit lock (`lynx xmit lock`) from send until its ACK or response (source:
  `LynxRespondable`, `LynxUsbDeviceImpl.networkTransmissionLock`). There is one lock per USB device: a Control Hub and
  an RS485 Expansion Hub share one, and an Expansion Hub on its own USB cable has its own.
- So a thread's hub read makes the OpMode thread's next command to either hub on that link wait. Threads overlap hub
  traffic only across separate USB links. Waiting threads queue in arrival order. What a 10 ms poller costs a loop:
  [`hub-bench-findings.md`](hub-bench-findings.md), Analog refresh and poller threads.
- A waiting command takes the lock anyway once its owner has held it 500 ms (`#### abandoning lock` in logcat). An
  interrupt that lands in `MessageKeyedLock.release` leaves the lock held until then: measured 2026-10-24 on a bare
  Control Hub with SDK 12.0.0, 7 of 1176 interrupts did, each stalling the next command 505 ms against a 2.68 ms read
  p50; interrupts before the send or while awaiting the response leaked nothing. This is why hub-reading threads never
  use `interrupt()` (CLAUDE.md, Testing). LinearOpMode's own STOP interrupt can leak it too
  ([`photon-bench.md`](photon-bench.md)).

## Motor hardware

- REV rates each Expansion Hub motor port at 10 A continuous and 20 A absolute maximum. "The outputs will self protect
  once they approach their thermal limit", and the battery fuse caps the total
  ([REV Expansion Hub basics](https://docs.revrobotics.com/duo-control/control-system-overview/expansion-hub-basics),
  fetched 2026-11-10).
- SDK 12 has no current limit. `DcMotorEx.setCurrentAlert()` sets the level behind `isOverCurrent()`, a flag in the
  bulk read; the hub command set has no limit command (source: `LynxSetMotorChannelCurrentAlertLevelCommand` and the
  `commands/core` package). Whether the firmware does anything else with that level is unknown.
- BRAKE and FLOAT are defined for zero power only: "a motor's behavior when a power level of zero is applied"
  (`DcMotor.ZeroPowerBehavior` javadoc). What the hub's driver does at partial power isn't published by REV or FIRST.
- All 32 motor entries in `TeamCode/src/main/res/xml/` are `goBILDA5202SeriesMotor`, which SDK 12 defines as the 99.5:1
  gearmotor: 2786 ticks/rev, 60 rpm (`com.qualcomm.hardware.motors.GoBILDA5202Series`). A goBILDA 5203 1:1 motor runs
  6000 rpm with "28 PPR at Output Shaft", which the hub counts as 28 counts/rev
  ([goBILDA 5203 1:1](https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/),
  fetched 2026-11-10). On any motor that isn't 99.5:1, `getVelocity(AngleUnit)`, `setVelocity(rate, AngleUnit)` and
  RUN_USING_ENCODER `setPower()` are scaled wrong; `getCurrentPosition()` and `getVelocity()` in ticks/s are fine. No
  team code uses the angle-unit methods or RUN_USING_ENCODER today (`EnhancedMotor` only passes them through).

## OpMode lifecycle

Before every OpMode's INIT except the SDK's built-in stop OpMode, iterative or linear, the SDK resets the hubs, then
every other device (source: `OpModeManagerImpl.resetHardwareForOpMode`):

| what | reset to |
|---|---|
| bulk caching | OFF |
| motors | RUN_WITHOUT_ENCODER, power 0, FLOAT, direction FORWARD, PIDF re-applied, last-known values forgotten |
| servos | PWM off (limp), PWM range default, scale range 0–1, direction FORWARD |
| CR servos | direction FORWARD |
| digital pins | input |
| not reset | encoder counts, target position, current-alert level |

- **Gamepads:** an iterative OpMode copies the latest Driver Station snapshot into `gamepad1`/`gamepad2` before every
  `init_loop()`, `start()`, `loop()` and `stop()`, but not before `init()` (source: `OpMode.internalPreUserCode`), so
  anything code writes into them is overwritten on the next call. A LinearOpMode copies on arrival, from the event-loop
  thread (`LinearOpMode.newGamepadDataAvailable`). Edge detection resets at INIT and at START.
- **Telemetry:** the SDK calls `telemetry.update()` after every `init_loop()`, `start()`, `loop()` and `stop()`, not
  after `init()` (`OpMode.internalPostUserCode`). The `EnhancedOpMode` rule: CLAUDE.md, OpMode lifecycle.
- **Overhead:** an iterative OpMode sleeps 1 ms (`Thread.sleep(1)`) after every `init_loop()` and `loop()`
  (`OpMode.internalRunOpMode`), and `EnhancedOpMode` extends `OpMode`, so every framework loop pays it. The
  empty-OpMode loop time is in [`hub-bench-findings.md`](hub-bench-findings.md), Hub costs.
- **No time limit while running:** `init()`, `init_loop()`, `start()` and `loop()` can block as long as they like; the
  `msStuckDetect*` fields are deprecated: "OpModes no longer have a time limit". Only STOP has one
  ([`hub-bench-findings.md`](hub-bench-findings.md)).
- **STOP:** before calling `stop()`, the SDK sends a fail-safe command to every hub, then refuses dangerous commands:
  non-zero motor power or velocity, a target position, a servo or PWM-output pulse width, and enabling a servo or PWM
  output. Each is NACKed inside the SDK (CANCELLED_FOR_SAFETY) without being sent or raising an error; the only trace
  is a verbose `nack rec'd` logcat line (source: `OpModeManagerImpl.callActiveOpModeStop`, `LynxModule.sendCommand`,
  each command's `isDangerous()`, `LynxRespondable.onNackReceived`). Zero power, mode and zero-power-behaviour
  changes, motor enable, reads and I2C still go out. The rule this gives: CLAUDE.md, Module pattern.

## Driver Station version warning

- The Robot Controller raises "The Driver Station app is obsolete. You should install the new version for this FTC
  season." when the DS app's SDK build month is before August of this year (before August of last year while it isn't
  yet October), or comes from an earlier FTC season than the Robot Controller's. A DS from a later season marks the
  Robot Controller obsolete instead (source: `SoftwareVersionWarningSource`, `AppUtil.appIsObsolete`; text from
  RobotCore 12.0.0 `res/values/values.xml`).
- Different SDK major or minor versions give "The Robot Controller and Driver Station app versions do not match"
  instead; only one of the two shows, obsolete first.
- On a Control Hub the check waits until the hub's clock is sane, which is after a DS connects (CLAUDE.md, Build &
  deploy). The Robot Controller's advanced setting "Obsolete software warning" hides it.
- Fix it by installing the Driver Station app from the robot's SDK release (12.0, `TeamCode/build.gradle`) on every
  Driver Station.

## Device drivers

**goBILDA Pinpoint** (the SDK's `GoBildaPinpointDriver`, as Pedro 3.0.2's `PinpointLocalizer` builds it):

- Error detection defaults to `LOCAL_TEST`, and neither Pedro nor our code changes it. On device version 3 or later
  (the bench's Pinpoint is firmware 4, [`hub-bench-findings.md`](hub-bench-findings.md)), `update()` reads status, loop
  time, encoders, pose and velocity in one I2C read with no CRC.
- A field that is NaN, a position that jumps more than 5000 mm, a heading that jumps more than 120 rad, or a velocity
  over 10,000 mm/s (120 rad/s for heading) keeps its previous value, and the whole status word becomes FAULT_BAD_READ,
  so `getDeviceStatus()` reports that instead of READY until the next read. The other fields of that read are still
  taken. The "no new data" check (loop time under 1) applies only to device versions 1 and 2.
- A failed I2C read (NACK, timeout or exception) returns all-zero bytes (`TimestampedI2cData.makeFakeData`); a NACK
  also raises the global warning "Error communicating with I2C device(s)". Zero bytes parse as status NOT_READY and
  position (0, 0), heading 0, which pass the jump check anywhere on the field: if the Pinpoint's reads fail mid-run,
  the pose snaps to the origin rather than freezing (read from source; not seen on a hub).
- Pedro's `PinpointLocalizer.update()` copies the pose without reading `getDeviceStatus()`, and only test and bench
  OpModes read the status, so no robot OpMode notices either case.

**Brushland laser rangefinder** (`vendor/brushland/LaserRangefinder.java`):

- `getScanDistance()` stores the status without the shift (`data[1] & 0xE0`, line 190) that `getDistance()` applies
  (`>> 5`, line 169). After a scan read, `getStatus()` returns 0, 32, … 224 instead of 0–7 (0 good, 1–2 okay, higher
  bad). No team code uses scan mode. If you do, shift the status right by 5 at the call site, since the vendored file
  isn't edited (CLAUDE.md, Conventions), and report it to Brushland.
- `write()` sleeps 10 ms after every register write (lines 198–205): configure the sensor in INIT, never in a loop.

## Still unknown

- **Hub velocity above 32,767 counts/s, and its multiples of 20:** spin a Through Bore on port 0 or 3 past 240 rpm and
  log raw `getVelocity()` next to velocity from position. It wraps if the raw value tracks the true value minus a
  multiple of 65,536; check `v % 20 == 0` on every sample.
- **The highest count rate ports 0 and 3 keep up with:** run a Through Bore at rising speeds and compare its rpm from
  position against an independent speed reference until they part.
- **BRAKE at partial power:** coast down from the same speed at power 0.05 under BRAKE and under FLOAT.
- **Whether the firmware acts on the current-alert level:** stall a motor with the alert set low and log its current.
- **What a Pinpoint reads when unplugged mid-run:** the Pinpoint cable pull in
  `scripts/bench/run_bench.py --interactive`, which records the first non-READY status and pose.
- **What the hub's fail-safe leaves motors and servos in** (brake or float, servo PWM off or held): press STOP with a
  servo under load and a motor spinning, and watch both.
