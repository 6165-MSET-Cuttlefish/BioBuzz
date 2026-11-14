# Framework findings

How `EnhancedOpMode` and `Module` show faults, deliver bound setpoints, order Modules and sample battery voltage, and
which unused-looking code is kept on purpose. Read it before changing `OptimizationToggles` for a match, feeding a
Module a setpoint computed every loop, relying on one Module running before another, reading `BatteryVoltage`, or
deleting code as unused.

Everything here was read from source on 2026-11-10 unless marked otherwise. Our paths are under
`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`; SDK classes are from FTC SDK 12.0.0's `Hardware` sources.

## What to do

- **Keep `OptimizationToggles.dsTelemetry` on for matches.** With it off, no fault reaches the Driver Station. To save
  loop time, raise `dsTransmissionIntervalMs` instead ([details](#faults-and-driver-station-telemetry)).
- **Pass a setpoint computed every loop to `write()` through a plain field, not `bindTunable`**, which delivers it a
  loop late ([details](#setpoints-through-bindtunable-arrive-a-loop-late)).
- **Don't depend on the order of Modules found in the same discovery pass**: it comes from `getDeclaredFields()`,
  which has no defined order. To force an order, create the later Module in `initialize()` or hold both in one array
  or `List` field ([details](#module-order)).
- **Treat `BatteryVoltage.get()` as up to 49 loops old, and give any new reader `WriteCache`'s guard against a
  reading under 6 V** ([details](#battery-voltage)).
- **Don't delete the pieces listed under [kept on purpose](#kept-on-purpose)** as dead code.

## Faults and Driver Station telemetry

A *fault* is an entry in `core/Faults`, such as the `Limelight` or `Webcam` fault. A *DS frame* is a loop whose Driver
Station lines will be sent: at most one per `dsTransmissionIntervalMs` (250 ms by default), and none while
`dsTelemetry` is off.

| where a fault shows | when | source |
|---|---|---|
| Driver Station: big red `FAULT <key>: <message>` line | only in a DS frame, so never with `dsTelemetry` off | `architecture/telemetry/DualTelemetry.java:89-91`, `:158-165` |
| dashboard telemetry: `FAULT <key>` row | while `dashboardTelemetry` is on | `DualTelemetry.java:100-111`, `:164` |
| dashboard field view: red `FAULT` text | always, every INIT and run loop | `architecture/core/EnhancedOpMode.java:168`, `:235`, `:523`, `:540-549` |
| logcat: tag `Fault` | when a key is raised while not already active | `architecture/core/Faults.java:36-40` |

- In a match the Driver Station is the only screen, so with `dsTelemetry` off a Limelight fault and the fallback it
  triggers happen with nothing shown.
- Both toggles default to on (`architecture/OptimizationToggles.java:22-23`) and are re-read every loop
  (`EnhancedOpMode.java:288`). They are statics, so a dashboard edit carries into every later OpMode until
  deploySloth or an app restart: one switched off while timing loops is still off in the next match unless someone
  switches it back.
- Raising `dsTransmissionIntervalMs` makes DS frames rarer, and with them the `updateTelemetry` spikes over 4 ms
  ([loop-time-findings.md](loop-time-findings.md)); a fault then reaches the Driver Station within that interval.
- A fault raised after the top of a loop reaches the telemetry lines from the next loop
  (`EnhancedOpMode.java:291-292`) and the Driver Station from the next DS frame. The field-view text is drawn at the
  end of the loop, so it shows at once.
- Crashes don't go through telemetry: a throwing hook is reported with `RobotLog.setGlobalErrorMsg`
  (`EnhancedOpMode.java:268-271`), which neither toggle affects.
- Turning DS telemetry off safely would need a code change: `DualTelemetry` would have to keep sending DS frames that
  carry only the fault lines while a fault is active.

## Setpoints through bindTunable arrive a loop late

`Module.bindTunable(state, supplier)` copies the supplier's value into the State only in `refreshTunables()`, which
`readModules()` runs just before each Module's `read()` (`architecture/core/Module.java:42-53`,
`EnhancedOpMode.java:379-387`). Each loop runs the reads, then `follower.update()`, `gameLoop()`, commands
(`Scheduler.execute()`) and the writes (`EnhancedOpMode.java:219-240`, `:287-315`). So a value that `gameLoop()` or a
command stores where a supplier reads it:

- reaches the State at the next loop's refresh and the hardware in that loop's `write()`: one loop late (15.3 ms mean
  in the Ivy Path Test log, [loop-time-findings.md](loop-time-findings.md));
- in an OpMode whose `shouldReadDuringInit()` returns false, isn't refreshed again after `initStates()` until the
  first loop after START, since the refresh is skipped with the reads (`EnhancedOpMode.java:309`). Only `DecodeTele`
  turns reads off today, and its modules bind no tunables.

What to do:

- Keep `bindTunable` for dashboard values (`Tuning.x`), where a loop of delay doesn't matter.
- For a setpoint computed every loop, store it in a field and use it in `write()`, as `Turret.setSotmAngles` does
  (`biobuzz/modules/Turret.java:87-91`, `:124-129`).
- Switching which State is active (`setState`) already lands in the same loop's `write()`; only the bound value lags.
- A value used in `read()` is a loop late however it arrives, because `read()` runs before `gameLoop()`. So is
  `follower.pose()` read there: `follower.update()` runs after the reads.
- `Shooter.setSotmTargets` goes through `bindTunable` (`biobuzz/modules/Shooter.java:190-193`, `:197-202`), so SOTM
  Test's RPM and hood targets are a loop late. What that and the read-to-write delay do to the flywheels:
  [flywheel-control-findings.md](flywheel-control-findings.md#where-our-loop-loses-accuracy).

## Module order

`EnhancedOpMode` builds its Module list in INIT, by reflection, in two passes, and never reorders it
(`EnhancedOpMode.java:123-128`, `:317-377`). Which Modules it finds at all: CLAUDE.md, OpMode lifecycle.

1. Pass 1, after `createRobot()`: the OpMode's own fields (its class first, then superclasses up to `EnhancedOpMode`),
   then `robot`'s. A non-Module object from our package found in a field has its fields searched depth-first; an
   array, `Iterable` or `Map` is searched in iteration order.
2. Pass 2, after `initialize()`: the same walk, appending only Modules not yet listed.

Reads, writes, module telemetry and `stop()` all follow that list (`EnhancedOpMode.java:379-398`, `:434-438`,
`:261-264`).

- **Guaranteed:** Modules reachable in pass 1 come before Modules created in `initialize()`. In a `BioBuzzOpMode`, the
  robot's `Drivetrain`, `CellTipCamera` and `Camera` (`biobuzz/BioBuzzRobot.java:14-16`, `:31-33`) come before SOTM
  Test's `Turret` and `Shooter` (`opmodes/test/SotmTest.java:68-69`).
- **Not guaranteed:** the order within one class's fields. It is `Class.getDeclaredFields()` order, whose elements
  "are not sorted and are not in any particular order"
  ([Android reference](https://developer.android.com/reference/java/lang/Class#getDeclaredFields()), fetched
  2026-11-10). In SOTM Test, `Turret` may read and write before or after `Shooter`; reordering field declarations is
  not a fix.
- **The only orders you control:** pass 1 before pass 2, and element order inside one array or `List` field.

What the order decides:

- **Which Module pays each hub's bulk read.** In MANUAL bulk caching the first bulk-eligible read of a hub after the
  clear does the bulk read and later ones use its snapshot ([sdk-hub-findings.md](sdk-hub-findings.md#bulk-reads));
  that costs about 2.2-2.5 ms on the Control Hub ([hub-bench-findings.md](hub-bench-findings.md)). `onLoopStart()`
  runs before the Module reads and `follower.update()` after them. The battery read never fills the cache
  ([below](#battery-voltage)).
- **How long a Module's output waits after its read:** every later Module's `read()`, `follower.update()`,
  `gameLoop()`, the commands and every earlier Module's `write()`. A changed motor or servo write costs about
  1.55-1.64 ms on the Control Hub ([hub-bench-findings.md](hub-bench-findings.md)).
- **The order of the MODULES group** in DS and dashboard telemetry, which makes the list order visible.

## Battery voltage

`BatteryVoltage.get()` is the value voltage compensation uses.

- `EnhancedOpMode` reads the first voltage sensor in the hardware map (`EnhancedOpMode.java:114`) once every 50 loops,
  INIT loops included (`:39`, `:150`, `:411-414`), and republishes that reading every loop (`:302-303`). START resets
  the count, so the first loop after START reads fresh (`:193`).
- The value is therefore up to 49 loops old: about 0.3 s at the bench's 6.3 ms idle loop
  ([hub-bench-findings.md](hub-bench-findings.md)), 0.75 s at Ivy Path Test's 15.3 ms mean
  ([loop-time-findings.md](loop-time-findings.md)). A sag shorter than that can fall between two reads unseen.
- The read is its own ADC command, never served from the bulk cache, and costs about 1.3 ms on the Control Hub
  ([hub-bench-findings.md](hub-bench-findings.md)): that is why it is throttled. A failed read returns 0.0 V instead
  of throwing ([sdk-hub-findings.md](sdk-hub-findings.md)), and that 0.0 then stands for up to 50 loops.
- `BatteryVoltage` is a static that starts at 12.0 (`architecture/hardware/BatteryVoltage.java:4`) and only
  `EnhancedOpMode` updates it (`EnhancedOpMode.java:303`). It keeps the previous OpMode's last reading until this
  OpMode's first INIT loop, so code reading it in a constructor, `initStates()`, `init()` or `initialize()` gets that,
  and a bare `OpMode` never updates it.
- The dashboard's `Voltage` row shows the same cached reading (`EnhancedOpMode.java:583`).

| reader | guard against a failed read | source |
|---|---|---|
| `EnhancedMotor.withVoltageCompensation(ref)`: power × ref / V, then clamped to ±1 | skips scaling below 6 V | `architecture/hardware/WriteCache.java:15-16`, `:32-37`; `architecture/hardware/EnhancedMotor.java:35` |
| `Shooter`'s flywheel headroom, min(1, V / 13) | none | `biobuzz/modules/Shooter.java:319` |

`withVoltageCompensation` is on both BioBuzz flywheels and both `decode/` flywheels, at 13 V
(`biobuzz/modules/Shooter.java:296`, `decode/modules/Shooter.java:87-88`). What the stale value and the missing guard
do to the flywheels: [flywheel-control-findings.md](flywheel-control-findings.md#where-our-loop-loses-accuracy).

## Kept on purpose

Nothing in this repo calls these (checked 2026-11-10). Decided 2026-10-18: they stay for reuse on the BIOBUZZ robot,
and "unused" is judged against last season's DECODE code, not only this repo. Before calling other code unused, check
`decode/` and last season's code for callers.

| piece | what it is |
|---|---|
| `architecture/hardware/AbsoluteAnalogEncoder.java` | an absolute encoder on an analog port: angle, multi-turn position, an optional filter that drops jumps over a set size |
| `Module.logDS` (`architecture/core/Module.java:144-150`) | a Driver Station-only line, formatted only in a DS frame |
| `Drivetrain.isBonk()` (`biobuzz/modules/Drivetrain.java:159-168`) | when called every loop: true once Pedro has been on a path, short of its end, at a speed under 2 (Pedro's velocity units) for 2 s; last season's autos, not in this repo, used it to skip a segment the robot was stuck on |
| `Camera.getPredictedBallPosition` and `getPredictedFieldPosition` (`biobuzz/modules/Camera.java:119-125`, `:162-168`), with the `predict()` they call on `TrackedBall` and `FieldBall` | where a tracked ball will be after a lookahead, extrapolated at constant velocity |
| `RobotShape.turnRadius()`, `centreFor`, `intakeAt` and `canTurnAt` (`architecture/planner/RobotShape.java:42-56`, `:132-139`) | footprint geometry for the ball planner |

`decode/` (last season's code) also stays while the Cuttle bot is still driven, decided the same day.

## Still unknown

- **Whether a fault shows on the Driver Station as it does on the dashboard:** first, big and red, readable with a
  150-character message, still first at a 1000 ms Driver Station interval, and absent with `dsTelemetry` off; and
  whether the first Driver Station frame after INIT shows its markup rendered. No bench run has had a Driver Station.
  Test: Bench: Fault Render, `python3 scripts/bench/run_bench.py --fault-render --interactive --ports-empty` with a
  Driver Hub paired ([`scripts/bench/README.md`](../scripts/bench/README.md#bench-fault-render)).
- **Field order from `getDeclaredFields()` on the Control Hub.** It decides today's order within a discovery pass, and
  whether that order survives a rename. Test: INIT any BioBuzz OpMode and read the MODULES group on the dashboard.
  Declaration order lists Drivetrain, CellTipCamera, Camera (in SOTM Test then Turret, Shooter); field-name order
  lists Camera, CellTipCamera, Drivetrain (then Shooter, Turret). Then rename one field so it sorts elsewhere, install
  with the Run button and look again.
