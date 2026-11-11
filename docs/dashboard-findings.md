# Dashboard findings

What the robot's dashboard does that its UI doesn't show: what freezes the app, the switch that keeps it off across
restarts, the Limelight proxy, what the team's build fixes, and how `@Config` fields reach it. Read it before using the
dashboard's Hardware Config View or Enable/Disable switch, replacing or bumping the dashboard, or writing test OpMode
knobs.

The dashboard is slothboard: Dairy Foundation's fork of acmerobotics FTC Dashboard that works with Sloth hot reload.
TeamCode ships the team's own build of it, `com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0-6165.6`
(`TeamCode/build.gradle:91`; how it is built: [CLAUDE.md](../CLAUDE.md), Dependencies). "Stock" below means Dairy's
`0.3.2+0.6.0` or acmerobotics' `ftc-dashboard`. Source facts are from the team build's tag unless noted. Java paths
are under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

## Do this

- **Switch, save and delete hardware configs on the Driver Station** (Configure Robot), never in the dashboard's
  Hardware Config View: there each of the three freezes the app until the hub kills and relaunches it. The Hardware
  View (single devices) is fine.
- **Run Enable/Disable Dashboard only on purpose.** It sits in the DS TeleOp list, and the off setting survives
  restarts, reinstalls and power cycles until it is run again.
- **Keep the team build.** Stock slothboard can't save an enum `@Config` (so not the alliance), sends dashboard
  gamepads only to LinearOpModes, and its Limelight proxy drops query strings.
- **Never use stock FTC Dashboard where Sloth is on the classpath**, as it is wherever Pedro's AutoTune is: its edits
  never reach the running OpMode.
- **Put test OpMode knobs in groups**, as in [Test OpMode knobs](#test-opmode-knobs). By the source, a `final`
  holder, a Pedro `Pose` or a top-level static array of objects shows nothing editable, and a reassigned holder's
  edits go nowhere.
- For events, follow [`game-findings.md`](game-findings.md), which covers the competition rule on dashboards.

## The Hardware Config View freezes the app

Measured on a Control Hub with SDK 12.0.0 (hub firmware not recorded) on 2026-10-01, 2026-10-05 and 2026-10-24, each
time on the team build of that day; every team tag from 6165.4 to 6165.6 has the same handler code. 6 of 6 tries froze
the app, covering activate, save and delete.

- About 10.0 s after the command the hub logs `The RC has not reported itself as alive for more than 10 seconds.
  Relaunching the RC.` and kills the app. On 2026-10-05 it came back on the new config about 37 s after the command.
- The restart loses dashboard edits and the alliance ([CLAUDE.md](../CLAUDE.md), Tuning).
- With an OpMode in INIT or running, the command is ignored. Any other time it freezes; idle doesn't help.
- With no Driver Station, activating from the view still lands the config, through that freeze: use it only then.

Cause, from thread dumps taken during the freeze on 2026-10-24 and the source (`FtcDashboard.java`, the
`SET_HARDWARE_CONFIG`, `WRITE_HARDWARE_CONFIG` and `DELETE_HARDWARE_CONFIG` cases):

- Each handler runs on the websocket thread (`NanoHttpd Request Processor`) inside `activeOpMode.with(...)` and calls
  `attemptRestart()` while still holding that lock.
- The restart stops the event loop. SDK 12.0.0's `EventLoopManager.stopEventLoop` waits up to 10 s for it, then exits
  the app (`ThreadPool.awaitTerminationOrExitApplication`, javap).
- The `EventLoop` thread is stopping the active OpMode (`OpModeManagerImpl.callActiveOpModeStop`), whose
  `FtcDashboard.onOpModePostStop` waits for the same lock. Idle, the active OpMode is the SDK's default stop OpMode, so
  the same path runs.
- Stock has the same code: Dairy `0.3.0+0.6.0` and `0.3.2+0.6.0`, and acmerobotics `ftc-dashboard` master (a876f09).
  No published tag of the team build fixes it (newest `0.3.2+0.6.0-6165.6`, checked 2026-11-10).
- Fix for the team build: have each handler's lambda return whether a restart is needed, and call `attemptRestart()`
  after `activeOpMode.with` returns, posted to the UI thread the way the SDK's own Restart Robot menu item runs.

To debug any app freeze (measured 2026-10-24): run `adb logcat -G 8M` first, since the hub's `Lights Hal` lines
(about 50 a second) push the app's own out of the default buffer. Copy `/data/anr/traces.txt` as soon as logcat
prints `Wrote stack traces`: the relaunched app's startup ANR overwrites it about 30 s later.

## Enable/Disable Dashboard

Read from source (`FtcDashboard.java`, `DashboardCore.java`, the client's `Dashboard.tsx`):

- slothboard adds a TeleOp named `Enable/Disable Dashboard` (group `dash`) to the DS list. INIT shows whether the
  dashboard is on; START flips it. The team build hides it from the dashboard's own list and refuses to INIT it from
  there; stock shows it, so the dashboard can switch itself off.
- The setting is saved in the app's SharedPreferences (`shared_prefs/FtcDashboard.xml`, key `autoEnable`, default on)
  and read only when the dashboard starts with the app. Off stays off through Restart Robot, app restarts and power
  cycles, until the OpMode is run again from the DS; the hub bench notes it also survives the Run button's reinstall
  (`scripts/bench/run_bench.py`). The RC app's own menu also has an Enable Dashboard item.
- While off: no telemetry, camera stream, dashboard gamepads, logcat capture (so no Error line) or Limelight proxy, and
  the server ignores every message except status requests. `http://192.168.43.1:8080/dash` still loads and says
  "FTC Dashboard is Disabled".
- Back on without a DS (needs the Run button's debuggable APK for `run-as`; the hub bench prints this command,
  `dashboard_recovery` in `scripts/bench/run_bench.py`; unverified on a hub):

  ```
  adb shell 'am force-stop com.qualcomm.ftcrobotcontroller && run-as com.qualcomm.ftcrobotcontroller rm -f shared_prefs/FtcDashboard.xml; am start -n com.qualcomm.ftcrobotcontroller/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper'
  ```

## Limelight proxy

While the dashboard is on, it serves the Limelight (USB address 172.29.0.1) on the hub's own addresses at the same
ports (read from source, `LimelightProxyManager`):

| port | service |
|---|---|
| 5800 | MJPEG camera stream |
| 5801 | Limelight web UI: `http://192.168.43.1:5801` on the robot Wi-Fi, no USB cable needed |
| 5805 | websocket |
| 5807 | REST API |

- It binds those ports on every address from app start, so nothing else on the hub can listen on them: the hub
  bench's fake Limelight uses 15807 (`scripts/bench/README.md`), and Panels would clash
  ([toolchain-findings.md](toolchain-findings.md)).
- The team build forwards query strings; stock's `HttpForwarder` drops them, so `POST /pipeline-switch?index=4` (what
  `Limelight3A.pipelineSwitch(4)` sends) would reach the Limelight through the proxy without its index. Every Limelight
  POST through the team build was accepted on the hub on 2026-10-09
  ([hub-bench-findings.md](hub-bench-findings.md#confirmed-on-the-hub-dont-fix-these-back)).

## What the team build fixes over stock

Before replacing it with stock, port these (read from source; stock = Dairy `0.3.2+0.6.0`, same in acmerobotics
master a876f09):

- **Enum `@Config` fields** (`Context.allianceColor`, `architecture/core/Context.java:10`; SOTM Test's `aim`,
  `opmodes/test/SotmTest.java:36`). Stock's `ConfigVariableDeserializer` finds the enum with one-argument
  `Class.forName`, which returns the dashboard class loader's copy rather than the one Sloth loaded, so saving the edit
  fails at `Field.set`. The team build's `BasicVariable.update` matches the constant by name in the field's own enum
  class.
- **Dashboard gamepads in iterative OpModes** (every `EnhancedOpMode`). Stock writes them into `opMode.gamepad1` and
  `gamepad2`, which SDK 12.0.0 overwrites at the top of every loop (`OpMode.internalPreUserCode`, javap), so they reach
  only LinearOpModes. The team build's `OpModeGamepads` hands them to the SDK's package-private
  `OpMode.newGamepadDataAvailable(Gamepad, Gamepad)` by reflection; confirmed on the hub for both kinds on 2026-10-09.
  - Both builds apply dashboard gamepads only while both DS gamepads are unassociated.
  - After every SDK bump, check with `javap -p` on the new RobotCore's `OpMode` that
    `newGamepadDataAvailable(Gamepad, Gamepad)` still exists. If it doesn't, the team build logs
    `OpMode.newGamepadDataAvailable not found` and falls back to stock's broken path.
- **Limelight proxy query strings** (above).
- Also only in the team build, each confirmed on the hub on 2026-10-09
  ([hub-bench-findings.md](hub-bench-findings.md#confirmed-on-the-hub-dont-fix-these-back)): the Error line shows the
  last user-code crash, a message that fails to serialize (a NaN in a field overlay) doesn't stop telemetry, a stale
  Hardware View power edit doesn't drive a motor under the next OpMode, and a config baseline lists every `@Config`
  class. The hidden Enable/Disable OpMode (above) is team-only too.

## How `@Config` fields reach the dashboard

### Test OpMode knobs

Reference: `opmodes/test/IntakeTransferShooterTest.java:23-72`.

- One `@Config` on the OpMode class, named as [CLAUDE.md](../CLAUDE.md) (Module pattern) says. No `@Config` on its
  nested classes.
- Group knobs as `public static Group name = new Group();`, where `Group` is a `public static` nested class whose
  fields are public, non-static and non-final.
- Never make the holder `final` (that hides the whole group) and never reassign it: the dashboard keeps editing the
  object that was there when the class was scanned.
- Read the fields every loop, so edits apply live.
- A table is an object array that is an instance field of a group (`biobuzz/modules/Shooter.java:32`'s
  `feedforward`, `biobuzz/ShotTuning.java:37`'s `table`), not a top-level static array of objects.
- Use doubles, not Pedro's `Pose`: its `x`, `y` and `heading` are `private final` (Pedro 3.0.2,
  `com.pedropathing.math.Pose`, javap on 2026-11-10), so it shows as an empty group.

### The rules behind it

Read from source (`FtcDashboardScanner.java`, `ReflectionConfig.java`, `VariableType.java`):

- A class is scanned if it has `@Config` and no `@Disabled`, at app start and at each deploySloth. `@Disabled` on an
  OpMode hides its group too.
- At the top level only `public static` non-final fields show (`Class.getFields()`, so inherited public statics count).
- Leaf types: `boolean`, `int`, `long`, `float`, `double` and their boxed types, `String`, and enums. Any other object
  is a group of its public non-static fields, bound to the instance present at the scan. A null field is an empty
  group: nothing is ever instantiated.
- Arrays get one entry per index, with the length fixed at the scan. Leaf elements are re-read through the field on
  every access.
- Quirks, not tested on a hub:
  - Inside a group the filter checks the holder field's `final`, not each field's. A `final` value field in a group
    shows, though `FieldProvider` sets it with a plain `Field.set`; a `final` object field in a group is empty.
  - Array elements are filtered by the array field itself, so a top-level `public static` array of objects shows every
    element as an empty group. An object array that is an instance field of a group works.
  - `ShotTuning.keepOuts` (`biobuzz/ShotTuning.java:193`) is such a top-level array, so by this reading its keep-out
    angles can't be entered on the dashboard, which [`sotm-tuning.md`](sotm-tuning.md) (Step 8) expects. Until the
    Config View shows otherwise, set them in the source or move the array into a group.

## Stock FTC Dashboard with Sloth

With Sloth on the classpath, stock acmerobotics FTC Dashboard (`com.acmerobotics.dashboard:dashboard:0.6.0`) shows
every `@Config` group, but an edit changes nothing in the running OpMode, for every class. This hits any project with
Sloth, including one that only adds Pedro's AutoTune: the POMs of `com.pedropathing:tuning` 1.0.1 and 1.0.3 depend on
`dev.frozenmilk.sinister:Sloth:0.3.2`.

- Why (read from source): Sloth 0.3.2's `SlothTeamCodeLoader` builds a `SlothClassLoader` over the app's own APK at
  boot even when nothing was hot-loaded, so OpModes run from a second copy of every TeamCode class. Stock Dashboard
  scans the APK once at app start with the app's class loader and edits that copy's statics, which no OpMode reads.
  slothboard registers `@Config` classes from a Sinister scanner (`FtcDashboardScanner`) that also runs on Sloth's
  loads and unloads, so groups from a deploySloth jar appear and stale ones go away.
- Fix for such a project: `com.acmerobotics.slothboard:dashboard:0.3.2+0.6.0` from
  `https://repo.dairy.foundation/releases` (Dairy's newest, and Sloth 0.3.2 still its newest, checked 2026-11-10).
  Imports stay the same; the first download needs Gradle's offline mode off. Another team reported it working on
  2026-10-30 after a full install (their report, not checked by us).
- `TeamCode/build.gradle:69-71` excludes group `com.acmerobotics.dashboard` from every configuration. Keep it: it
  stops any dependency pulling stock Dashboard in beside slothboard, which uses the same package names.

## Still unknown

- **Whether the handler fix ends the freeze.** Build it as a new tag of the team build, activate a config from the
  Hardware Config View on a hub: the app should come back on the new config with no `has not reported itself as
  alive` line in logcat.
- **Whether the adb recovery brings the dashboard back.** Turn it off from the DS, run the command above, then check
  that the page shows telemetry and the Limelight web UI answers on port 5801.
- **The `@Config` quirks.** Open Shot Tuning in the Config View and check whether `keepOuts` shows its four angles per
  entry. Then deploySloth a test class holding a `final` value field in a group and a `final` object field in a group,
  and edit each.
