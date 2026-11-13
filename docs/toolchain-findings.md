# Toolchain findings

What holds the build toolchain back, where deploySloth and adb go wrong, and what to do after an install. Read it
before changing AGP, Gradle, a library or `TeamCode/build.gradle`, and when a deploy, a hardware config or adb
misbehaves.

## Changing the build

- **Stay on Android Gradle Plugin (AGP) 8.13.2: decline Android Studio's AGP upgrade prompt.** AGP 9 builds the APK
  but breaks `:TeamCode:assembleSloth`, so deploySloth (hot reload) stops working. Measured on a Mac with the Gradle
  daemon on JDK 17: AGP 9.4.1 with Gradle 9.6.0 (2026-09-30) and AGP 9.5.0 with Gradle 9.6.0 (2026-10-30) both pass
  `assembleDebug` and fail `assembleSloth` with D8's `Attempt at compiling intermediate artifact without its context`
  on the `$$ExternalSyntheticBackport` classes.
  - Cause: `assembleSloth` merges the per-class dex files from AGP's `dexBuilderDebug` into one indexed dex with D8 in
    RELEASE mode, without `setIntermediate` (read from [Sloth](https://github.com/Dairy-Foundation/Sloth) 0.3.2,
    `Load/src/main/kotlin/dev/frozenmilk/sinister/sloth/AssembleSloth.kt:31-37` and `DexSloth.kt`). The D8 that runs
    is the project's AGP's. D8 9.x refuses that merge when it holds the backport helpers D8 writes for Java APIs newer
    than `minSdkVersion 24` (`List.of`, `String.join`, used in `pedro/procedures/` and `WebcamControls`); D8 8.13.19,
    from AGP 8.13.2, merges the same jar (both run standalone, 2026-10-30).
  - Adding `setIntermediate(true)` to that D8 call builds on AGP 9.5.0, but the result has never been loaded on a hub:
    not a fix yet.
  - What unblocks it: a Sloth release after 0.3.2 that handles AGP 9. On 2026-11-10, 0.3.2 is still the newest Sloth
    and Sloth Load plugin (the Gradle plugin that adds `assembleSloth` and `deploySloth`) on `repo.dairy.foundation`,
    and 8.13.2 is the last AGP 8 release (Maven metadata).
  - AGP 9.5 needs Gradle 9.6.0 or later, SDK Build Tools 36.0.0 and JDK 17
    ([release notes](https://developer.android.com/build/releases/agp-9-5-0-release-notes), fetched 2026-11-10).
    AGP 9.5.1 is out and untried.
- **Gradle stays on 9.1.0; with AGP 8, no Gradle from 9.6.0 on works.** Gradle 9.6.0 removed an internal Problems
  API that AGP 8 uses: on Gradle 9.8.2 both `assembleDebug` and `assembleSloth` fail configuring
  `TeamCode/build.gradle` line 4 in about 1 s (measured 2026-10-30; Gradle's
  [upgrade guide](https://docs.gradle.org/9.8.2/userguide/upgrading_version_9.html), "Plugins relying on removed
  Problems API internals"). The current Gradle, 9.9.1 (2026-11-10), is ruled out with it.
  - Gradle 9.4.1 passes both with the same two warnings as 9.1.0, both from AGP 8.13.2's own multi-string dependency
    notation, which "will fail with an error in Gradle 10" (measured 2026-10-30). Gradle 9.5.x hasn't been tried.
  - 9.4.1 isn't worth a move on its own: it only puts AGP 8.13.2 inside Gradle's tested range (9.1.0 is tested with
    AGP 8.4 to 8.12, 9.4.1 with 8.13 to 9.1.0-alpha04, 9.5.0 with 9.0 to 9.2.0-alpha05: each version's
    `docs.gradle.org/<version>/userguide/compatibility.html`, fetched 2026-11-10), and every IDE has to download a new
    distribution. Move Gradle together with AGP 9 and a working `assembleSloth`.
- **Keep `TeamCode/libs/OpModeAnnotationProcessor.jar` wired in** (`TeamCode/build.gradle:83`), though the SDK v12.0
  template has no annotation processor ([`build.dependencies.gradle` at v12.0][sdk-deps], fetched 2026-11-10). It
  fails the build with `Clash of OpMode names detected for the following classes` when two `@TeleOp`/`@Autonomous`
  classes share a name (string read from the jar's `MyProcessor` class). Without it a duplicate shows only on the
  hub, where Sloth 0.3.2's `SinisterRegisteredOpModes.register` throws `DuplicateNameException` while building the
  OpMode list.

[sdk-deps]: https://github.com/FIRST-Tech-Challenge/FtcRobotController/blob/v12.0/build.dependencies.gradle

### Library versions on 2026-11-10

| dependency | in the repo | newest | note |
|---|---|---|---|
| FTC SDK (`org.firstinspires.ftc:*`) | 12.0.0 | 12.0.0 | |
| `com.pedropathing:revhub` | 3.0.2 | 3.0.2 | |
| `com.pedropathing:tuning` (AutoTune) | 1.0.1 | 1.0.3 | bump it as [pedro-findings.md](pedro-findings.md#autotune) says |
| `com.pedropathing.ivy:pedro` | 1.1.1 | 1.1.1 | |
| Sloth and its Load plugin | 0.3.2 | 0.3.2 | |
| Sinister | 2.3.0 | 2.3.0 | comes with Sloth 0.3.2, whose module metadata requires 2.3.0 |
| slothboard | team build `0.3.2+0.6.0-6165.7` | Dairy's `0.3.2+0.6.0` | the team build sits on Dairy's newest |
| `androidx.appcompat:appcompat` | 1.2.0 | 1.8.0 | the SDK v12.0 template and Pedro's Quickstart both pin 1.2.0 |
| AGP | 8.13.2 | 9.5.1 | above |
| Gradle | 9.1.0 | 9.9.1 | above |

Newest versions are from Maven Central, Google Maven, `repo.dairy.foundation` and `services.gradle.org`; the SDK's
FtcRobotController tags end at v12.0.

- `scripts/` Python imports only the standard library (checked 2026-11-10).
- `vendor/prism`, `vendor/digitalchickenlabs` and `vendor/sdk` match upstream's current revision apart from the edits
  their `*_REV` files list (diffed 2026-11-10). `vendor/brushland` is unchecked (Still unknown).
- Everything on Dairy's and acmerobotics' `ftc-dashboard` master is in the team's slothboard fork. The one open
  acmerobotics PR it lacks, [#231](https://github.com/acmerobotics/ftc-dashboard/pull/231) "Allow disabling Limelight
  proxy", matters only if Panels (`com.bylazar:fullpanels`) is added: slothboard's Limelight proxy listens on the
  hub's ports 5800, 5801, 5805 and 5807 (`LimelightProxyManager` in our slothboard build), and Panels binds the same
  ports (`com.bylazar.limelightproxy.Plugin` in `limelightproxy` 1.0.6, part of `fullpanels` 1.0.20; both read from
  their jars 2026-11-10). AutoTune's 10158 and 12649 don't clash.

## Deploying

- **After a full install (Run button) that adds, renames or deletes a file in `TeamCode/src/main/res/xml/`,
  re-activate the hub's hardware config on the Driver Station (DS)** (Configure Robot, pick it, Activate). Otherwise
  the hub can load another config's devices under the name the DS shows. When an OpMode can't find a device that the
  config the DS shows has, suspect this first.
  - The Robot Controller stores the active shipped config as its name plus its Android resource ID, and loads the XML
    by ID alone (read from FtcCommon 12.0.0 source, `RobotConfigFile.getXmlResource` and
    `RobotConfigFileManager.getActiveConfig`; our
    `TeamCode/src/main/java/org/firstinspires/ftc/robotcontroller/internal/FtcRobotControllerActivity.java:716-728`).
    A stored ID that names no XML falls back to no config.
  - Resource IDs for xml files run in name order across our and the SDK's xml files (`advanced_rc_settings`
    0x7f110000 to `teamwebcamcalibrations` 0x7f110016 in a 2026-11-10 debug build's
    `TeamCode/build/intermediates/runtime_symbol_list/debug/processDebugResources/R.txt`), so a change shifts every
    config named after it. Adding `biobuzz.xml` on 2026-11-08 shifted `butterfly`, `camera`, `cuttle_decode`,
    `intake_transfer_shooter` and both `laser_rangefinder` configs by one: a hub left on `cuttle_decode` from before
    that install loads `camera.xml`'s devices.
  - A by-name lookup would fix it (`getResources().getIdentifier(name, "xml", packageName)` in
    `FtcRobotControllerActivity.requestRobotSetup`, then re-save the config), but `robotcontroller/` is SDK template
    code that CLAUDE.md keeps unedited. The bench already re-activates shipped configs by name
    ([bench README](../scripts/bench/README.md)).
- **deploySloth never really waits for the load, and can push over a load still in progress.** The wait rule is in
  [CLAUDE.md](../CLAUDE.md) and the timings in [hub-bench-findings.md](hub-bench-findings.md). The cause is a
  lock-file name mismatch in Sloth 0.3.2: the Load plugin checks for `/storage/emulated/0/FIRST/dairy/sloth/lock`
  before the push and polls it after (`Load/.../DeploySloth.kt:49` and `:89`), while the runtime's lock file is
  `sloth.lock` in that folder (`SlothTeamCodeLoader.kt:25`). `lock` never exists, so both checks pass at once. Only a
  change to Dairy's Load plugin fixes it.
- **With two adb devices attached, set `ANDROID_SERIAL`.** The Load plugin runs `adb` without `-s` (Sloth 0.3.2,
  `DeploySloth.kt`, `RemoveRemoteSloth.kt`). With two hubs, or one hub on both USB and Wi-Fi, deploySloth fails with
  adb's more-than-one-device error, and `removeSlothRemote` (which Gradle's `installDebug` runs first) ignores adb's
  exit code and silently deletes nothing. Set `ANDROID_SERIAL=<serial from adb devices>` in the environment Gradle
  runs in, as the [bench](../scripts/bench/README.md) does: a task's `ExecOperations` child sees the value `./gradlew`
  was started with, on a reused daemon too (an init-script task on Gradle 9.1.0, run twice with two values, on a Mac,
  2026-11-12).
- **Connect adb yourself before deploySloth, or it drops the hub afterwards.** With nothing in `adb devices`, the Load
  plugin's default autoconnect (`LoadSettings.kt`: address 192.168.43.1, `AutoConnect.MAINTAIN`) runs
  `adb connect 192.168.43.1`, deploys, then a bare `adb disconnect`, which disconnects every network device
  (`adb --help`, platform-tools 37.0.1). With a device already listed it neither connects nor disconnects
  (`AutoConnect.kt`), so run `adb connect 192.168.43.1:5555` first to keep Wi-Fi logcat and the Run button working.

## Laptop and adb

- **If the laptop's adb server dies every 10 to 80 s, look for something killing it before blaming the IDE**:
  `launchctl list | grep -i adbtunnel` and `ls ~/Library/LaunchAgents/` on a Mac.
  - Relay's laptop tunnel did this before its VERSION 10. It is a LaunchAgent `com.adbtunnel.<bridge>` that forwards
    local port 5037, the adb server's port, over ssh to an adb server on the Relay bridge (Relay's
    `laptop/install.sh`). While the bridge was out of reach (another network, or a Control Hub's own Wi-Fi), it shut
    down any local adb server before each ssh retry, every 10 to 76 s (the tunnel's own log on one Mac, 2026-10-22):
    Android Studio's, Wi-Fi ADB to a hub and the bench's.
  - A laptop with an SSH-key tunnel picks up the fix by itself overnight, even off the bridge's network; a password
    tunnel needs `laptop/install.sh` re-run from a Relay checkout. Relay's README (Updates and Troubleshooting, read
    2026-11-10) covers both, and what to do if the tunnel then stays down.
- **Before a hub bench run, `lsof -nP -iTCP:5037 -sTCP:LISTEN` must not show `ssh`.** While a tunnel holds 5037,
  adb reaches whatever hub the bridge is on, and the bench drives motor ports. To pause Relay's tunnel on a Mac:
  `launchctl bootout gui/$(id -u)/<name>`, where `<name>` is the `com.adbtunnel.*` file in `~/Library/LaunchAgents/`
  without `.plist`, not `com.adbtunnel.updater`. It comes back at the next login. `run_bench.py` and `bare_hub.sh`
  check the port themselves before their first adb command and stop, naming the process and its pid
  ([bench README](../scripts/bench/README.md)).
- **Over a USB cable, a connection through `adb forward` proves nothing about the hub's port.** adb accepts it on the
  laptop before it asks the device (`adb_listeners.cpp`, `listener_event_func`), sends a bare `tcp:<port>` to the
  device's loopback (`socket_spec.cpp`), drops a device's forwards when the device leaves (a reboot or a pulled cable),
  and prints the port it picked for `forward tcp:0` on a line of its own (`client/commandline.cpp`): adb's source on
  android.googlesource.com, read 2026-11-12. Read the hub's socket table instead, as the bench does with `--usb`. A
  page needs more than its own forward: slothboard's `/dash` script connects to port 8000 of the address it came from
  (`assets/dash/assets/index.*.js` in the 6165.7 AAR), and the RC page's to its own port + 1 (`websocket-core.js` in
  RobotServer 12.0.0).
- **A new clone or git worktree can't build from a terminal until it has a `local.properties`.** Gradle stops with
  "SDK location not found" (seen 2026-11-12 in a worktree; the file is gitignored, so only the folder Android Studio
  opened has one). Open the folder in Android Studio once, copy `local.properties` from another checkout on the same
  laptop, or set `ANDROID_HOME` to the SDK folder. `bare_hub.sh --build` hits this too.
- **After an Xcode update, a Mac's `git` and `/usr/bin/python3` refuse to run until its licence is accepted.** Both
  print "You have not agreed to the Xcode license agreements" and exit 69 (seen 2026-11-12). Run
  `sudo xcodebuild -license` in Terminal. Until then `bare_hub.sh` stops and says so; before 2026-11-12 its `--build`
  passed the uncommitted-changes check and stamped the APK with an empty commit.
- **Fake hubs stay off ports 5554 to 5585 and never touch the real adb.** The adb server finds emulators by scanning
  the odd localhost ports 5555 to 5585 ([adb docs](https://developer.android.com/tools/adb), fetched 2026-11-10). A
  fake hub on `127.0.0.1:5555` was picked up as `emulator-5554` and churned about every 1.3 s, disturbing a real hub's
  Wi-Fi ADB session on the same laptop (2026-10-22). In a harness, rewrite the hub address `192.168.43.1:5555`
  (`scripts/bench/run_bench.py:38-39`) to something like `127.0.0.1:15555`, use a fake adb (first on `PATH`, or
  in-process like `FakeAdb` in `scripts/bench/photon-offline/runner/dryrun_photon_block.py:84`), and never run
  `adb kill-server` or `adb connect`. Isolated like that, it can run while someone uses a real hub.

## Still unknown

- Whether Sloth with `setIntermediate(true)` loads on a hub under AGP 9: build Sloth with that change, move the repo
  to AGP 9.5.1 on a throwaway branch, deploySloth to a bench hub, and check for the `Processed Sloth Load` toast and
  the changed code running.
- Gradle 9.5.1 with AGP 8.13.2: on a throwaway branch, set the wrapper to 9.5.1 and build
  `:TeamCode:assembleDebug :TeamCode:assembleSloth`.
- Whether a Gradle daemon started by Android Studio sees `ANDROID_SERIAL`: with two hubs attached, set it in the
  deploySloth run configuration's environment, run it, and check which hub's logcat shows the load.
- What a duplicate OpMode name does to the running app under Sloth: on a throwaway branch, remove the annotation
  processor, give two OpModes one name, deploySloth, and watch the DS and logcat.
- `vendor/brushland/LaserRangefinder.java` against Brushland's page: the docs site reset the connection on 2026-10-30
  and 2026-11-10. From another network, diff the page's class against the file after reapplying `BRUSHLAND_REV`'s
  edits.
