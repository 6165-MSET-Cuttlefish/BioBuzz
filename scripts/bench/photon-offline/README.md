# Offline Photon tests

Desktop checks for `architecture/hardware/photon/` and the Photon part of `run_bench.py`. The Java suites run the real
SDK 12 classes against a fake hub; the runner suites feed `run_bench.py`'s checks synthetic and recorded results. They
catch regressions before a hub run. They don't replace the hub bench: only its verdict can say Photon is safe.

## Before running

- JDK 17 (`javac`, `java`) and `python3` on `PATH`.
- A Gradle build of this checkout, from the repo root: `./gradlew :TeamCode:assembleDebug`. The Java suites compile
  against its classes and pull the SDK, Sinister, androidx.annotation and Kotlin jars from the Gradle cache.
- The Android SDK from `ANDROID_HOME`, `ANDROID_SDK_ROOT` or `sdk.dir` in `local.properties`.

`classpath.py` finds all of that and says what's missing and how to fetch it. Everything built goes to `out/`
(gitignored).

## Suites

Run from the repo root.

| suite | command | last line when it passes |
|---|---|---|
| harness (about 15 s) | `sh scripts/bench/photon-offline/harness/run.sh` | `108 passed, 0 failed` |
| mutants: each undoes one fork change, and the harness must catch it (about 3 min, after the harness) | `python3 -I scripts/bench/photon-offline/harness/mutants.py` | `mutants: 16 of 16 caught` |
| runner checks | `python3 -I scripts/bench/photon-offline/runner/test_photon_checks.py` | `... passed, 0 failed` |
| dry run of the whole Photon block | `python3 -I scripts/bench/photon-offline/runner/dryrun_photon_block.py` | `FAILURES: none` |
| network-lock leak under a LinearOpMode STOP | `sh scripts/bench/photon-offline/lockleak/run.sh` | `13 passed, 0 failed` |
| per-write speed, desktop only | `sh scripts/bench/photon-offline/speed/run_perf.sh` | timing rows |

`runner/fixtures/` holds STOP results from the 2026-10-29 hub run that the linear STOP checks replay; pass another
`bench-results/<timestamp>` folder to `test_photon_checks.py` to use that run instead.

The speed suite ranks costs on a desktop, not Control Hub numbers. It adds stock SolversLib Photon 0.3.6 as a row
when that AAR is in the Gradle cache.

Run all of them after any change to `architecture/hardware/photon/` or to the Photon bench, before a full install.
Clean up with `rm -rf scripts/bench/photon-offline/out`.
