#!/bin/bash
set -euo pipefail

HOST=192.168.43.1
SERIAL="$HOST:5555"
PACKAGE=com.qualcomm.ftcrobotcontroller
LAUNCHER="$PACKAGE/org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper"
SLOTH_DIR=/storage/emulated/0/FIRST/dairy/sloth
SETUP_DONE="======= INIT FINISH ======="
APK_DIR=TeamCode/build/outputs/apk/debug
STAMP="$APK_DIR/.hubtest-build"
BUILD_CONFIG=TeamCode/build/generated/source/buildConfig/debug/com/qualcomm/ftcrobotcontroller/BuildConfig.java
START_WAIT_S=180

cd "$(dirname "$0")/../.."

RESULTS=""
REINSTALL_HINT=""

say() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

die() {
  printf '\nSTOPPED: %s\n' "$*" >&2
  if [ -n "$REINSTALL_HINT" ]; then
    printf '\nThe bench build is installed on the hub. To put the old app and its settings back: %s\n' "$REINSTALL_HINT" >&2
  fi
  if [ -n "$RESULTS" ]; then
    printf '\nEverything it printed is in %s/console.log.\nReconnect to the internet and tell Claude the run stopped.\n' "$PWD/$RESULTS" >&2
  fi
  exit 1
}

usage() {
  cat <<EOF
usage: scripts/bench/bare_hub.sh --build          while online: build the debug APK, check Gradle works
                                                  offline, and stamp the APK with HEAD
       scripts/bench/bare_hub.sh [runner flags]   on the Control Hub's Wi-Fi: back up, install, launch, and run
                                                  every bench a bare hub allows (extra flags go to run_bench.py;
                                                  a bench flag such as --photon runs only that bench)
See "Bare hub, unattended" in scripts/bench/README.md.
EOF
}

with_timeout() {
  python3 -I -c 'import subprocess, sys
try:
    sys.exit(subprocess.run(sys.argv[2:], timeout=float(sys.argv[1])).returncode)
except subprocess.TimeoutExpired:
    sys.exit(124)' "$@"
}

tcp_open() {
  python3 -I -c 'import socket, sys
socket.create_connection((sys.argv[1], int(sys.argv[2])), timeout=3).close()' "$1" "$2" 2>/dev/null
}

find_adb() {
  local c
  if command -v adb >/dev/null 2>&1; then
    command -v adb
    return
  fi
  for c in "$HOME/Library/Android/sdk/platform-tools/adb" "${ANDROID_HOME:-/nonexistent}/platform-tools/adb" \
           "${ANDROID_SDK_ROOT:-/nonexistent}/platform-tools/adb"; do
    if [ -x "$c" ]; then
      echo "$c"
      return
    fi
  done
  return 1
}

find_apk() {
  local f found="" n=0
  for f in "$APK_DIR"/*.apk; do
    [ -e "$f" ] || continue
    found="$f"
    n=$((n + 1))
  done
  [ "$n" -eq 1 ] && echo "$found"
}

sha256() { shasum -a 256 "$1" | cut -d' ' -f1; }

stamp_get() { sed -n "s/^$1=//p" "$STAMP" | head -n 1; }

# The runner's own hash, so the stamp agrees with the Photon block's install check.
photon_sha() {
  python3 -I -B -c 'import importlib.util, sys
spec = importlib.util.spec_from_file_location("run_bench", sys.argv[1])
rb = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rb)
print(rb.photon_source_sha256(rb.REPO_ROOT / rb.PHOTON_SOURCE_DIR))' scripts/bench/run_bench.py
}

hub() { with_timeout "${HUB_TIMEOUT:-120}" "$ADB" -s "$SERIAL" "$@" </dev/null; }

die_if_hub_lost() {
  local state
  state=$(with_timeout 10 "$ADB" -s "$SERIAL" get-state </dev/null 2>/dev/null | tr -d '\r' || true)
  [ "$state" = "device" ] && return 0
  local changed="Nothing on the hub was changed."
  [ -z "$REINSTALL_HINT" ] || changed=""
  local how="adb now reports"
  [ -z "$2" ] || how="adb exited $2 and now reports"
  die "adb lost its connection to the Control Hub while $1 ($how the hub as '${state:-not connected}'). ${3:-$changed} To recover: check that the Mac is still on the hub's Wi-Fi (macOS can hop to another network) and that nothing else on the Mac restarted adb, wait a minute if the hub rebooted, run '$ADB connect $SERIAL' until it prints 'connected to $SERIAL', then run scripts/bench/bare_hub.sh again."
}

need_python() {
  command -v python3 >/dev/null 2>&1 || die "python3 isn't on PATH (macOS offers to install the Command Line Tools the first time you type python3)"
  python3 -I -c 'import sys; sys.exit(sys.version_info < (3, 9))' || die "run_bench.py needs python3 3.9 or newer"
}

build() {
  need_python
  [ -z "$(git status --porcelain)" ] || die "the worktree has uncommitted changes (see git status): commit or stash them, so the stamp names exactly what is built"
  say "building the debug APK"
  ./gradlew :TeamCode:assembleDebug
  say "checking that Gradle builds offline, as the run's Sloth steps will have to"
  ./gradlew --offline :TeamCode:assembleSloth || die "Gradle can't build offline on this Mac, so the run's Sloth steps would fail: run --build again while online, and tell Claude if it keeps failing"
  local apk photon built_photon
  apk=$(find_apk) || die "expected exactly one APK in $APK_DIR"
  photon=$(photon_sha) || die "run_bench.py couldn't hash the photon sources"
  built_photon=$(sed -n 's/.*PHOTON_SOURCE_SHA256 = "\([0-9a-f]*\)";.*/\1/p' "$BUILD_CONFIG")
  [ "$built_photon" = "$photon" ] || die "the build's BuildConfig.PHOTON_SOURCE_SHA256 (${built_photon:-not found in $BUILD_CONFIG}) isn't run_bench.py's hash of the photon sources ($photon), so the Photon block would fail its install check: tell Claude"
  printf 'head=%s\napk=%s\nsha256=%s\nphoton=%s\nbuilt=%s\n' "$(git rev-parse HEAD)" "$(basename "$apk")" "$(sha256 "$apk")" \
    "$photon" "$(date '+%Y-%m-%d %H:%M:%S')" > "$STAMP"
  say "stamped $apk for $(git rev-parse --short HEAD)"
  echo "Next: power the Control Hub with nothing plugged in, join its Wi-Fi, and run scripts/bench/bare_hub.sh"
}

run() {
  need_python
  ADB=$(find_adb) || die "adb isn't on PATH or in ~/Library/Android/sdk/platform-tools: install Android Studio's platform-tools"
  local apk head
  apk=$(find_apk) || die "no single prebuilt APK in $APK_DIR: run scripts/bench/bare_hub.sh --build while online first"
  [ -f "$STAMP" ] || die "$apk has no build stamp: run scripts/bench/bare_hub.sh --build while online first"
  head=$(git rev-parse HEAD)
  [ "$(stamp_get head)" = "$head" ] || die "the APK was built from $(stamp_get head), not HEAD $head: run scripts/bench/bare_hub.sh --build while online"
  if [ "$(stamp_get apk)" != "$(basename "$apk")" ] || [ "$(stamp_get sha256)" != "$(sha256 "$apk")" ]; then
    die "$apk changed since it was stamped: run scripts/bench/bare_hub.sh --build while online"
  fi
  [ -z "$(git status --porcelain --untracked-files=no)" ] || die "tracked files changed since the build (see git status), so the runner wouldn't match the APK: commit and --build again, or stash them"
  [ "$(stamp_get photon)" = "$(photon_sha)" ] || die "the photon sources here aren't the ones $apk was built from (stamp photon=$(stamp_get photon)), so the Photon block would fail its install check: remove what changed in them, or run scripts/bench/bare_hub.sh --build while online"

  say "looking for the Control Hub at $SERIAL"
  local tries=0
  until tcp_open "$HOST" 5555; do
    tries=$((tries + 1))
    [ "$tries" -lt 3 ] || die "the Mac can't reach a Control Hub at $SERIAL. Power the hub, join its Wi-Fi network on the Mac (the network name and password are on the hub's label or in REV Hardware Client), wait until macOS says it is connected, and run this again. Nothing was changed on any hub."
    sleep 2
  done
  local state="" attempt
  for attempt in 1 2 3; do
    with_timeout 20 "$ADB" connect "$SERIAL" </dev/null || true
    state=$(with_timeout 10 "$ADB" -s "$SERIAL" get-state </dev/null 2>/dev/null || true)
    [ "$state" = "device" ] && break
    sleep 2
  done
  [ "$state" = "device" ] || die "adb can't use $SERIAL (state '${state:-none}'). Run '$ADB connect $SERIAL' to see why; 'unauthorized' means accepting the debugging prompt. Nothing was changed on the hub."
  local serialasusb rc=0
  serialasusb=$(hub shell getprop persist.ftcandroid.serialasusb) || rc=$?
  if [ "$rc" -ne 0 ]; then
    die_if_hub_lost "reading its properties" "$rc"
    die "adb couldn't read the device's properties (exit $rc: $serialasusb); nothing was changed on it"
  fi
  serialasusb=$(printf '%s' "$serialasusb" | tr -d '\r')
  case "$serialasusb" in
    1|true|y|yes|on) ;;
    *) die "the device at $SERIAL isn't a REV Control Hub (persist.ftcandroid.serialasusb is '$serialasusb', which the FTC SDK reads as no). Nothing was changed on it." ;;
  esac
  local pm_out paths
  rc=0
  pm_out=$(hub shell pm path "$PACKAGE" 2>&1) || rc=$?
  paths=$(printf '%s\n' "$pm_out" | tr -d '\r' | sed -n 's/^package://p')
  if [ "$rc" -ne 0 ] || [ -z "$paths" ]; then
    die_if_hub_lost "asking it where the Robot Controller app is (pm path)" "$rc"
    case "$rc" in
      0|1) die "the Control Hub answers but has no Robot Controller app ($PACKAGE), which every Control Hub ships with: tell Claude. Nothing was changed on the hub." ;;
      *) die "adb shell pm path failed with exit $rc although adb still sees the hub: $pm_out. Run this again, and tell Claude if it repeats. Nothing was changed on the hub." ;;
    esac
  fi

  local stamp backup
  stamp=$(date +%Y-%m-%d_%H%M%S)
  RESULTS="bench-results/$stamp"
  # Outside the worktree: removing a worktree deletes its ignored bench-results/ with it.
  backup="$(git rev-parse --path-format=absolute --git-common-dir)/bench-backups/$stamp"
  mkdir -p "$RESULTS" "$backup/apk" "$backup/FIRST"
  exec > >(tee -i -a "$RESULTS/console.log") 2>&1
  caffeinate -dims -w $$ &
  say "Control Hub found; results go to $RESULTS, the backup to $backup; the Mac stays awake until this ends"

  local sloth_flag=""
  say "checking that Gradle builds offline for the Sloth steps"
  if ! with_timeout 600 ./gradlew --offline -q :TeamCode:assembleSloth </dev/null; then
    sloth_flag="--no-sloth"
    say "Gradle can't build offline here: skipping the Sloth steps (--no-sloth)"
  fi

  say "backing up the Robot Controller app and the hub's hardware configs"
  hub shell getprop > "$backup/getprop.txt" 2>&1 || true
  local p
  for p in $paths; do
    rc=0
    HUB_TIMEOUT=900 hub pull "$p" "$backup/apk/$(basename "$p")" >/dev/null || rc=$?
    if [ "$rc" -ne 0 ]; then
      die_if_hub_lost "copying $p off it" "$rc"
      die "couldn't copy $p off the hub, so there would be no way back: stopped before installing anything"
    fi
  done
  (cd "$backup/apk" && shasum -a 256 ./*.apk > SHA256SUMS)
  hub shell dumpsys package "$PACKAGE" | tr -d '\r' \
    | grep -E 'versionName|versionCode|firstInstallTime|lastUpdateTime|pkgFlags' > "$backup/package.txt" || true
  local listing
  rc=0
  listing=$(hub shell ls -1 /sdcard/FIRST/) || rc=$?
  [ "$rc" -eq 0 ] || die_if_hub_lost "listing its hardware configs" "$rc"
  printf '%s\n' "$listing" | tr -d '\r' | grep '\.xml$' > "$backup/FIRST/configs.txt" || true
  local x
  while IFS= read -r x; do
    rc=0
    hub pull "/sdcard/FIRST/$x" "$backup/FIRST/$x" >/dev/null || rc=$?
    if [ "$rc" -ne 0 ]; then
      die_if_hub_lost "copying /sdcard/FIRST/$x off it" "$rc"
      die "couldn't copy /sdcard/FIRST/$x off the hub: stopped before installing anything"
    fi
  done < "$backup/FIRST/configs.txt"
  rc=0
  hub shell run-as "$PACKAGE" cat "shared_prefs/${PACKAGE}_preferences.xml" > "$backup/rc_preferences.xml" 2>&1 || rc=$?
  [ "$rc" -eq 0 ] || die_if_hub_lost "reading the app's settings" "$rc"
  local original_entry=""
  original_entry=$(python3 -I -c 'import sys, xml.etree.ElementTree as ET
try:
    root = ET.parse(sys.argv[1]).getroot()
except Exception:
    sys.exit(0)
for e in root.iter("string"):
    if e.get("name") == "pref_hardware_config_filename" and e.text:
        print(e.text)' "$backup/rc_preferences.xml")
  if [ -n "$original_entry" ]; then
    say "the app's active hardware config before the install: $original_entry"
  elif python3 -I -c 'import sys, xml.etree.ElementTree as ET; ET.parse(sys.argv[1])' "$backup/rc_preferences.xml" 2>/dev/null; then
    say "the app has no active hardware config set"
  else
    say "!! couldn't read the app's settings (not a debuggable build): REINSTALL.sh can't restore them, and if its active config is one shipped inside that app (res/xml), the bench build may load a different config or none in its place; re-select it by hand afterwards"
  fi
  rc=0
  hub shell "test -d $SLOTH_DIR" || rc=$?
  case "$rc" in
    0)
      HUB_TIMEOUT=300 hub pull "$SLOTH_DIR" "$backup/sloth" >/dev/null || rc=$?
      if [ "$rc" -ne 0 ]; then
        die_if_hub_lost "copying $SLOTH_DIR off it" "$rc"
        die "couldn't copy $SLOTH_DIR off the hub: stopped before installing anything"
      fi ;;
    1) ;;
    *)
      die_if_hub_lost "looking for its Sloth folder" "$rc"
      die "adb couldn't check for $SLOTH_DIR on the hub (exit $rc): stopped before installing anything" ;;
  esac
  chmod 0644 "$backup/rc_preferences.xml"
  local apks f reinstall="$backup/REINSTALL.sh"
  apks=$(ls "$backup/apk"/*.apk | wc -l | tr -d ' ')
  {
    echo '#!/bin/bash'
    echo 'set -e'
    printf '"%s" connect %s || true\n' "$ADB" "$SERIAL"
    printf '"%s" -s %s wait-for-device\n' "$ADB" "$SERIAL"
    if [ "$apks" -eq 1 ]; then
      printf '"%s" -s %s install -r -t -d "%s"\n' "$ADB" "$SERIAL" "$(ls "$backup"/apk/*.apk)"
    else
      printf '"%s" -s %s install-multiple -r -t -d' "$ADB" "$SERIAL"
      for f in "$backup"/apk/*.apk; do printf ' "%s"' "$f"; done
      printf '\n'
    fi
    printf '"%s" -s %s shell "rm -rf %s/*"\n' "$ADB" "$SERIAL" "$SLOTH_DIR"
    if [ -n "$(ls -A "$backup/sloth" 2>/dev/null || true)" ]; then
      printf '"%s" -s %s push "%s/." %s/\n' "$ADB" "$SERIAL" "$backup/sloth" "$SLOTH_DIR"
      printf '"%s" -s %s shell "touch %s/* 2>/dev/null; true"\n' "$ADB" "$SERIAL" "$SLOTH_DIR"
    fi
    if python3 -I -c 'import sys, xml.etree.ElementTree as ET; ET.parse(sys.argv[1])' "$backup/rc_preferences.xml" 2>/dev/null; then
      printf '"%s" -s %s push "%s" /data/local/tmp/bench_rc_prefs.xml\n' "$ADB" "$SERIAL" "$backup/rc_preferences.xml"
      printf 'out=$("%s" -s %s shell "am force-stop %s && run-as %s cp /data/local/tmp/bench_rc_prefs.xml shared_prefs/%s_preferences.xml && echo PREFS_OK; rm -f /data/local/tmp/bench_rc_prefs.xml")\n' \
        "$ADB" "$SERIAL" "$PACKAGE" "$PACKAGE" "$PACKAGE"
      printf 'case "$out" in *PREFS_OK*) ;; *) echo "the app is back, but its settings were not restored: $out" >&2 ;; esac\n'
    else
      printf '"%s" -s %s shell "am force-stop %s"\n' "$ADB" "$SERIAL" "$PACKAGE"
    fi
    printf '"%s" -s %s shell "am start -n %s"\n' "$ADB" "$SERIAL" "$LAUNCHER"
  } > "$reinstall"
  chmod +x "$reinstall"
  say "backed up $apks APK(s) and $(wc -l < "$backup/FIRST/configs.txt" | tr -d ' ') hardware config(s); to put that app and its settings back later: bash $reinstall"

  say "installing $apk"
  local out
  rc=0
  out=$(with_timeout 600 "$ADB" -s "$SERIAL" install -r -t -g "$apk" </dev/null 2>&1) || rc=$?
  echo "$out"
  case "$out" in
    *INSTALL_FAILED_UPDATE_INCOMPATIBLE*)
      die "the Robot Controller app on the hub is signed with a different key than this build (a release build, or one built on another computer). Replacing it would mean uninstalling it, which wipes its settings, so nothing was installed and the hub is as it was. Tell Claude." ;;
    *INSTALL_FAILED_VERSION_DOWNGRADE*)
      die "the hub's Robot Controller app has a newer versionCode than this build ($backup/package.txt), so nothing was installed and the hub is as it was. Tell Claude." ;;
    *Success*) ;;
    *)
      die_if_hub_lost "installing the bench APK" "$rc" "If it dropped mid-install, the hub has either its old app or the bench build: once adb is back, bash $reinstall puts the old one back either way."
      die "adb install didn't succeed (output above); the hub keeps the app it had. Tell Claude." ;;
  esac
  REINSTALL_HINT="bash $reinstall"

  say "removing hot-loaded Sloth jars, as the Run button's removeSlothRemote does"
  hub shell "rm -rf $SLOTH_DIR/*" || true
  hub shell logcat -G 8M || true
  local t0
  rc=0
  t0=$(hub shell date "'+%m-%d %H:%M:%S'") || rc=$?
  if [ "$rc" -ne 0 ]; then
    die_if_hub_lost "reading its clock after the install" "$rc"
    die "adb couldn't read the hub's clock after the install"
  fi
  t0="$(printf '%s' "$t0" | tr -d '\r').000"
  say "starting the Robot Controller app"
  hub shell "am force-stop $PACKAGE; am start -n $LAUNCHER -a android.intent.action.MAIN -c android.intent.category.LAUNCHER" \
    || { rc=$?; die_if_hub_lost "starting the Robot Controller app" "$rc"; die "adb couldn't start the Robot Controller app"; }
  local launched deadline
  launched=$(date +%s)
  deadline=$((launched + START_WAIT_S))
  until hub shell "logcat -d -v threadtime -T '$t0' | grep -F -e '$SETUP_DONE'" 2>/dev/null | grep -q -F -e "$SETUP_DONE" \
        && tcp_open "$HOST" 8000; do
    die_if_hub_lost "waiting for the Robot Controller app to start" ""
    [ "$(date +%s)" -lt "$deadline" ] || die "the Robot Controller app didn't finish starting within $START_WAIT_S s of launching it (no '$SETUP_DONE' in logcat, or no dashboard on port 8000); besides the install, only the hot-loaded Sloth jars were removed, and REINSTALL.sh puts them back"
    sleep 5
  done
  say "the Robot Controller is up after about $(( $(date +%s) - launched )) s"

  say "running the benches: about 45 minutes, at most 90, no keypresses (--photon runs only the Photon block, which with --ports-empty waits for one Enter at its start); the runner puts the hardware config back at the end even if a step fails"
  local rc=0 entry_flag=()
  [ -z "$original_entry" ] || entry_flag=(--original-entry "$original_entry")
  # A non-empty trap resets to default in the runner, which restores on Ctrl-C; this shell lives on to print the backup.
  trap 'true' INT
  python3 scripts/bench/run_bench.py --gradle-offline --out "$RESULTS" $sloth_flag \
    ${entry_flag[@]+"${entry_flag[@]}"} ${1+"$@"} || rc=$?
  trap - INT
  echo
  echo "Backup of the hub's app and configs: $backup (to put that app and its settings back: bash $reinstall)"
  [ "$rc" -eq 0 ] || echo "The runner exited with status $rc; $RESULTS/summary.md says which checks failed or why it stopped."
  echo "Reconnect to the internet and tell Claude the run finished."
  return "$rc"
}

case "${1:-}" in
  --build) build ;;
  -h|--help) usage ;;
  *) run ${1+"$@"} ;;
esac
