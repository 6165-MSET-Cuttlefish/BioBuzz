"""The offline Photon suites' inputs, from this checkout's Gradle build and the Gradle cache. As a script it prints them
as shell assignments for the run.sh scripts."""
import hashlib
import importlib.util
import json
import os
import pathlib
import re
import shlex
import subprocess
import sys
import zipfile

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[2]
OUT = HERE / "out"
BUILD = "./gradlew :TeamCode:assembleDebug"
TC = REPO / "TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes"
BUILD_FILES = ["build.gradle", "settings.gradle", "gradle.properties", "TeamCode/build.gradle",
               "gradle/wrapper/gradle-wrapper.properties"]
TREE_LINE = re.compile(r"--- ([\w.-]+):([\w.-]+)(?::(\{[^}]*\}|[^\s{}]+))?(?: -> (\S+))?")


def missing(what):
    raise SystemExit(f"missing {what}: build the branch first ({BUILD} from the repo root)")


def gradle_cache():
    home = os.environ.get("GRADLE_USER_HOME") or str(pathlib.Path.home() / ".gradle")
    return pathlib.Path(home) / "caches/modules-2/files-2.1"


def build_gradle():
    return (REPO / "TeamCode/build.gradle").read_text()


def declared_sdk_version(name):
    m = re.search(rf"['\"]org\.firstinspires\.ftc:{name}:([^'\"]+)['\"]", build_gradle())
    if not m:
        raise SystemExit(f"TeamCode/build.gradle declares no org.firstinspires.ftc:{name}")
    return m.group(1)


def compile_sdk():
    m = re.search(r"^\s*compileSdk(?:Version)?\s*=?\s*(\d+)\s*$", build_gradle(), re.M)
    if not m:
        raise SystemExit("TeamCode/build.gradle has no compileSdk")
    return m.group(1)


def android_sdk():
    for var in ("ANDROID_HOME", "ANDROID_SDK_ROOT"):
        if os.environ.get(var):
            return pathlib.Path(os.environ[var])
    props = REPO / "local.properties"
    if props.is_file():
        for line in props.read_text().splitlines():
            if line.startswith("sdk.dir="):
                return pathlib.Path(line[len("sdk.dir="):].replace("\\:", ":").replace("\\\\", "\\"))
    raise SystemExit("no Android SDK: set ANDROID_HOME (or ANDROID_SDK_ROOT, or sdk.dir in local.properties)")


def resolved_versions():
    """group:name -> the version Gradle resolves for TeamCode's debug runtime classpath, cached per build-file content."""
    key = hashlib.sha256()
    for f in BUILD_FILES:
        p = REPO / f
        key.update(f.encode() + b"\0" + (p.read_bytes() if p.is_file() else b"") + b"\0")
    cache = OUT / "resolved.json"
    if cache.is_file():
        saved = json.loads(cache.read_text())
        if saved.get("key") == key.hexdigest():
            return saved["versions"]
    gradlew = "gradlew.bat" if os.name == "nt" else "./gradlew"
    env = dict(os.environ)
    env.setdefault("ANDROID_HOME", str(android_sdk()))
    r = subprocess.run([gradlew, "-q", "--offline", ":TeamCode:dependencies", "--configuration", "debugRuntimeClasspath"],
                       cwd=REPO, env=env, capture_output=True, text=True)
    if r.returncode != 0 or "FAILED" in r.stdout:
        missing("Gradle can't resolve TeamCode's debugRuntimeClasspath offline")
    versions = {}
    for m in map(TREE_LINE.search, r.stdout.splitlines()):
        version = m and (m.group(4) or m.group(3))
        if version and not version.startswith("{"):
            versions[f"{m.group(1)}:{m.group(2)}"] = version
    OUT.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps({"key": key.hexdigest(), "versions": versions}, indent=1))
    return versions


def cached(group, name, version, ext):
    hits = sorted((gradle_cache() / group / name / version).glob(f"*/{name}-{version}.{ext}"))
    if len(hits) > 1:
        raise SystemExit(f"{group}:{name}:{version} has {len(hits)} different .{ext} files in the Gradle cache: {hits}")
    return hits[0] if hits else None


def cached_jar(coordinate, versions):
    if coordinate not in versions:
        raise SystemExit(f"TeamCode's debugRuntimeClasspath has no {coordinate}")
    group, name = coordinate.split(":")
    jar = cached(group, name, versions[coordinate], "jar")
    if jar is None:
        missing(f"{coordinate}:{versions[coordinate]} jar in the Gradle cache")
    return jar


def aar_classes(aar):
    dest = OUT / "jars" / f"{aar.stem}-{aar.parent.name[:12]}.jar"
    if not dest.is_file():
        dest.parent.mkdir(parents=True, exist_ok=True)
        part = dest.with_suffix(".part")
        with zipfile.ZipFile(aar) as z:
            part.write_bytes(z.read("classes.jar"))
        part.replace(dest)
    return dest


def sdk_classes(name):
    version = declared_sdk_version(name)
    aar = cached("org.firstinspires.ftc", name, version, "aar")
    if aar is None:
        missing(f"org.firstinspires.ftc:{name}:{version} AAR in the Gradle cache")
    return aar_classes(aar)


def check_build():
    if not TC.is_dir():
        missing(f"compiled TeamCode classes ({TC.relative_to(REPO)})")
    javap = subprocess.run(["javap", "-constants", "-cp", str(TC), "com.qualcomm.ftcrobotcontroller.BuildConfig"],
                           capture_output=True, text=True)
    built = re.search(r'PHOTON_SOURCE_SHA256 = "([0-9a-f]+)"', javap.stdout)
    spec = importlib.util.spec_from_file_location("run_bench", REPO / "scripts/bench/run_bench.py")
    rb = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(rb)
    sources = rb.photon_source_sha256(rb.REPO_ROOT / rb.PHOTON_SOURCE_DIR)
    if not built or built.group(1) != sources:
        raise SystemExit(f"compiled Photon ({built.group(1) if built else 'no hash'}) isn't built from this checkout's "
                         f"photon sources ({sources}): rebuild ({BUILD} from the repo root)")


def resolve():
    sys.dont_write_bytecode = True
    check_build()
    versions = resolved_versions()
    android = android_sdk() / "platforms" / f"android-{compile_sdk()}" / "android.jar"
    if not android.is_file():
        missing(f"android.jar for compileSdk {compile_sdk()} ({android})")
    stock = cached("org.solverslib", "photon", "0.3.6", "aar")
    return {"REPO": REPO, "OUT": OUT, "TC": TC,
            "HW": sdk_classes("Hardware"), "RC": sdk_classes("RobotCore"), "FC": sdk_classes("FtcCommon"),
            "SIN": cached_jar("dev.frozenmilk:Sinister", versions),
            "ANN": cached_jar("androidx.annotation:annotation", versions),
            "KOT": cached_jar("org.jetbrains.kotlin:kotlin-stdlib", versions),
            "ANDROID": android, "STOCK": aar_classes(stock) if stock else ""}


if __name__ == "__main__":
    for k, v in resolve().items():
        print(f"{k}={shlex.quote(str(v))}")
