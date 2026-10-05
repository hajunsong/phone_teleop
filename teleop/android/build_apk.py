"""Build (and optionally install) the phone app without Gradle.

    python build_apk.py                 -> build/HumanoidTeleop.apk
    python build_apk.py --install       ... and adb install -r on every connected phone
    python build_apk.py --install -s SERIAL

The app has no resources and no libraries (only assets: the robot model
exported by ../export_phone_model.py), so the plain SDK tool chain is enough:  aapt2 link -> javac -> d8 -> zipalign -> apksigner.  This avoids a
Gradle/AGP download and works offline.  Tools are found from ANDROID_HOME /
ANDROID_SDK_ROOT / %LOCALAPPDATA%\\Android\\Sdk and the JDK bundled with
Android Studio (or JAVA_HOME).
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE / "build"
APK = OUT / "HumanoidTeleop.apk"
MIN_SDK, TARGET_SDK = 26, 35


def find_sdk() -> Path:
    for k in ("ANDROID_HOME", "ANDROID_SDK_ROOT"):
        if os.environ.get(k):
            return Path(os.environ[k])
    p = Path(os.environ.get("LOCALAPPDATA", "")) / "Android" / "Sdk"
    if p.exists():
        return p
    sys.exit("Android SDK not found (set ANDROID_HOME)")


def find_jdk() -> Path:
    cands = [os.environ.get("JAVA_HOME"),
             r"C:\Program Files\Android\Android Studio\jbr"]
    for c in cands:
        if c and (Path(c) / "bin" / "javac.exe").exists():
            return Path(c)
    sys.exit("JDK not found (set JAVA_HOME or install Android Studio)")


def newest(d: Path, prefix: str = "") -> Path:
    def key(p: Path):
        return [int(x) if x.isdigit() else 0 for x in p.name.replace("android-", "").replace("-", ".").split(".")]
    items = sorted((p for p in d.iterdir() if p.is_dir() and p.name.startswith(prefix)), key=key)
    if not items:
        sys.exit(f"nothing in {d}")
    return items[-1]


def run(cmd, env=None):
    print(">", " ".join(str(c) for c in cmd))
    r = subprocess.run([str(c) for c in cmd], env=env)
    if r.returncode != 0:
        sys.exit(f"failed ({r.returncode}): {cmd[0]}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--install", action="store_true")
    ap.add_argument("-s", "--serial", help="adb serial (default: every device)")
    a = ap.parse_args()

    sdk, jdk = find_sdk(), find_jdk()
    bt = newest(sdk / "build-tools")
    platform = sdk / "platforms" / f"android-{TARGET_SDK}"
    if not platform.exists():
        platform = newest(sdk / "platforms", "android-")
    android_jar = platform / "android.jar"
    env = dict(os.environ, JAVA_HOME=str(jdk), PATH=str(jdk / "bin") + os.pathsep + os.environ["PATH"])
    print(f"SDK {sdk}\nbuild-tools {bt.name}, platform {platform.name}, JDK {jdk}")

    shutil.rmtree(OUT, ignore_errors=True)
    (OUT / "classes").mkdir(parents=True)
    (OUT / "dex").mkdir()

    res_apk = OUT / "res.apk"
    if not (HERE / "assets" / "robot_model.txt").exists():
        sys.exit("assets missing - run  python ../export_phone_model.py  first")
    run([bt / "aapt2.exe", "link", "-o", res_apk, "-I", android_jar,
         "--manifest", HERE / "AndroidManifest.xml", "-A", HERE / "assets",
         "--min-sdk-version", MIN_SDK, "--target-sdk-version", TARGET_SDK], env)

    srcs = sorted((HERE / "src").rglob("*.java"))
    run([jdk / "bin" / "javac.exe", "-encoding", "UTF-8", "-source", "11", "-target", "11",
         "-Xlint:-options", "-classpath", android_jar, "-d", OUT / "classes", *srcs], env)

    classes = sorted((OUT / "classes").rglob("*.class"))
    run([bt / "d8.bat", "--release", "--min-api", MIN_SDK, "--lib", android_jar,
         "--output", OUT / "dex", *classes], env)

    unaligned = OUT / "unaligned.apk"
    shutil.copy(res_apk, unaligned)
    with zipfile.ZipFile(unaligned, "a", zipfile.ZIP_DEFLATED) as z:
        z.write(OUT / "dex" / "classes.dex", "classes.dex")

    aligned = OUT / "aligned.apk"
    run([bt / "zipalign.exe", "-f", "-p", "4", unaligned, aligned], env)

    ks = Path.home() / ".android" / "debug.keystore"
    if not ks.exists():
        ks.parent.mkdir(exist_ok=True)
        run([jdk / "bin" / "keytool.exe", "-genkeypair", "-keystore", ks, "-storepass", "android",
             "-alias", "androiddebugkey", "-keypass", "android", "-keyalg", "RSA", "-validity", "10000",
             "-dname", "CN=Android Debug,O=Android,C=US"], env)
    run([bt / "apksigner.bat", "sign", "--ks", ks, "--ks-pass", "pass:android",
         "--key-pass", "pass:android", "--ks-key-alias", "androiddebugkey",
         "--out", APK, aligned], env)
    print(f"\nAPK: {APK}")

    if a.install:
        sys.path.insert(0, str(HERE.parent))
        import adb_tools
        serials = [a.serial] if a.serial else adb_tools.ready_devices()
        if not serials:
            sys.exit("no authorized device (check 'adb devices')")
        for s in serials:
            run([adb_tools.ADB, "-s", s, "install", "-r", APK])
            run([adb_tools.ADB, "-s", s, "shell", "am", "start", "-n",
                 "com.humanoid.teleop/.MainActivity"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
