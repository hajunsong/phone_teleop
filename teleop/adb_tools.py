"""adb helpers: find adb, list phones, set up / tear down adb reverse.

USB mode works by  adb reverse tcp:PORT tcp:PORT : the phone app connects to
127.0.0.1:PORT on the phone and adb carries it over the cable to PORT on the PC.
Each phone has its own reverse table, so every phone can use the same port.
"""

from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path


def _find_adb() -> str:
    a = shutil.which("adb")
    if a:
        return a
    for k in ("ANDROID_HOME", "ANDROID_SDK_ROOT"):
        if os.environ.get(k):
            p = Path(os.environ[k]) / "platform-tools" / "adb.exe"
            if p.exists():
                return str(p)
    p = Path(os.environ.get("LOCALAPPDATA", "")) / "Android" / "Sdk" / "platform-tools" / "adb.exe"
    return str(p) if p.exists() else "adb"


ADB = _find_adb()


def _adb(*args, timeout=10) -> subprocess.CompletedProcess:
    return subprocess.run([ADB, *args], capture_output=True, text=True, timeout=timeout)


def devices() -> list[tuple[str, str]]:
    """[(serial, state)] — state is 'device', 'unauthorized', 'offline', ..."""
    try:
        r = _adb("devices")
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return []
    out = []
    for line in r.stdout.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 2:
            out.append((parts[0], parts[1]))
    return out


def ready_devices() -> list[str]:
    return [s for s, st in devices() if st == "device"]


def reverse(serial: str, port: int) -> bool:
    r = _adb("-s", serial, "reverse", f"tcp:{port}", f"tcp:{port}")
    return r.returncode == 0


def unreverse(serial: str, port: int) -> None:
    try:
        _adb("-s", serial, "reverse", "--remove", f"tcp:{port}")
    except Exception:                                  # noqa: BLE001
        pass
