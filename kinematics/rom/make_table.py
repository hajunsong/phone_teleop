"""Module drive-range table from the RecurDyn interference sweep.

    python make_table.py      -> rom_table.md (also printed), rom_limits.json

Rows come from rom_limits_pose.csv (this folder, 2026-10-06 model) and are set
next to the 2026-09-03 measurement (motion_generation/rom/rom_limits_pose.csv)
and the limits applied on the robot (teleop/joint_limits_robot.json, which were
taken from that September run).

Angles are in the controller convention (= params/arm_*.json, used by the
sweep for the axis lines).  In the current rmd q7 is reversed, so its range in
rmd terms is the mirror image; both are listed.
"""

from __future__ import annotations

import csv
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
CLAUDE_WS = HERE.parents[1]
OLD = CLAUDE_WS / "motion_generation" / "rom" / "rom_limits_pose.csv"
ROBOT = CLAUDE_WS / "teleop" / "joint_limits_robot.json"
MODULE = {"RA": [2, 3, 4, 5, 6, 7, 8], "LA": [9, 10, 11, 12, 13, 14, 15]}
NAME = ["어깨 Pitch", "어깨 Roll", "어깨 Yaw", "팔꿈치 Pitch", "손목 Roll", "손목 Yaw", "손목 Pitch"]


def read(path):
    if not path.exists():
        return {}
    with open(path, encoding="utf-8-sig", newline="") as f:
        return {r["axis"]: r for r in csv.DictReader(f)}


def val(s):
    return None if s in (None, "") else float(s)


def fmt(v, clear="간섭 없음"):
    return clear if v is None else f"{v:+.2f}"


def short(who):
    """'RightArm/body3.부품 vs base.부품' -> 'body3.부품 ↔ base.부품'."""
    if not who:
        return ""
    parts = [p.strip().split("@")[0] for p in who.split(" vs ")]
    return " ↔ ".join(p.split("/")[-1] for p in parts)


def main() -> int:
    new, old = read(HERE / "rom_limits_pose.csv"), read(OLD)
    robot = json.loads(ROBOT.read_text(encoding="utf-8"))
    out, md = {}, []
    md.append("| 팔 | q | 관절 | 모듈 | 하한 [°] | 상한 [°] | 하한을 막는 부품 | 상한을 막는 부품 | 9/3 측정 | 실기 적용 |")
    md.append("|---|---|---|---|---|---|---|---|---|---|")
    for side in ("RA", "LA"):
        for j in range(1, 8):
            ax = f"{side}_q{j}"
            r = new.get(ax)
            o = old.get(ax)
            lo = val(r["q_min_deg"]) if r else None
            hi = val(r["q_max_deg"]) if r else None
            out[ax] = {"module": f"M{MODULE[side][j - 1]}", "joint": NAME[j - 1], "measured": r is not None,
                       "q_min_deg": lo, "q_max_deg": hi,
                       "blocks_min": r["blocks_min"] if r else "", "blocks_max": r["blocks_max"] if r else ""}
            old_txt = "-" if not o else f"{fmt(val(o['q_min_deg']), '없음')} ~ {fmt(val(o['q_max_deg']), '없음')}"
            rb = f"{robot[side]['min'][j - 1]:+.2f} ~ {robot[side]['max'][j - 1]:+.2f}"
            if r is None:
                md.append(f"| {side} | q{j} | {NAME[j - 1]} | M{MODULE[side][j - 1]} | (미측정) | | | | {old_txt} | {rb} |")
                continue
            md.append(f"| {side} | q{j} | {NAME[j - 1]} | M{MODULE[side][j - 1]} | {fmt(lo)} | {fmt(hi)} | "
                      f"{short(r['blocks_min'])} | {short(r['blocks_max'])} | {old_txt} | {rb} |")
    q7 = []
    for side in ("RA", "LA"):
        e = out[f"{side}_q7"]
        if e["measured"]:
            lo, hi = e["q_min_deg"], e["q_max_deg"]
            q7.append(f"- {side} q7 (현재 rmd 규약, 부호 반대): "
                      f"{fmt(None if hi is None else -hi)} ~ {fmt(None if lo is None else -lo)}")
    text = "\n".join(md) + ("\n\n" + "\n".join(q7) if q7 else "") + "\n"
    (HERE / "rom_table.md").write_text(text, encoding="utf-8")
    (HERE / "rom_limits.json").write_text(json.dumps(out, indent=1, ensure_ascii=False), encoding="utf-8")
    print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
