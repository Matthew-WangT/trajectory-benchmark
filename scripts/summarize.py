#!/usr/bin/env python3
"""Summarizes the evaluated runs of a case: mean and range over the runs of the headline metrics.

    python3 scripts/summarize.py runs/case02_fold_clothes_spd50/real_20261027_101500

Reads <session>/run_*/report/metrics.json, writes <session>/summary.txt and summary.json.
"""

from __future__ import annotations

import argparse
import json
import sys
from fnmatch import fnmatch
from pathlib import Path

import numpy as np

# (title, metric glob, unit); a glob summarizes every matching key.
HEADLINE = [
    ("关节跟踪误差 RMS 均值", "joint_error.*.rms_deg_mean", "deg"),
    ("关节跟踪误差 RMS 最大关节", "joint_error.*.rms_deg_max", "deg"),
    ("关节跟踪误差 max", "joint_error.*.max_deg", "deg"),
    ("关节时延 p50", "joint.*.delay_p50_ms", "ms"),
    ("关节时延 p95", "joint.*.delay_p95_ms", "ms"),
    ("关节等效时延 median", "joint.*.equiv_delay_median_ms", "ms"),
    ("TCP 位置误差 RMS", "tcp.*.pos_rms_mm.norm", "mm"),
    ("TCP 位置误差 p95", "tcp.*.pos_p95_mm.norm", "mm"),
    ("TCP 位置误差 max", "tcp.*.pos_max_mm.norm", "mm"),
    ("TCP 姿态误差 RMS", "tcp.*.rot_rms_deg", "deg"),
    ("TCP 姿态误差 max", "tcp.*.rot_max_deg", "deg"),
    ("TCP 等效时延", "tcp.*.equiv_delay_ms", "ms"),
    ("IK hold 占比", "health.hold_frac", ""),
    ("policy age p95", "health.policy_age_p95_ms", "ms"),
    ("反馈最大间隔", "health.feedback_gap_max_ms", "ms"),
]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path)
    args = parser.parse_args()

    runs = sorted(p.parent.parent for p in args.session.glob("run_*/report/metrics.json"))
    if not runs:
        print(f"no run_*/report/metrics.json under {args.session}, run scripts/evaluate.py first", file=sys.stderr)
        return 1
    metrics = [json.loads((r / "report/metrics.json").read_text())["metrics"] for r in runs]

    lines = [f"{args.session}: {len(runs)} runs ({', '.join(r.name for r in runs)}), mean [min, max]", ""]
    summary = {}
    for title, pattern, unit in HEADLINE:
        keys = sorted({k for m in metrics for k in m if fnmatch(k, pattern)})
        for key in keys:
            values = np.array([m[key] for m in metrics if m.get(key) is not None], dtype=float)
            if values.size == 0:
                continue
            summary[key] = {"mean": values.mean(), "min": values.min(), "max": values.max(), "n": int(values.size)}
            lines.append(
                f"{title:24s} {key:40s} {values.mean():9.3f} [{values.min():9.3f}, {values.max():9.3f}] {unit}"
            )
    text = "\n".join(lines) + "\n"
    print(text)
    (args.session / "summary.txt").write_text(text)
    (args.session / "summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False) + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
