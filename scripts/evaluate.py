#!/usr/bin/env python3
"""Evaluates one delivery CSV (docs/test_requirements.md section 4) and writes report.txt, metrics.json and figures.

    # A Marvin replay run: the folder holds delivery.csv plus the raw records used by the health stage.
    python3 scripts/evaluate.py runs/case02_fold_clothes_spd50/real_20261027_101500/run_1
    # A CSV delivered for another robot, without a URDF:
    python3 scripts/evaluate.py their_case02_run1.csv --robot generic --out reports/their_case02_run1

Only the trajectory window is evaluated: t_s from 0 to the end of the reference (--reference eef_command.csv, or the
run_info.json of a Marvin run), unless --t-start / --t-end say otherwise. Adapted from the evaluate_tracking.py of
mindon_robotics scripts/bag_analysis, without the gate rules: the benchmark reports the numbers, it does not judge.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from trackeval import TrackEvalError  # noqa: E402
from trackeval.analyzers import DEFAULT_ANALYZERS, Context  # noqa: E402
from trackeval.analyzers.base import Options  # noqa: E402
from trackeval.csvio import load_delivery, load_reference  # noqa: E402
from trackeval.model import ArmModel  # noqa: E402
from trackeval.report import Report  # noqa: E402
from trackeval.robots import PROFILES, generic  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", type=Path, help="delivery CSV, or a Marvin run folder holding delivery.csv")
    parser.add_argument("--robot", choices=(*PROFILES, "generic"), default=None,
                        help="profile, the robot of run_info.json for a run folder and generic for a CSV by default")
    parser.add_argument("--reference", type=Path, help="eef_command.csv of the case, sets the end of the window")
    parser.add_argument("--t-start", type=float, default=0.0)
    parser.add_argument("--t-end", type=float, default=None)
    parser.add_argument("--out", type=Path, help="output folder, by default <run>/report or <csv stem>_report")
    parser.add_argument("--max-lag-ms", type=float, default=200.0)
    parser.add_argument("--no-fig", action="store_true")
    parser.add_argument("--dpi", type=int, default=150)
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args()

    raw_dir = args.path if args.path.is_dir() else None
    csv_path = raw_dir / "delivery.csv" if raw_dir else args.path
    out = args.out or (raw_dir / "report" if raw_dir else csv_path.with_name(csv_path.stem + "_report"))
    out.mkdir(parents=True, exist_ok=True)

    info = json.loads((raw_dir / "run_info.json").read_text()) if raw_dir and (raw_dir / "run_info.json").is_file() \
        else None
    t_end = args.t_end
    if t_end is None and args.reference:
        t_end = float(load_reference(args.reference).t[-1])
    if t_end is None and info:
        t_end = info["s4_start_s"] - info["s3_start_s"]

    try:
        data = load_delivery(csv_path, args.t_start, t_end)
    except TrackEvalError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1
    robot = args.robot or ((info or {}).get("robot", "marvin") if raw_dir else "generic")
    if robot != "generic":
        profile = PROFILES[robot]
        expected = {g.name: len(g.joints) for g in profile.groups}
        found = {g: n for g, n in data.n_joints.items() if n}
        if found != expected:
            print(f"error: the {robot} profile expects the joints {expected}, the CSV has {found}", file=sys.stderr)
            return 1
        model = ArmModel(profile)
    else:
        profile, model = generic(data.n_joints["left"], data.n_joints["right"]), None

    rep = Report(echo=True)
    opts = Options(args.max_lag_ms * 1e-3, figures=not args.no_fig, dpi=args.dpi, verbose=args.verbose)
    ctx = Context(profile=profile, data=data, out=out, rep=rep, opts=opts, model=model, raw_dir=raw_dir)
    header = [
        f"Delivery {csv_path}",
        f"Robot profile {profile.name}, window t_s [{data.t[0]:.3f}, {data.t[-1]:.3f}] s, {len(data.t)} rows, "
        f"{data.fs_hz:.0f} Hz",
    ]
    if data.torque_kind == "current":
        header.append("Joint effort given as current [A], the torque based numbers are in A")

    ran = []
    for cls in DEFAULT_ANALYZERS:
        analyzer = cls()
        why = analyzer.applicable(ctx)
        if why:
            rep.skipped[analyzer.name] = why
            continue
        start = time.time()
        ctx.results[analyzer.name] = analyzer.run(ctx)
        ran.append(f"{analyzer.name} ({time.time() - start:.1f} s)")
    if opts.figures:
        for result in ctx.results.values():
            for figure in result.figures:
                figure(ctx)
    header.append("Stages run: " + ", ".join(ran))
    header += [f"Skipped {name}: {why}" for name, why in rep.skipped.items()]
    rep.prepend(header)
    rep.write(out, {"delivery": str(csv_path), "robot": profile.name, "t_start": float(data.t[0]),
                    "t_end": float(data.t[-1])})
    print(f"\nreport in {out}")
    return 0 if ran else 3


if __name__ == "__main__":
    sys.exit(main())
