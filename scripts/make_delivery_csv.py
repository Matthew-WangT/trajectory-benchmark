#!/usr/bin/env python3
"""Builds the delivery CSV of docs/test_requirements.md section 4 from the raw records of one replay run.

    python3 scripts/make_delivery_csv.py runs/case02_fold_clothes_spd50/real_20261027_101500/run_1

Rows are the IK evaluations (500 Hz) from S1 on, t_s = 0 at the first trajectory command (first S3 row). The command
pose is the TCP target fed to the IK controller, the measured pose FK(q_meas) of the URDF, both in the robot BASE
frame (T_yours_ours = I on the Marvin, robots/<name>/README.md otherwise). A robot with a torso gets t_* joint
columns before the arms. The feedback is interpolated linearly onto the command ticks. A `phase`
column keeps the S1..S4 split. Writes delivery.csv into the run folder.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from trackeval.csvio import read_csv  # noqa: E402
from trackeval.model import ArmModel  # noqa: E402
from trackeval.robots import PROFILES, RobotProfile  # noqa: E402

POSE = ("x", "y", "z", "qx", "qy", "qz", "qw")


def joint_columns(header: list[str], values: np.ndarray, stem: str, group) -> np.ndarray:
    index = {h: i for i, h in enumerate(header)}
    return values[:, [index[f"{group.prefix}_{stem}_{j}"] for j in range(1, len(group.joints) + 1)]]


def continuous_quat(quat: np.ndarray) -> np.ndarray:
    """Flips the quaternions (N,4) onto the hemisphere of their predecessor, the rotation is unchanged."""
    out = quat.copy()
    for i in range(1, len(out)):
        if np.dot(out[i], out[i - 1]) < 0:
            out[i] = -out[i]
    return out


def fk_poses(profile: RobotProfile, urdf: Path, q: np.ndarray) -> dict[str, np.ndarray]:
    """TCP poses (N,7) per arm prefix for the joint vectors q (N, joint_order), the torso included."""
    poses = ArmModel(profile, urdf).frame_poses(q, [a.ee_frame for a in profile.arms])
    out = {}
    for arm in profile.arms:
        pose = poses[arm.ee_frame]
        pose[:, 3:] = continuous_quat(pose[:, 3:])
        out[arm.prefix] = pose
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", type=Path)
    parser.add_argument("--urdf", type=Path, help="URDF of the FK, by default the one of the robot of run_info.json")
    args = parser.parse_args()

    run = args.run_dir
    info = json.loads((run / "run_info.json").read_text())
    profile = PROFILES[info.get("robot", "marvin")]
    if info.get("stop_after_s0"):
        print(f"{run} only ran S0, nothing to deliver", file=sys.stderr)
        return 1
    header_c, cmd, text_c = read_csv(run / "raw_cmd.csv")
    col = {h: i for i, h in enumerate(header_c)}
    phase = np.array(text_c["phase"])
    keep = phase != "S0"
    cmd, phase = cmd[keep], phase[keep]
    t_run = cmd[:, col["t_s"]]

    header_f, fb, _ = read_csv(run / "raw_fb.csv")
    t_fb = fb[:, 0]
    if t_run[0] < t_fb[0] or t_run[-1] > t_fb[-1] + 0.01:
        print("warning: the commands extend beyond the feedback records, the edges hold the end values", file=sys.stderr)

    def interp(values: np.ndarray) -> np.ndarray:
        return np.column_stack([np.interp(t_run, t_fb, values[:, j]) for j in range(values.shape[1])])

    q_meas = {g.prefix: interp(joint_columns(header_f, fb, "q", g)) for g in profile.groups}
    meas_pose = fk_poses(profile, args.urdf or profile.urdf,
                         np.column_stack([q_meas[g.prefix] for g in profile.groups]))

    columns = ["t_s", "phase"]
    blocks = []
    # The torso (if any) first: joint columns only, it has no TCP.
    for g in profile.groups:
        p, n = g.prefix, range(1, len(g.joints) + 1)
        named = []
        if g in profile.arms:
            target = cmd[:, [col[f"{p}_target_{f}"] for f in POSE]].copy()
            target[:, 3:] = continuous_quat(target[:, 3:])
            named += [([f"{p}_cmd_{f}" for f in POSE], target), ([f"{p}_meas_{f}" for f in POSE], meas_pose[p])]
        named += [
            ([f"{p}_q_cmd_{j}" for j in n], joint_columns(header_c, cmd, "q_cmd", g)),
            ([f"{p}_q_meas_{j}" for j in n], q_meas[p]),
            ([f"{p}_dq_meas_{j}" for j in n], interp(joint_columns(header_f, fb, "dq", g))),
            ([f"{p}_tau_meas_{j}" for j in n], interp(joint_columns(header_f, fb, "tau", g))),
        ]
        for names, values in named:
            columns += names
            blocks.append(values)
    # t_s = 0 at the first command of the trajectory, the first IK evaluation of S3.
    t_s = t_run - t_run[np.flatnonzero(phase == "S3")[0]]
    values = np.column_stack([t_s, *blocks])

    out = run / "delivery.csv"
    with open(out, "w") as f:
        f.write(",".join(columns) + "\n")
        for k in range(len(values)):
            f.write(f"{values[k, 0]:.6f},{phase[k]}," + ",".join(f"{v:.9g}" for v in values[k, 1:]) + "\n")
    s3 = phase == "S3"
    print(f"{out}: {len(values)} rows, S3 {s3.sum()} rows at {1 / np.median(np.diff(t_s[s3])):.0f} Hz")
    return 0


if __name__ == "__main__":
    sys.exit(main())
