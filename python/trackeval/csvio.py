"""Reads the delivery CSV of docs/test_requirements.md section 4 and the reference eef_command.csv."""

from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import TrackEvalError

POSE_FIELDS = ("x", "y", "z", "qx", "qy", "qz", "qw")
ARM_PREFIXES = {"left": "l", "right": "r"}
# Joint columns of a robot with a torso shared by both arms, before the arms: t_q_cmd_*, t_q_meas_*, ...
TORSO_PREFIX = "t"


def read_csv(path: Path) -> tuple[list[str], np.ndarray, dict[str, list[str]]]:
    """Header, numeric columns as a float array (non numeric cells NaN) and the raw text of non numeric columns."""
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if len(rows) < 2:
        raise TrackEvalError(f"{path} holds no data rows")
    header, body = rows[0], rows[1:]
    values = np.full((len(body), len(header)), np.nan)
    text: dict[str, list[str]] = {}
    for j, name in enumerate(header):
        column = [r[j] if j < len(r) else "" for r in body]
        try:
            values[:, j] = [float(v) for v in column]
        except ValueError:
            text[name] = column
    return header, values, text


def _columns(header: list[str], values: np.ndarray, names: list[str]) -> np.ndarray:
    index = {name: i for i, name in enumerate(header)}
    missing = [n for n in names if n not in index]
    if missing:
        raise TrackEvalError(f"missing columns {missing[:4]}{' ...' if len(missing) > 4 else ''}")
    return values[:, [index[n] for n in names]]


def _joint_count(header: list[str], stem: str) -> int:
    pattern = re.compile(rf"^{re.escape(stem)}_(\d+)$")
    numbers = sorted(int(m.group(1)) for h in header if (m := pattern.match(h)))
    if numbers and numbers != list(range(1, len(numbers) + 1)):
        raise TrackEvalError(f"columns {stem}_* are not numbered 1..N")
    return len(numbers)


@dataclass
class Delivery:
    path: Path
    t: np.ndarray
    cmd_pose: dict[str, np.ndarray]  # arm -> (N,7) TCP command, x y z qx qy qz qw
    meas_pose: dict[str, np.ndarray]  # arm -> (N,7) measured TCP
    q_cmd: np.ndarray | None  # (N, n_left + n_right), left arm first
    q_meas: np.ndarray | None
    dq_meas: np.ndarray | None
    tau_meas: np.ndarray | None  # torque in N·m, or current in A when torque_kind is "current"
    torque_kind: str | None
    n_joints: dict[str, int]
    fs_hz: float


def load_delivery(path: Path, t_start: float = 0.0, t_end: float | None = None) -> Delivery:
    """Rows with t_start <= t_s <= t_end, by default from the first trajectory command to the end of the reference."""
    header, values, _ = read_csv(path)
    t_all = _columns(header, values, ["t_s"])[:, 0]
    fs_hz = 1.0 / float(np.median(np.diff(t_all)))
    keep = t_all >= t_start - 1e-9
    if t_end is not None:
        keep &= t_all <= t_end + 1e-9
    if keep.sum() < 10:
        raise TrackEvalError(f"{path}: fewer than 10 rows in [{t_start}, {t_end}] s")
    values = values[keep]
    t = t_all[keep]

    cmd_pose, meas_pose, n_joints = {}, {}, {}
    for arm, p in ARM_PREFIXES.items():
        cmd_pose[arm] = _columns(header, values, [f"{p}_cmd_{f}" for f in POSE_FIELDS])
        meas_pose[arm] = _columns(header, values, [f"{p}_meas_{f}" for f in POSE_FIELDS])
        n_joints[arm] = _joint_count(header, f"{p}_q_meas")
    n_joints["torso"] = _joint_count(header, f"{TORSO_PREFIX}_q_meas")
    prefixes = {"torso": TORSO_PREFIX, **ARM_PREFIXES} if n_joints["torso"] else ARM_PREFIXES

    def joints(stem: str) -> np.ndarray | None:
        if any(_joint_count(header, f"{p}_{stem}") == 0 for p in prefixes.values()):
            return None
        cols = [f"{p}_{stem}_{i}" for group, p in prefixes.items() for i in range(1, n_joints[group] + 1)]
        return _columns(header, values, cols)

    tau, kind = joints("tau_meas"), "tau"
    if tau is None:
        tau, kind = joints("current"), "current"
    if tau is None:
        kind = None
    return Delivery(
        path=path,
        t=t,
        cmd_pose=cmd_pose,
        meas_pose=meas_pose,
        q_cmd=joints("q_cmd"),
        q_meas=joints("q_meas"),
        dq_meas=joints("dq_meas"),
        tau_meas=tau,
        torque_kind=kind,
        n_joints=n_joints,
        fs_hz=fs_hz,
    )


@dataclass
class Reference:
    t: np.ndarray
    pose: dict[str, np.ndarray]  # arm -> (N,7)


def load_reference(path: Path) -> Reference:
    """eef_command.csv of a case, in the BASE frame of the original robot."""
    header, values, _ = read_csv(path)
    t = _columns(header, values, ["t_s"])[:, 0]
    pose = {}
    for arm, p in ARM_PREFIXES.items():
        pose[arm] = _columns(header, values, [f"{p}_x_m", f"{p}_y_m", f"{p}_z_m", f"{p}_qx", f"{p}_qy", f"{p}_qz", f"{p}_qw"])
    return Reference(t, pose)
