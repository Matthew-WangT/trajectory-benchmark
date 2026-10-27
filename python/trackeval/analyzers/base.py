"""Analyzer interface and the shared evaluation context.

Adapted from mindon_robotics python/trackeval: the context reads a delivery CSV (csvio.Delivery) instead of a
rosbag, and the joint chain no longer needs a kinematic model, so the joint stages also evaluate the CSV of a robot
without a URDF here.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import numpy as np

from ..csvio import Delivery
from ..model import ArmModel
from ..report import Report
from ..robots import RobotProfile


@dataclass
class JointArrays:
    """Joint-space command / state aligned on the delivery time axis."""

    t: np.ndarray
    q_cmd: np.ndarray
    qd_cmd: np.ndarray
    q_meas: np.ndarray
    qd_meas: np.ndarray
    tau_meas: np.ndarray

    @property
    def fs(self) -> float:
        return 1.0 / float(np.median(np.diff(self.t)))

    @property
    def dq(self) -> np.ndarray:
        return self.q_cmd - self.q_meas


@dataclass
class Options:
    max_lag_s: float = 0.2
    coh_min: float = 0.9
    band_hz: tuple[float, float] = (0.2, 8.0)
    figures: bool = True
    dpi: int = 200
    verbose: bool = False


@dataclass
class Context:
    profile: RobotProfile
    data: Delivery
    out: Path
    rep: Report
    opts: Options
    model: ArmModel | None = None
    raw_dir: Path | None = None  # raw records of the Marvin replay, None for a CSV delivered by anyone else
    results: dict[str, Any] = field(default_factory=dict)  # analyzer name -> arbitrary result object
    _joints: JointArrays | None = None
    _plt: Any = None

    @property
    def joint_names(self) -> tuple[str, ...]:
        return self.profile.joint_order

    def plt(self):
        if self._plt is None:
            import matplotlib

            matplotlib.use("Agg")
            import matplotlib.pyplot as plt

            plt.rcParams.update({"savefig.dpi": self.opts.dpi, "axes.grid": True, "grid.alpha": 0.3, "font.size": 9})
            self._plt = plt
        return self._plt

    def joints(self) -> JointArrays | None:
        """Joint arrays of the evaluated window, both arms side by side in the profile joint order."""
        if self._joints is not None:
            return self._joints
        d = self.data
        if d.q_cmd is None or d.q_meas is None:
            return None
        qd_meas = d.dq_meas if d.dq_meas is not None else np.gradient(d.q_meas, d.t, axis=0)
        tau = d.tau_meas if d.tau_meas is not None else np.zeros_like(d.q_meas)
        self._joints = JointArrays(d.t, d.q_cmd, np.gradient(d.q_cmd, d.t, axis=0), d.q_meas, qd_meas, tau)
        return self._joints

    def ee_ros(self, kind: str) -> dict[str, np.ndarray]:
        """Reported end-effector positions per arm on the joint time axis. kind: 'state' (the *_meas_* columns).

        There is no 'command' counterpart: the *_cmd_* columns hold the TCP target fed to the IK, not FK(q_cmd).
        """
        if kind != "state":
            return {}
        return {arm.name: self.data.meas_pose[arm.name][:, :3] for arm in self.profile.arms}


@dataclass
class AnalyzerResult:
    data: Any = None
    figures: list = field(default_factory=list)  # callables (ctx) -> None that save figures


class Analyzer:
    name = "base"
    stage = ""
    title = ""

    def applicable(self, ctx: Context) -> str | None:
        """Return None when the analyzer can run, otherwise the reason it is skipped."""
        return None

    def run(self, ctx: Context) -> AnalyzerResult:
        raise NotImplementedError


def needs_joint_chain(ctx: Context) -> str | None:
    if ctx.joints() is None:
        return "the CSV has no joint command / measurement columns"
    return None


def needs_model(ctx: Context) -> str | None:
    why = needs_joint_chain(ctx)
    if why:
        return why
    if ctx.model is None:
        return "no kinematic model (URDF) for this robot"
    return None
