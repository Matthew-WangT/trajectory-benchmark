"""L3c: TCP tracking, the command fed to the controller against the measured TCP, for any robot."""

from __future__ import annotations

import numpy as np

from ..util import quat_angle_deg, rms
from .base import Analyzer, AnalyzerResult, Context

AXES = "xyz"


def _pos_rms_at_lag(t: np.ndarray, cmd: np.ndarray, meas: np.ndarray, lag: float) -> float:
    """RMS of |meas(t) - cmd(t - lag)| over the samples where the shifted command exists."""
    ok = t - lag >= t[0]
    shifted = np.column_stack([np.interp(t[ok] - lag, t, cmd[:, k]) for k in range(3)])
    return float(rms(np.linalg.norm(meas[ok] - shifted, axis=1)))


class TcpTrackingAnalyzer(Analyzer):
    name = "tcp"
    stage = "L3c"
    title = "Step 3c  末端层: TCP 命令 -> TCP 实测"

    def run(self, ctx: Context) -> AnalyzerResult:
        rep, d, o = ctx.rep, ctx.data, ctx.opts
        rep.section(self.title)
        rep.add("误差 = meas - cmd. 位置 [mm]: 各轴 RMS / p95|e| / max|e|, 模长; 姿态 [deg]")
        errors = {}
        lags = np.arange(0.0, o.max_lag_s + 1e-9, 0.001)
        for arm in ctx.profile.arms:
            a = arm.name
            cmd, meas = d.cmd_pose[a], d.meas_pose[a]
            pos = (meas[:, :3] - cmd[:, :3]) * 1e3
            norm = np.linalg.norm(pos, axis=1)
            rot = quat_angle_deg(cmd[:, 3:], meas[:, 3:])
            errors[a] = (pos, rot)
            axis_rms, axis_p95, axis_max = rms(pos, axis=0), np.percentile(np.abs(pos), 95, axis=0), np.abs(pos).max(0)
            rep.add(
                f"  {a:5s} 位置 RMS {np.round(axis_rms, 2).tolist()}  p95 {np.round(axis_p95, 2).tolist()}  "
                f"max {np.round(axis_max, 2).tolist()}"
            )
            rep.add(
                f"        |e| RMS {rms(norm):.2f}  p95 {np.percentile(norm, 95):.2f}  max {norm.max():.2f} mm;  "
                f"姿态 RMS {rms(rot):.3f}  p95 {np.percentile(rot, 95):.3f}  max {rot.max():.3f} deg"
            )
            for k, ax in enumerate(AXES):
                rep.metric(f"tcp.{a}.pos_rms_mm.{ax}", axis_rms[k])
                rep.metric(f"tcp.{a}.pos_p95_mm.{ax}", axis_p95[k])
                rep.metric(f"tcp.{a}.pos_max_mm.{ax}", axis_max[k])
            rep.metric(f"tcp.{a}.pos_rms_mm.norm", rms(norm))
            rep.metric(f"tcp.{a}.pos_p95_mm.norm", np.percentile(norm, 95))
            rep.metric(f"tcp.{a}.pos_max_mm.norm", norm.max())
            rep.metric(f"tcp.{a}.rot_rms_deg", rms(rot))
            rep.metric(f"tcp.{a}.rot_p95_deg", np.percentile(rot, 95))
            rep.metric(f"tcp.{a}.rot_max_deg", rot.max())

            # Equivalent delay: the shift of the command that best explains the measurement.
            curve = np.array([_pos_rms_at_lag(d.t, cmd[:, :3], meas[:, :3], lag) for lag in lags])
            best = int(np.argmin(curve))
            rep.add(
                f"        等效时延 {lags[best] * 1e3:.0f} ms (平移命令后 |e| RMS {curve[0] * 1e3:.2f} -> "
                f"{curve[best] * 1e3:.2f} mm, 剩余部分不是时延能解释的)"
            )
            rep.metric(f"tcp.{a}.equiv_delay_ms", lags[best] * 1e3)
            rep.metric(f"tcp.{a}.pos_rms_after_delay_mm", curve[best] * 1e3)
        return AnalyzerResult(data=errors, figures=[fig_tcp])


def fig_tcp(ctx: Context) -> None:
    plt = ctx.plt()
    t, errors, arms = ctx.data.t, ctx.results["tcp"].data, ctx.profile.arms
    fig, axes = plt.subplots(len(arms), 2, figsize=(16, 3.8 * len(arms)), squeeze=False, sharex=True)
    for r, arm in enumerate(arms):
        pos, rot = errors[arm.name]
        for k, ax in enumerate(AXES):
            axes[r, 0].plot(t, pos[:, k], lw=0.8, label=ax)
        axes[r, 0].set_ylabel("meas - cmd [mm]")
        axes[r, 0].set_title(f"{arm.name} TCP position error")
        axes[r, 0].legend(fontsize=7)
        axes[r, 1].plot(t, rot, lw=0.8)
        axes[r, 1].set_ylabel("[deg]")
        axes[r, 1].set_title(f"{arm.name} TCP orientation error")
    for ax in axes[-1]:
        ax.set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(ctx.out / "fig_tcp_error.png")
    plt.close(fig)
