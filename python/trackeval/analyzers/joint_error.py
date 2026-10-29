"""L1: joint tracking error q_cmd - q_meas, the headline numbers of the benchmark."""

from __future__ import annotations

import numpy as np

from ..util import rms
from .base import Analyzer, AnalyzerResult, Context, needs_joint_chain


class JointErrorAnalyzer(Analyzer):
    name = "joint_error"
    stage = "L1"
    title = "Step 1  关节跟踪误差 q_cmd - q_meas"

    def applicable(self, ctx):
        return needs_joint_chain(ctx)

    def run(self, ctx: Context) -> AnalyzerResult:
        rep, d, names, p = ctx.rep, ctx.joints(), ctx.joint_names, ctx.profile
        rep.section(self.title)
        err = np.degrees(d.dq)
        abs_err = np.abs(err)
        rep.add(f"{len(d.t)} 样本, {d.t[-1] - d.t[0]:.2f} s, 单位 deg")
        rep.add("关节            RMS      p95|e|    max|e|   max|qd_cmd| [deg/s]")
        for i, name in enumerate(names):
            e_rms, e_p95, e_max = rms(err[:, i]), np.percentile(abs_err[:, i], 95), abs_err[:, i].max()
            qd_max = np.degrees(np.abs(d.qd_cmd[:, i]).max())
            rep.add(f"  {name:13s} {e_rms:7.3f}   {e_p95:7.3f}   {e_max:7.3f}   {qd_max:7.1f}")
            rep.metric(f"joint_error.rms_deg.{name}", e_rms)
            rep.metric(f"joint_error.p95_deg.{name}", e_p95)
            rep.metric(f"joint_error.max_deg.{name}", e_max)
        # Per joint group: the torso (if any) and the arms.
        for arm in p.groups:
            idx = p.joint_index(arm)
            arm_rms = rms(err[:, idx], axis=0)
            rep.add(
                f"{arm.name:5s}: RMS 均值 {arm_rms.mean():.3f} deg, 最大 RMS {arm_rms.max():.3f} deg "
                f"({names[idx[int(np.argmax(arm_rms))]]}), max|e| {abs_err[:, idx].max():.3f} deg"
            )
            rep.metric(f"joint_error.{arm.name}.rms_deg_mean", arm_rms.mean())
            rep.metric(f"joint_error.{arm.name}.rms_deg_max", arm_rms.max())
            rep.metric(f"joint_error.{arm.name}.max_deg", abs_err[:, idx].max())
        return AnalyzerResult(data=err, figures=[fig_joint_error])


def fig_joint_error(ctx: Context) -> None:
    plt = ctx.plt()
    d, names, arms = ctx.joints(), ctx.joint_names, ctx.profile.groups
    err = ctx.results["joint_error"].data
    fig, axes = plt.subplots(len(arms), 1, figsize=(14, 3.8 * len(arms)), squeeze=False, sharex=True)
    for r, arm in enumerate(arms):
        ax = axes[r, 0]
        for i in ctx.profile.joint_index(arm):
            ax.plot(d.t, err[:, i], lw=0.8, label=names[i])
        ax.set_ylabel("q_cmd - q_meas [deg]")
        ax.set_title(f"{arm.name} joint tracking error")
        ax.legend(fontsize=7, ncol=7)
    axes[-1, 0].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(ctx.out / "fig_joint_error.png")
    plt.close(fig)
