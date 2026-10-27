"""Configuration margins: distance to joint position limits, velocity / torque headroom, cond(J)."""

from __future__ import annotations

import numpy as np

from .base import Analyzer, AnalyzerResult, Context, needs_model

POS_MARGIN_WARN_RAD = 0.05
VEL_NEAR_RATIO = 0.8


class ConfigMarginAnalyzer(Analyzer):
    name = "margin"
    stage = "L4"
    title = "Step 4  构型余量: 位置限位 / 速度 / 力矩 / 奇异"

    def applicable(self, ctx):
        return needs_model(ctx)

    def run(self, ctx: Context) -> AnalyzerResult:
        rep, d, model, names = ctx.rep, ctx.joints(), ctx.model, ctx.joint_names
        rep.section(self.title)
        margin_cmd = np.minimum(d.q_cmd - model.lower, model.upper - d.q_cmd)
        margin_meas = np.minimum(d.q_meas - model.lower, model.upper - d.q_meas)
        vel_ratio = np.abs(d.qd_meas) / model.vel_limit
        tau_ratio = np.abs(d.tau_meas) / model.effort_limit
        rep.add(f"URDF 位置限位 [deg]: lower {np.round(np.degrees(model.lower), 1).tolist()}")
        rep.add(f"                     upper {np.round(np.degrees(model.upper), 1).tolist()}")
        rep.add(
            f"关节          限位余量 min cmd/meas [deg]  <{POS_MARGIN_WARN_RAD} rad 占比  "
            f"|qd|/limit peak/p99  near>{VEL_NEAR_RATIO} 占比  |tau|/limit peak"
        )
        binding = []
        for i, name in enumerate(names):
            m_cmd, m_meas = margin_cmd[:, i].min(), margin_meas[:, i].min()
            near = float(np.mean(margin_meas[:, i] < POS_MARGIN_WARN_RAD))
            v_peak, v_p99 = vel_ratio[:, i].max(), np.percentile(vel_ratio[:, i], 99)
            v_near = float(np.mean(vel_ratio[:, i] > VEL_NEAR_RATIO))
            t_peak = tau_ratio[:, i].max()
            flag = "  <<< 贴限位" if m_meas < POS_MARGIN_WARN_RAD else ""
            rep.add(
                f"  {name:13s} {np.degrees(m_cmd):7.1f} / {np.degrees(m_meas):6.1f}        {100 * near:5.1f}%         "
                f"{v_peak:5.2f} / {v_p99:5.2f}       {100 * v_near:5.1f}%        {t_peak:5.2f}{flag}"
            )
            rep.metric(f"margin.pos_min_rad.{name}", m_meas)
            rep.metric(f"margin.pos_near_frac.{name}", near)
            rep.metric(f"margin.vel_peak_ratio.{name}", v_peak)
            rep.metric(f"margin.vel_p99_ratio.{name}", v_p99)
            rep.metric(f"margin.vel_near_frac.{name}", v_near)
            rep.metric(f"margin.tau_peak_ratio.{name}", t_peak)
            if m_meas < POS_MARGIN_WARN_RAD:
                binding.append(name)
        rep.metric("margin.pos_min_rad.all", float(margin_meas.min()))
        rep.metric("margin.vel_peak_ratio.all", float(vel_ratio.max()))
        rep.metric("margin.tau_peak_ratio.all", float(tau_ratio.max()))
        if binding:
            rep.add(f"贴限位关节 {binding}: 这些关节的误差里含限位截断成分, 且姿态/基座调整会首先被它们卡住")
        ee = ctx.results.get("ee")
        if ee is not None:
            for arm in ctx.profile.arms:
                c = ee.data.cond[arm.name]
                rep.add(f"{arm.name} arm cond(J_pos) max {c.max():.1f} (median {np.median(c):.1f})")
        return AnalyzerResult(
            data={"margin_meas": margin_meas, "vel_ratio": vel_ratio, "tau_ratio": tau_ratio}, figures=[fig_margin]
        )


def fig_margin(ctx: Context) -> None:
    plt = ctx.plt()
    res = ctx.results["margin"].data
    names, t = ctx.joint_names, ctx.joints().t
    fig, axes = plt.subplots(1, 3, figsize=(18, 5))
    x = np.arange(len(names))
    ax = axes[0]
    ax.bar(x, np.degrees(res["margin_meas"].min(axis=0)), 0.6)
    ax.axhline(np.degrees(POS_MARGIN_WARN_RAD), color="r", ls=":", label=f"{POS_MARGIN_WARN_RAD} rad")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=45, fontsize=7)
    ax.set_ylabel("min distance to position limit [deg]")
    ax.set_title("joint position margin (measured)")
    ax.legend()
    ax = axes[1]
    ax.bar(x - 0.2, res["vel_ratio"].max(axis=0), 0.4, label="peak")
    ax.bar(x + 0.2, np.percentile(res["vel_ratio"], 99, axis=0), 0.4, label="p99")
    ax.axhline(1.0, color="r", ls=":")
    ax.set_xticks(x)
    ax.set_xticklabels(names, rotation=45, fontsize=7)
    ax.set_ylabel("|qd| / velocity limit")
    ax.set_title("velocity headroom")
    ax.legend()
    ax = axes[2]
    for i, name in enumerate(names):
        ax.plot(t, np.degrees(res["margin_meas"][:, i]), lw=0.8, label=name)
    ax.axhline(np.degrees(POS_MARGIN_WARN_RAD), color="r", ls=":")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("margin [deg]")
    ax.set_title("position margin over time")
    ax.legend(fontsize=6, ncol=2)
    fig.tight_layout()
    fig.savefig(ctx.out / "fig_margin.png")
    plt.close(fig)
