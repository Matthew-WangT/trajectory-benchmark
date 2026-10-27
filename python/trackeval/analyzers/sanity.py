"""L0: sample timing, reference smoothness and saturation."""

from __future__ import annotations

import numpy as np

from .base import Analyzer, AnalyzerResult, Context, needs_joint_chain

SAT_RATIO = 0.95


class SanityAnalyzer(Analyzer):
    name = "sanity"
    stage = "L0"
    title = "Step 0  数据健全性"

    def applicable(self, ctx):
        return needs_joint_chain(ctx)

    def run(self, ctx: Context) -> AnalyzerResult:
        rep, d, model, names = ctx.rep, ctx.joints(), ctx.model, ctx.joint_names
        rep.section(self.title)
        dt = np.diff(d.t)
        dt_nom = float(np.median(dt))
        dropout = dt > 1.5 * dt_nom
        drops = int(np.sum(dropout))
        runs = np.diff(np.r_[False, dropout, False].astype(int))
        run_starts, run_ends = np.flatnonzero(runs == 1), np.flatnonzero(runs == -1)
        max_run = int(max((end - start for start, end in zip(run_starts, run_ends)), default=0))
        dropout_frac = float(dropout.mean())
        rep.add(f"样本数 {len(d.t)}, 时长 {d.t[-1] - d.t[0]:.2f} s, 采样 {1 / dt_nom:.1f} Hz")
        rep.add(
            f"dt 均值 {dt.mean() * 1e3:.3f} ms, std {dt.std() * 1e3:.3f} ms, max {dt.max() * 1e3:.2f} ms, "
            f"疑似丢包(dt>1.5*dt_nom) {drops} 次 ({100 * dropout_frac:.2f}%), 最长连续 {max_run} 个间隔"
        )
        rep.metric("sanity.fs_hz", 1 / dt_nom)
        rep.metric("sanity.dt_std_ms", dt.std() * 1e3)
        rep.metric("sanity.dropouts", drops)
        rep.metric("sanity.dropout_frac", dropout_frac)
        rep.metric("sanity.dropout_max_run", max_run)

        d2 = np.diff(d.q_cmd, n=2, axis=0)
        abs_d2 = np.abs(d2)
        worst_flat = int(np.argmax(abs_d2))
        worst_t, worst_joint = np.unravel_index(worst_flat, abs_d2.shape)
        rep.add(
            f"q_cmd discontinuity |d2q|: global p99 {np.percentile(abs_d2, 99):.2e}, "
            f"global max {abs_d2[worst_t, worst_joint]:.2e}, worst {names[worst_joint]} @ {d.t[worst_t + 1]:.2f}s"
        )
        rep.metric("sanity.cmd_d2q_p99_global", np.percentile(abs_d2, 99))
        rep.metric("sanity.cmd_d2q_max_global", abs_d2.max())
        if ctx.opts.verbose:
            rep.add("q_cmd 二阶差分 |d2q| (rad/sample^2): p99 / max @ t")
        for i, name in enumerate(names):
            k = int(np.argmax(np.abs(d2[:, i])))
            p99 = np.percentile(np.abs(d2[:, i]), 99)
            if ctx.opts.verbose:
                rep.add(f"  {name:13s} p99 {p99:.2e}  max {np.abs(d2[k, i]):.2e} @ {d.t[k + 1]:.2f}s")
            rep.metric(f"sanity.cmd_d2q_p99.{name}", p99)

        if model is None:
            rep.add("saturation: skipped, no URDF limits for this robot")
            return AnalyzerResult(data={"saturated": np.zeros(len(d.t), dtype=bool)})
        vel_sat = np.abs(d.qd_meas) > SAT_RATIO * model.vel_limit
        tau_sat = np.abs(d.tau_meas) > SAT_RATIO * model.effort_limit
        vel_frac, tau_frac = vel_sat.mean(axis=0), tau_sat.mean(axis=0)
        worst_vel, worst_tau = int(np.argmax(vel_frac)), int(np.argmax(tau_frac))
        vel_summary = "none" if vel_frac[worst_vel] == 0 else f"{names[worst_vel]} {100 * vel_frac[worst_vel]:.1f}%"
        tau_summary = "none" if tau_frac[worst_tau] == 0 else f"{names[worst_tau]} {100 * tau_frac[worst_tau]:.1f}%"
        rep.add(f"saturation (> {SAT_RATIO:.2f}*URDF limit): velocity {vel_summary}; torque {tau_summary}")
        if ctx.opts.verbose:
            rep.add(f"velocityLimit={np.round(model.vel_limit, 2).tolist()}")
            rep.add(f"effortLimit={np.round(model.effort_limit, 1).tolist()}")
            rep.add("饱和样本占比: 速度 / 力矩")
        for i, name in enumerate(names):
            if ctx.opts.verbose:
                rep.add(
                    f"  {name:13s} {100 * vel_sat[:, i].mean():5.1f}% / {100 * tau_sat[:, i].mean():5.1f}%   "
                    f"max|qd| {np.abs(d.qd_meas[:, i]).max():.2f} rad/s  "
                    f"max|tau| {np.abs(d.tau_meas[:, i]).max():.1f} Nm"
                )
            rep.metric(f"sanity.vel_sat_frac.{name}", vel_sat[:, i].mean())
            rep.metric(f"sanity.tau_sat_frac.{name}", tau_sat[:, i].mean())
        saturated = np.any(vel_sat | tau_sat, axis=1)
        rep.add(f"任一关节饱和的样本占比 {100 * saturated.mean():.1f}% (回归时剔除)")
        over = [names[i] for i in np.flatnonzero(np.abs(d.qd_meas).max(axis=0) > model.vel_limit)]
        if over:
            rep.add(f"  注: {over} 的实测速度超过 URDF velocityLimit, URDF 限幅不是真实硬件上限, 饱和占比仅供参考")

        return AnalyzerResult(data={"saturated": saturated})
