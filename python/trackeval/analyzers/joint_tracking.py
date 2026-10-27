"""L2: joint-level command -> state tracking.

delay      cross-correlation of velocities + frequency-domain phase fit (per joint)
slope      error-velocity regression (equivalent delay, catches non-delay error)
regression dq = a*qd + b*g(q) + c*qdd + d*sign(qd) + e with partial R^2
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..util import lstsq_r2, parabolic_peak, pearson, rms, smooth
from .base import Analyzer, AnalyzerResult, Context, needs_joint_chain

DELAY_RANGE_INCONSISTENT_MS = 10.0
DELAY_CV_INCONSISTENT = 0.3
SLOPE_VS_DELAY_MISMATCH = 0.3
EQUIV_DELAY_CORR_MIN = 0.85
DELAY_DISAGREE_WARN_MS = 10.0
PARTIAL_R2_DOMINANT = 0.5
SIGN_DEADBAND = 0.02
XCORR_MIN_PEAK = 0.5
XCORR_MIN_PROMINENCE = 0.05
REG_TERMS = ["qd", "g(q)", "qdd", "sign(qd)", "const"]


@dataclass
class SlidingXcorrWindows:
    delays_ms: list[np.ndarray]
    peak_corr: list[np.ndarray]
    peak_prominence: list[np.ndarray]
    valid_count: np.ndarray
    total_windows: int
    valid_fraction: np.ndarray


def sliding_xcorr_windows(
    command_velocity: np.ndarray,
    measured_velocity: np.ndarray,
    fs: float,
    max_lag_s: float,
    window_s: float,
) -> SlidingXcorrWindows:
    """Estimate delays and quality for each command-to-state xcorr window."""
    from scipy.signal import correlate, correlation_lags

    window_len = max(int(round(window_s * fs)), 16)
    window_step = max(window_len // 2, 1)
    max_lag = min(int(round(max_lag_s * fs)), window_len - 1)
    delays = [[] for _ in range(command_velocity.shape[1])]
    peaks = [[] for _ in range(command_velocity.shape[1])]
    prominences = [[] for _ in range(command_velocity.shape[1])]
    valid_counts = np.zeros(command_velocity.shape[1], dtype=int)
    total_windows = 0
    lags = correlation_lags(window_len, window_len)
    keep = np.abs(lags) <= max_lag
    kept_lags = lags[keep]
    for start in range(0, len(command_velocity) - window_len + 1, window_step):
        total_windows += 1
        stop = start + window_len
        for joint in range(command_velocity.shape[1]):
            command = command_velocity[start:stop, joint]
            measured = measured_velocity[start:stop, joint]
            command = command - command.mean()
            measured = measured - measured.mean()
            if np.std(command) < 1e-9 or np.std(measured) < 1e-9:
                continue
            corr = correlate(measured, command, mode="full")[keep]
            corr /= np.std(command) * np.std(measured) * window_len
            peak = int(np.argmax(corr))
            peak_value = corr[peak]
            exclusion = max(int(round(0.01 * fs)), 1)
            outside = np.concatenate((corr[: max(peak - exclusion, 0)], corr[peak + exclusion + 1 :]))
            prominence = peak_value - (outside.max() if outside.size else 0.0)
            peaks[joint].append(peak_value)
            prominences[joint].append(prominence)
            if peak_value < XCORR_MIN_PEAK or prominence < XCORR_MIN_PROMINENCE:
                continue
            delays[joint].append((parabolic_peak(corr, peak) + kept_lags[0]) / fs * 1e3)
            valid_counts[joint] += 1
    return SlidingXcorrWindows(
        [np.asarray(values) for values in delays],
        [np.asarray(values) for values in peaks],
        [np.asarray(values) for values in prominences],
        valid_counts,
        total_windows,
        valid_counts / max(total_windows, 1),
    )


def sliding_xcorr_delays_ms(
    command_velocity: np.ndarray,
    measured_velocity: np.ndarray,
    fs: float,
    max_lag_s: float,
    window_s: float,
) -> list[np.ndarray]:
    """Estimate quality-filtered per-window command-to-state delays in milliseconds."""
    return sliding_xcorr_windows(command_velocity, measured_velocity, fs, max_lag_s, window_s).delays_ms


@dataclass
class DelayResult:
    tau_xcorr: np.ndarray
    tau_xcorr_p95: np.ndarray
    tau_fd: np.ndarray
    xcorr_lags: np.ndarray
    xcorr_curves: np.ndarray
    freq: np.ndarray
    gain: np.ndarray
    phase: np.ndarray
    coh: np.ndarray


@dataclass
class RegResult:
    coef: np.ndarray
    r2: np.ndarray
    partial_r2: np.ndarray
    resid_ratio: np.ndarray
    resid_peak_hz: np.ndarray
    dominant: list[str]


@dataclass
class JointResult:
    delay: DelayResult
    slope_ms: np.ndarray
    corr: np.ndarray
    reg: RegResult


def delay_inconsistent(tau_ms: np.ndarray) -> bool:
    mean = abs(tau_ms.mean())
    return (tau_ms.max() - tau_ms.min()) > DELAY_RANGE_INCONSISTENT_MS or (
        mean > 1e-6 and tau_ms.std() / mean > DELAY_CV_INCONSISTENT
    )


class JointTrackingAnalyzer(Analyzer):
    name = "joint"
    stage = "L2"
    title = "Step 2  关节层: 指令 -> 状态"

    def applicable(self, ctx):
        return needs_joint_chain(ctx)

    def run(self, ctx: Context) -> AnalyzerResult:
        rep, d, names = ctx.rep, ctx.joints(), ctx.joint_names
        rep.section(self.title)
        delay = self._delay(ctx)
        slope, corr = self._error_velocity(ctx, delay)
        sat = ctx.results.get("sanity")
        saturated = sat.data["saturated"] if sat else np.zeros(len(d.t), dtype=bool)
        reg = self._regression(ctx, saturated)
        res = JointResult(delay, slope, corr, reg)
        return AnalyzerResult(data=res, figures=[lambda c, r=res: fig_delay(c, r.delay)])

    # ---------------------------------------------------------------- delay
    def _delay(self, ctx: Context) -> DelayResult:
        from scipy.signal import coherence, correlate, correlation_lags, csd, welch

        rep, d, names, o = ctx.rep, ctx.joints(), ctx.joint_names, ctx.opts
        fs = d.fs
        x_all = smooth(np.gradient(d.q_cmd, d.t, axis=0))  # 指令速度用数值微分, 消息里的 velocity 不是真实导数
        y_all = smooth(d.qd_meas)
        n = len(d.t)
        max_k = int(round(o.max_lag_s * fs))
        lags_all = correlation_lags(n, n)
        keep = np.abs(lags_all) <= max_k
        lags = lags_all[keep] / fs
        nper = min(1024, n)
        nj = len(names)
        tau_x, tau_x_p95, tau_f = np.zeros(nj), np.full(nj, np.nan), np.full(nj, np.nan)
        window_s = 5.0
        win_len = max(int(round(window_s * fs)), 16)
        win_step = max(win_len // 2, 1)
        xcorr_windows = sliding_xcorr_windows(x_all, y_all, fs, o.max_lag_s, window_s)
        tau_windows = xcorr_windows.delays_ms
        curves = np.zeros((nj, keep.sum()))
        gain = phase = coh_all = freq = None
        rep.add(
            f"延迟估计: 滑窗 {win_len / fs:.2f}s / 步长 {win_step / fs:.2f}s, "
            f"max_lag {o.max_lag_s * 1e3:.0f} ms, 质量阈值 peak>={XCORR_MIN_PEAK}, "
            f"prom>={XCORR_MIN_PROMINENCE}, nperseg {nper}, 相干性阈值 {o.coh_min}, 频段 {o.band_hz} Hz"
        )
        rep.add("tau_phasefit: 在高相干频段中，以传递函数相位 phi(f) ≈ -2*pi*f*tau 做最小二乘拟合。")
        rep.add("关节          tau_xcorr(p50/p95)  valid/total  peak/prom  tau_phasefit  disagree  |G|@1Hz  ph@1Hz")
        for i in range(nj):
            x = x_all[:, i] - x_all[:, i].mean()
            y = y_all[:, i] - y_all[:, i].mean()
            c = correlate(y, x, mode="full")[keep] / (np.std(x) * np.std(y) * n + 1e-18)  # 正 lag = y 滞后 x
            curves[i] = c
            k = int(np.argmax(c))
            global_tau = parabolic_peak(c, k) / fs + lags[0]
            samples = tau_windows[i]
            tau_x[i] = np.percentile(samples, 50) / 1e3 if samples.size else global_tau
            tau_x_p95[i] = np.percentile(samples, 95) if samples.size else global_tau * 1e3
            f, pxy = csd(x, y, fs=fs, nperseg=nper)
            _, pxx = welch(x, fs=fs, nperseg=nper)
            _, coh = coherence(x, y, fs=fs, nperseg=nper)
            g = pxy / (pxx + 1e-18)
            if gain is None:
                freq = f
                gain, phase, coh_all = (np.zeros((nj, len(f))) for _ in range(3))
            gain[i], phase[i], coh_all[i] = np.abs(g), np.unwrap(np.angle(g)), coh
            m = (coh > o.coh_min) & (f >= o.band_hz[0]) & (f <= o.band_hz[1])
            if m.sum() < 3:
                m = (coh > 0.7) & (f >= o.band_hz[0]) & (f <= o.band_hz[1])
            if m.sum() >= 3:
                tau_f[i] = -np.sum(phase[i][m] * f[m]) / (2 * np.pi * np.sum(f[m] ** 2))

            def at(arr, f0):
                return arr[int(np.argmin(np.abs(freq - f0)))]

            p50, p95 = np.percentile(samples, [50, 95]) if samples.size else [global_tau * 1e3, global_tau * 1e3]
            peak = np.median(xcorr_windows.peak_corr[i]) if xcorr_windows.peak_corr[i].size else float("nan")
            prominence = (
                np.median(xcorr_windows.peak_prominence[i]) if xcorr_windows.peak_prominence[i].size else float("nan")
            )
            disagreement = abs(tau_x[i] * 1e3 - tau_f[i] * 1e3)
            warn = " WARN" if np.isfinite(disagreement) and disagreement > DELAY_DISAGREE_WARN_MS else ""
            rep.add(
                f"  {names[i]:13s} {p50:6.1f}/{p95:6.1f}ms "
                f"{xcorr_windows.valid_count[i]:3d}/{xcorr_windows.total_windows:<3d} "
                f"{peak:4.2f}/{prominence:4.2f} {tau_f[i] * 1e3:7.1f}ms {disagreement:7.1f}ms{warn}  "
                f"{at(gain[i], 1):6.3f} {np.degrees(at(phase[i], 1)):7.1f}deg"
            )
            rep.metric(f"joint.delay_xcorr_ms.{names[i]}", tau_x[i] * 1e3)
            rep.metric(f"joint.delay_xcorr_p95_ms.{names[i]}", tau_x_p95[i])
            rep.metric(f"joint.xcorr_valid_frac.{names[i]}", xcorr_windows.valid_fraction[i])
            rep.metric(f"joint.xcorr_peak_median.{names[i]}", peak)
            rep.metric(f"joint.xcorr_prominence_median.{names[i]}", prominence)
            rep.metric(f"joint.delay_fd_ms.{names[i]}", tau_f[i] * 1e3)
            rep.metric(f"joint.delay_phasefit_ms.{names[i]}", tau_f[i] * 1e3)
            rep.metric(f"joint.delay_disagree_ms.{names[i]}", disagreement)
        for arm in ctx.profile.arms:
            tx = tau_x[ctx.profile.joint_index(arm)] * 1e3
            tag = "不一致" if delay_inconsistent(tx) else "一致"
            arm_windows = [tau_windows[i] for i in ctx.profile.joint_index(arm) if tau_windows[i].size]
            window_values = np.concatenate(arm_windows) if arm_windows else np.empty(0)
            arm_p50 = np.percentile(window_values, 50) if window_values.size else np.median(tx)
            arm_p95 = np.percentile(window_values, 95) if window_values.size else np.percentile(tx, 95)
            delay_jitter = arm_p95 - arm_p50
            joint_p95 = tau_x_p95[ctx.profile.joint_index(arm)]
            p95_range = joint_p95.max() - joint_p95.min()
            arm_idx = ctx.profile.joint_index(arm)
            valid_fraction = np.mean(xcorr_windows.valid_fraction[arm_idx])
            valid_count = int(np.sum(xcorr_windows.valid_count[arm_idx]))
            total_count = xcorr_windows.total_windows * len(arm_idx)
            arm_peaks = [np.median(xcorr_windows.peak_corr[i]) for i in arm_idx if xcorr_windows.peak_corr[i].size]
            arm_prominences = [
                np.median(xcorr_windows.peak_prominence[i]) for i in arm_idx if xcorr_windows.peak_prominence[i].size
            ]
            peak = np.median(arm_peaks) if arm_peaks else float("nan")
            prominence = np.median(arm_prominences) if arm_prominences else float("nan")
            rep.add(
                f"{arm.name:5s} arm tau_xcorr: P50/P95 {arm_p50:.1f}/{arm_p95:.1f} ms "
                f"(jitter {delay_jitter:.1f} ms, valid {valid_count}/{total_count}, "
                f"peak/prom {peak:.2f}/{prominence:.2f}), "
                f"关节 P95 极差 {p95_range:.1f} ms -> {tag}"
            )
            rep.metric(f"joint.{arm.name}.delay_p50_ms", arm_p50)
            rep.metric(f"joint.{arm.name}.delay_p95_ms", arm_p95)
            rep.metric(f"joint.{arm.name}.delay_jitter_ms", delay_jitter)
            rep.metric(f"joint.{arm.name}.delay_p95_range_ms", p95_range)
            rep.metric(f"joint.{arm.name}.xcorr_valid_frac", valid_fraction)
            rep.metric(f"joint.{arm.name}.xcorr_peak_median", peak)
            rep.metric(f"joint.{arm.name}.xcorr_prominence_median", prominence)
        return DelayResult(tau_x, tau_x_p95, tau_f, lags, curves, freq, gain, phase, coh_all)

    # ---------------------------------------------------------------- slope
    def _error_velocity(self, ctx: Context, delay: DelayResult):
        rep, d, names = ctx.rep, ctx.joints(), ctx.joint_names
        qd = np.gradient(d.q_cmd, d.t, axis=0)
        r, slope = np.zeros(len(names)), np.zeros(len(names))
        rep.add(
            "两种延迟: tau_xcorr 是指令/实测速度互相关得到的实际信号滞后；"
            "等效延迟 slope 是 dq ~ tau*qd_cmd 的回归斜率，表示延迟型误差的等效时间，"
            "两者不一致说明还存在带宽、阻尼或其他非延迟误差。"
        )
        if ctx.opts.verbose:
            rep.add("dq ~ qd_cmd corr（corr < 0.85 时等效延迟仅供参考）")
        for i, name in enumerate(names):
            r[i] = pearson(qd[:, i], d.dq[:, i])
            slope[i] = np.polyfit(qd[:, i], d.dq[:, i], 1)[0] * 1e3 if np.std(qd[:, i]) > 1e-9 else float("nan")
            tx = delay.tau_xcorr[i] * 1e3
            mismatch = abs(slope[i] - tx) / max(abs(tx), 1e-3)
            if ctx.opts.verbose:
                note = (
                    f"corr={r[i]:.3f} < {EQUIV_DELAY_CORR_MIN:.2f} -> 等效延迟仅供参考"
                    if r[i] < EQUIV_DELAY_CORR_MIN
                    else f"slope-vs-xcorr difference {100 * mismatch:.0f}%"
                )
                rep.add(f"  {name:13s} {note}")
            rep.metric(f"joint.err_vel_corr.{name}", r[i])
        for arm in ctx.profile.arms:
            values = slope[ctx.profile.joint_index(arm)]
            values = values[np.isfinite(values)]
            if values.size:
                rep.add(
                    f"{arm.name:5s} arm 等效延迟: median {np.median(values):.1f} ms, "
                    f"P95 {np.percentile(values, 95):.1f} ms"
                )
                rep.metric(f"joint.{arm.name}.equiv_delay_median_ms", np.median(values))
                rep.metric(f"joint.{arm.name}.equiv_delay_p95_ms", np.percentile(values, 95))
        return slope, r

    # ---------------------------------------------------------------- regression
    def _regression(self, ctx: Context, saturated: np.ndarray) -> RegResult:
        from scipy.signal import welch

        rep, d, names, model, o = ctx.rep, ctx.joints(), ctx.joint_names, ctx.model, ctx.opts
        qd = smooth(np.gradient(d.q_cmd, d.t, axis=0))
        qdd = smooth(np.gradient(qd, d.t, axis=0))
        # Without a URDF the gravity column is zero, its partial R^2 then reads 0.
        g = model.gravity_series(d.q_meas) if model is not None else np.zeros_like(d.q_meas)
        sgn = np.where(np.abs(qd) > SIGN_DEADBAND, np.sign(qd), 0.0)
        ok = ~saturated
        nj = len(names)
        coef, r2, pr2 = np.zeros((nj, 5)), np.zeros(nj), np.zeros((nj, 5))
        resid_ratio, resid_peak, dominant = np.zeros(nj), np.zeros(nj), []
        rep.add(f"误差成分回归 dq = a*qd + b*g(q) + c*qdd + d*sign(qd) + e; 剔除饱和后 {int(ok.sum())}/{len(ok)} 样本")
        rep.add("关节          R2     偏R2: qd     g(q)    qdd    sign   共享  | resid/dq  resid峰值  主导项")
        for i, name in enumerate(names):
            X = np.column_stack([qd[ok, i], g[ok, i], qdd[ok, i], sgn[ok, i], np.ones(ok.sum())])
            y = d.dq[ok, i]
            beta, res, full = lstsq_r2(X, y)
            coef[i], r2[i] = beta, full
            for j in range(5):
                pr2[i, j] = full - lstsq_r2(np.delete(X, j, axis=1), y)[2]
            resid_ratio[i] = rms(res) / max(rms(y), 1e-12)
            f, p = welch(res, fs=d.fs, nperseg=min(1024, len(res)))
            m = (f >= o.band_hz[0]) & (f <= o.band_hz[1])
            resid_peak[i] = f[m][np.argmax(p[m])]
            jmax = int(np.argmax(pr2[i, :4]))
            unique_sum = max(float(np.sum(pr2[i, :4])), 1e-9)  # 偏 R^2 只计唯一方差, 主导判据按其占比
            dom = (
                REG_TERMS[jmax]
                if pr2[i, jmax] > PARTIAL_R2_DOMINANT * unique_sum and pr2[i, jmax] > 0.1
                else "无单一主导"
            )
            if full < 0.3:
                dom = "模型外因素 (R2<0.3)"
            dominant.append(dom)
            rep.add(
                f"  {name:13s} {full:5.2f}   {pr2[i, 0]:6.3f} {pr2[i, 1]:6.3f} {pr2[i, 2]:6.3f} {pr2[i, 3]:6.3f} "
                f"{full - np.sum(pr2[i]):5.2f} | {resid_ratio[i]:6.2f}   {resid_peak[i]:5.2f} Hz   {dom}"
            )
            rep.metric(f"joint.reg_r2.{name}", full)
            rep.metric(f"joint.reg_resid_ratio.{name}", resid_ratio[i])
            for j, term in enumerate(("qd", "g", "qdd", "sign")):
                rep.metric(f"joint.reg_partial_r2_{term}.{name}", pr2[i, j])
        return RegResult(coef, r2, pr2, resid_ratio, resid_peak, dominant)


def fig_delay(ctx: Context, delay: DelayResult) -> None:
    plt = ctx.plt()
    arms, names = ctx.profile.arms, ctx.joint_names
    fig, axes = plt.subplots(len(arms), 3, figsize=(17, 4.6 * len(arms)), squeeze=False)
    for r, arm in enumerate(arms):
        joints = ctx.profile.joint_index(arm)
        ax = axes[r, 0]
        for i in joints:
            ax.plot(delay.xcorr_lags * 1e3, delay.xcorr_curves[i], label=names[i])
            ax.axvline(delay.tau_xcorr[i] * 1e3, color=ax.lines[-1].get_color(), ls=":", lw=0.8)
        ax.set_xlabel("lag [ms]  (positive = measured lags command)")
        ax.set_ylabel("normalized xcorr of velocities")
        ax.set_title(f"{arm.name} arm: cross-correlation")
        ax.legend(fontsize=7)
        ax = axes[r, 1]
        x = np.arange(len(joints))
        ax.bar(x - 0.2, delay.tau_xcorr[joints] * 1e3, 0.4, label="xcorr")
        ax.bar(x + 0.2, delay.tau_fd[joints] * 1e3, 0.4, label="freq-domain")
        ax.set_xticks(x)
        ax.set_xticklabels([names[i] for i in joints], rotation=30)
        ax.set_ylabel("delay [ms]")
        ax.set_title("per-joint delay")
        ax.legend()
        ax = axes[r, 2]
        ax2 = ax.twinx()
        m = (delay.freq > 0) & (delay.freq <= 10)
        for i in joints:
            (line,) = ax.plot(delay.freq[m], delay.gain[i][m], label=names[i])
            ax2.plot(delay.freq[m], np.degrees(delay.phase[i][m]), color=line.get_color(), ls="--", lw=0.8)
            ax.fill_between(delay.freq[m], 0, 2, where=delay.coh[i][m] < 0.9, color="grey", alpha=0.04)
        ax.set_ylim(0, 1.6)
        ax.set_xlabel("f [Hz]")
        ax.set_ylabel("|G| (solid)")
        ax2.set_ylabel("phase [deg] (dashed)")
        ax2.grid(False)
        ax.set_title("transfer function q_meas/q_cmd (grey: coherence<0.9)")
        ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(ctx.out / "fig_joint_delay.png")
    plt.close(fig)
