"""L5: health of the replay chain, from the raw records of the Marvin replay (raw_*.csv and run_info.json)."""

from __future__ import annotations

import json

import numpy as np

from ..csvio import read_csv
from .base import Analyzer, AnalyzerResult, Context


def _gaps_ms(t: np.ndarray) -> tuple[float, float, float]:
    dt = np.diff(t) * 1e3
    return float(1e3 / np.median(dt)), float(np.percentile(dt, 99)), float(dt.max())


def _joint_columns(header: list[str], values: np.ndarray, stem: str) -> np.ndarray:
    index = {h: i for i, h in enumerate(header)}
    return values[:, [index[f"{p}_{stem}_{j}"] for p in ("l", "r") for j in range(1, 8)]]


class HealthAnalyzer(Analyzer):
    name = "health"
    stage = "L5"
    title = "Step 5  链路健康度: 下发 / 反馈 / MPC"

    def applicable(self, ctx):
        if ctx.raw_dir is None or not (ctx.raw_dir / "raw_cmd.csv").is_file():
            return "no raw records of the replay (only the Marvin replay writes them)"
        return None

    def run(self, ctx: Context) -> AnalyzerResult:
        rep, raw = ctx.rep, ctx.raw_dir
        rep.section(self.title)
        info = json.loads((raw / "run_info.json").read_text())
        t0, t1 = info["s3_start_s"], info["s4_start_s"]
        rep.add(f"S3 窗口 [{t0:.3f}, {t1:.3f}] s (run 时钟); loop 超时 {info['loop_overruns']} 次, "
                f"最大滞后 {info['loop_max_lateness_s'] * 1e3:.2f} ms")
        rep.metric("health.loop_overruns", info["loop_overruns"])
        rep.metric("health.loop_max_lateness_ms", info["loop_max_lateness_s"] * 1e3)

        header, cmd, text = read_csv(raw / "raw_cmd.csv")
        col = {h: i for i, h in enumerate(header)}
        s3 = np.array([p == "S3" for p in text["phase"]])
        c = cmd[s3]
        rate, p99, gap = _gaps_ms(c[:, col["t_s"]])
        held = c[:, col["held"]] > 0.5
        age = c[:, col["policy_age_s"]] * 1e3
        age = age[age >= 0]
        clamps = c[-1, col["clamp_count"]] - c[0, col["clamp_count"]]
        rep.add(f"IK 评估: {rate:.0f} Hz, 间隔 p99 {p99:.2f} ms / max {gap:.2f} ms; hold {held.mean() * 100:.2f}% "
                f"(最长连续 {int(_longest_run(held))} 次); 限幅 {int(clamps)} 次")
        rep.add(f"policy age: p50 {np.median(age):.1f} ms, p95 {np.percentile(age, 95):.1f} ms, max {age.max():.1f} ms")
        rep.metric("health.eval_rate_hz", rate)
        rep.metric("health.eval_gap_max_ms", gap)
        rep.metric("health.hold_frac", held.mean())
        rep.metric("health.hold_max_run", _longest_run(held))
        rep.metric("health.clamp_count", clamps)
        rep.metric("health.policy_age_p50_ms", np.median(age))
        rep.metric("health.policy_age_p95_ms", np.percentile(age, 95))
        rep.metric("health.policy_age_max_ms", age.max())

        header_s, send, _ = read_csv(raw / "raw_send.csv")
        t_send = send[:, 0]
        in_s3 = (t_send >= t0) & (t_send < t1)
        rate, p99, gap = _gaps_ms(t_send[in_s3])
        rep.add(f"下发: {rate:.0f} Hz, 间隔 p99 {p99:.2f} ms / max {gap:.2f} ms")
        rep.metric("health.send_rate_hz", rate)
        rep.metric("health.send_gap_max_ms", gap)

        header_f, fb, _ = read_csv(raw / "raw_fb.csv")
        colf = {h: i for i, h in enumerate(header_f)}
        t_fb = fb[:, 0]
        in_s3_fb = (t_fb >= t0) & (t_fb < t1)
        rate, p99, gap = _gaps_ms(t_fb[in_s3_fb])
        miss = {p: fb[in_s3_fb, colf[f"{p}_frame_miss"]] for p in ("l", "r")}
        miss_delta = max(float(m.max() - m.min()) for m in miss.values())
        rep.add(f"反馈新帧: {rate:.0f} Hz, 间隔 p99 {p99:.2f} ms / max {gap:.2f} ms; 控制器丢帧计数增量 {miss_delta:.0f}")
        rep.metric("health.feedback_rate_hz", rate)
        rep.metric("health.feedback_gap_max_ms", gap)
        rep.metric("health.frame_miss_delta", miss_delta)

        # Command as received by the controller (RT_IN m_Joint_CMD_Pos) against what was sent: the transport delay.
        # Measured position against what was sent: the whole chain up to the motion.
        if "l_q_cmd_rx_1" in colf:
            q_send = _joint_columns(header_s, send, "q_send")
            rx = _joint_columns(header_f, fb, "q_cmd_rx")
            q_meas = _joint_columns(header_f, fb, "q")
            for name, series in (("cmd_rx", rx), ("meas", q_meas)):
                lag, err = _best_lag(t_send, q_send, t_fb, series, in_s3_fb)
                rep.add(f"下发 -> {name}: 最佳对齐 {lag * 1e3:.1f} ms, 残差 max {err:.4f} deg")
                rep.metric(f"health.send_to_{name}_lag_ms", lag * 1e3)
                rep.metric(f"health.send_to_{name}_err_max_deg", err)
        return AnalyzerResult()


def _longest_run(mask: np.ndarray) -> int:
    runs = np.diff(np.r_[False, mask, False].astype(int))
    starts, ends = np.flatnonzero(runs == 1), np.flatnonzero(runs == -1)
    return int(max((e - s for s, e in zip(starts, ends)), default=0))


def _best_lag(t_send, q_send, t_fb, series, mask, max_lag_s=0.5):
    """Delay of `series` behind the sent command, by the smallest max error over the lags."""
    best_lag, best_err = 0.0, np.inf
    for lag in np.arange(0.0, max_lag_s, 0.001):
        sent = np.column_stack([np.interp(t_fb[mask] - lag, t_send, q_send[:, j]) for j in range(q_send.shape[1])])
        err = float(np.degrees(np.abs(sent - series[mask])).max())
        if err < best_err:
            best_lag, best_err = lag, err
    return best_lag, best_err
