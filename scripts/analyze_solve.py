#!/usr/bin/env python3
"""Evaluates a trajectory solve: the TCP target against FK(q_cmd), joint limit margins and the joint velocity and
acceleration distributions of the IK controller commands.

    python3 scripts/analyze_solve.py runs/case02_fold_clothes_spd50/solve_marvin_20261029_101500

Reads run_*/raw_cmd.csv of `replay --solve`. The statistics are over S3 (the case trajectory), the violations over
S1..S4. Velocity: central difference of q_cmd on the 2 ms evaluation grid. Acceleration: central difference of the
velocity of q_cmd low passed (4th order Butterworth, zero phase, robot.yaml solve.acc_lowpass_hz); the raw second
difference is kept to show the steps of the MPC policy updates. Distributions give rms / p50 / p95 / p99 / max of the
magnitude. Writes per run solve.csv and plots/, and for the session metrics.json (the worst run per metric),
report.txt and config_snapshot/.
"""

from __future__ import annotations

import argparse
import json
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from trackeval.analyzers.tcp_tracking import pos_rms_at_lag  # noqa: E402
from trackeval.csvio import read_csv  # noqa: E402
from trackeval.model import ArmModel  # noqa: E402
from trackeval.report import Report  # noqa: E402
from trackeval.robots import REPO_ROOT, load_profile, read_yaml  # noqa: E402
from trackeval.solve_inputs import input_files, input_hashes  # noqa: E402
from trackeval.util import quat_angle_deg, rms  # noqa: E402

POSE = ("x", "y", "z", "qx", "qy", "qz", "qw")
ARMS = (("left", "l"), ("right", "r"))
STATS = ("rms", "p50", "p95", "p99", "max")
LAGS_S = np.arange(-0.1, 0.2 + 1e-9, 0.001)
PHASE_COLORS = {"S1": "#dbe9f6", "S2": "#eeeeee", "S4": "#eeeeee"}


def stats(x: np.ndarray) -> dict[str, np.ndarray]:
    """rms / p50 / p95 / p99 / max of |x| along the first axis."""
    a = np.abs(x)
    return {"rms": rms(a, axis=0), "p50": np.percentile(a, 50, axis=0), "p95": np.percentile(a, 95, axis=0),
            "p99": np.percentile(a, 99, axis=0), "max": a.max(axis=0)}


def continuous_quat(quat: np.ndarray) -> np.ndarray:
    out = quat.copy()
    for i in range(1, len(out)):
        if np.dot(out[i], out[i - 1]) < 0:
            out[i] = -out[i]
    return out


def first_violation(t: np.ndarray, mask: np.ndarray, names: list[str]) -> dict:
    """Count of samples (N,J) beyond the limit, the first time and the joints involved."""
    rows = mask.any(axis=1)
    if not rows.any():
        return {"count": 0, "first_t_s": None, "joints": []}
    return {"count": int(rows.sum()), "first_t_s": float(t[np.argmax(rows)]),
            "joints": [names[j] for j in np.flatnonzero(mask.any(axis=0))]}


class Solve:
    """One run of the solve, resampled on the uniform evaluation grid."""

    def __init__(self, run: Path, profile, model: ArmModel, lowpass_hz: float):
        from scipy.signal import butter, filtfilt

        self.run = run
        self.info = json.loads((run / "run_info.json").read_text())
        header, values, text = read_csv(run / "raw_cmd.csv")
        col = {h: i for i, h in enumerate(header)}
        evaluated = values[:, col["evaluated"]] > 0
        values = values[evaluated]
        phase = np.array(text["phase"])[evaluated]
        t_raw = values[:, col["t_s"]]
        self.held = values[:, col["held"]] > 0
        self.t_raw = t_raw
        dt = self.info["tick_s"] * self.info["evaluate_every_ticks"]
        self.dt = dt
        self.t = np.arange(t_raw[0], t_raw[-1] + 1e-9, dt)
        hold = np.clip(np.searchsorted(t_raw, self.t, side="right") - 1, 0, len(t_raw) - 1)
        self.phase = phase[hold]
        q_names = [f"{g.prefix}_q_cmd_{j}" for g in profile.groups for j in range(1, len(g.joints) + 1)]
        q_raw = values[:, [col[n] for n in q_names]]
        self.q_raw = q_raw
        self.q = np.column_stack([np.interp(self.t, t_raw, q_raw[:, j]) for j in range(q_raw.shape[1])])
        self.target = {}
        for arm, p in ARMS:
            target = values[:, [col[f"{p}_target_{f}"] for f in POSE]][hold]
            target[:, 3:] = continuous_quat(target[:, 3:])
            self.target[arm] = target
        poses = model.frame_poses(self.q, [a.ee_frame for a in profile.arms])
        self.solved = {}
        for a in profile.arms:
            pose = poses[a.ee_frame]
            pose[:, 3:] = continuous_quat(pose[:, 3:])
            self.solved[a.name] = pose

        self.dq = np.gradient(self.q, dt, axis=0)
        self.ddq_raw = np.zeros_like(self.q)
        self.ddq_raw[1:-1] = (self.q[2:] - 2 * self.q[1:-1] + self.q[:-2]) / dt**2
        b, a_ = butter(4, lowpass_hz / (0.5 / dt))
        q_lp = filtfilt(b, a_, self.q, axis=0)
        self.ddq = np.gradient(np.gradient(q_lp, dt, axis=0), dt, axis=0)
        self.s3 = self.phase == "S3"
        self.ik = self.phase != "S0"


def analyze(solve: Solve, profile, model: ArmModel, cfg: dict, controller: dict, rep: Report) -> dict:
    """Metrics of one run, also written to `rep`."""
    m: dict = {}
    names = list(profile.joint_order)
    s3, t = solve.s3, solve.t
    deg = np.degrees

    rep.section("求解误差 (S3): 目标 TCP -> FK(q_cmd)")
    rep.add("误差 = 求解 - 目标. 位置 [mm] 各轴与模长, 姿态 [deg]; 时间对齐 = 平移目标抵消固定时延后的残差")
    for arm, _ in ARMS:
        target, solved = solve.target[arm][s3], solve.solved[arm][s3]
        pos = (solved[:, :3] - target[:, :3]) * 1e3
        norm = np.linalg.norm(pos, axis=1)
        rot = quat_angle_deg(target[:, 3:], solved[:, 3:])
        curve = np.array([pos_rms_at_lag(t[s3], target[:, :3], solved[:, :3], lag) for lag in LAGS_S]) * 1e3
        best = int(np.argmin(curve))
        st_axes, st_norm, st_rot = stats(pos), stats(norm), stats(rot)
        for k, ax in enumerate("xyz"):
            rep.add(f"  {arm:5s} {ax}  " + "  ".join(f"{s} {st_axes[s][k]:6.2f}" for s in STATS))
            for s in STATS:
                m[f"tcp.{arm}.pos_{s}_mm.{ax}"] = float(st_axes[s][k])
        rep.add(f"  {arm:5s} |e| " + "  ".join(f"{s} {st_norm[s]:6.2f}" for s in STATS) + " mm")
        rep.add(f"  {arm:5s} 姿态 " + "  ".join(f"{s} {st_rot[s]:6.3f}" for s in STATS) + " deg")
        rep.add(f"  {arm:5s} 等效时延 {LAGS_S[best] * 1e3:.0f} ms, 对齐后 |e| RMS {curve[best]:.2f} mm "
                f"(未对齐 {curve[np.argmin(np.abs(LAGS_S))]:.2f} mm)")
        for s in STATS:
            m[f"tcp.{arm}.pos_{s}_mm.norm"] = float(st_norm[s])
            m[f"tcp.{arm}.rot_{s}_deg"] = float(st_rot[s])
        m[f"tcp.{arm}.equiv_delay_ms"] = float(LAGS_S[best] * 1e3)
        m[f"tcp.{arm}.pos_rms_aligned_mm"] = float(curve[best])

    margin = cfg["solve"].get("joint_limit_margin_rad", 0.02)
    lower, upper = model.lower, model.upper
    vel_limit = model.vel_limit
    acc_limit = np.array(profile.max_acc) if profile.max_acc else None
    q3, dq3, ddq3, raw3 = solve.q[s3], solve.dq[s3], solve.ddq[s3], solve.ddq_raw[s3]
    pos_margin = np.minimum(q3 - lower, upper - q3).min(axis=0)
    st_v, st_a, st_raw = stats(dq3), stats(ddq3), stats(raw3)

    rep.section("关节余量与分布 (S3)")
    rep.add("位置余量 = 到 URDF 限位的最小距离; 速度/加速度余量 = 1 - max|x| / 限值. 速度 [deg/s], 加速度 [deg/s^2]")
    w = max(10, *(len(n) for n in names))
    rep.add(f"  {'关节':{w}s} {'位置余量 deg (%行程)':>20s} {'速度 rms/p50/p95/p99/max':>36s} {'余量':>6s}"
            f" {'加速度 rms/p50/p95/p99/max':>40s} {'余量':>6s} {'原始 p99/max':>16s}")
    for j, name in enumerate(names):
        span = upper[j] - lower[j]
        vel_margin = 1 - st_v["max"][j] / vel_limit[j]
        acc_margin = 1 - st_a["max"][j] / acc_limit[j] if acc_limit is not None else None
        v = "/".join(f"{deg(st_v[s][j]):.0f}" for s in STATS)
        a = "/".join(f"{deg(st_a[s][j]):.0f}" for s in STATS)
        rep.add(f"  {name:{w}s} {deg(pos_margin[j]):9.1f} ({100 * pos_margin[j] / span:4.1f}%)  {v:>34s} "
                f"{100 * vel_margin:5.0f}% {a:>40s} {'-' if acc_margin is None else f'{100 * acc_margin:5.0f}%':>6s}"
                f" {deg(st_raw['p99'][j]):7.0f}/{deg(st_raw['max'][j]):.0f}")
        m[f"margin.pos_deg.{name}"] = float(deg(pos_margin[j]))
        m[f"margin.pos_pct.{name}"] = float(100 * pos_margin[j] / span)
        m[f"margin.vel_pct.{name}"] = float(100 * vel_margin)
        if acc_margin is not None:
            m[f"margin.acc_pct.{name}"] = float(100 * acc_margin)
        for s in STATS:
            m[f"vel.{s}_deg_s.{name}"] = float(deg(st_v[s][j]))
            m[f"acc.{s}_deg_s2.{name}"] = float(deg(st_a[s][j]))
        m[f"acc_raw.p99_deg_s2.{name}"] = float(deg(st_raw["p99"][j]))
        m[f"acc_raw.max_deg_s2.{name}"] = float(deg(st_raw["max"][j]))
    for group in profile.groups:
        arm, idx = group.name, [names.index(n) for n in group.joints]
        worst = idx[int(np.argmin(pos_margin[idx]))]
        m[f"margin.{arm}.pos_deg_min"] = float(deg(pos_margin[worst]))
        m[f"margin.{arm}.pos_joint_min"] = names[worst]
        m[f"margin.{arm}.vel_pct_min"] = float(min(m[f"margin.vel_pct.{names[j]}"] for j in idx))
        if acc_limit is not None:
            m[f"margin.{arm}.acc_pct_min"] = float(min(m[f"margin.acc_pct.{names[j]}"] for j in idx))
        m[f"vel.{arm}.p99_deg_s_max"] = float(max(m[f"vel.p99_deg_s.{names[j]}"] for j in idx))
        m[f"vel.{arm}.max_deg_s"] = float(max(m[f"vel.max_deg_s.{names[j]}"] for j in idx))
        m[f"acc.{arm}.p99_deg_s2_max"] = float(max(m[f"acc.p99_deg_s2.{names[j]}"] for j in idx))
        m[f"acc.{arm}.max_deg_s2"] = float(max(m[f"acc.max_deg_s2.{names[j]}"] for j in idx))

    # Violations over the IK controlled phases S1..S4, on the raw evaluations for the command step and the holds.
    ik = solve.ik
    t_ik = t[ik]
    q_ik = solve.q[ik]
    step_limit = controller.get("max_joint_velocity_rad_s", np.inf)
    step = np.abs(np.diff(solve.q_raw, axis=0)) / np.maximum(np.diff(solve.t_raw), 1e-6)[:, None]
    held = solve.held
    runs = np.diff(np.flatnonzero(np.diff(np.r_[0, held.astype(int), 0])))[::2] if held.any() else np.array([0])
    violations = {
        "position": first_violation(t_ik, (q_ik < lower + margin) | (q_ik > upper - margin), names),
        "velocity": first_violation(t_ik, np.abs(solve.dq[ik]) > vel_limit, names),
        "acceleration": first_violation(t_ik, np.abs(solve.ddq[ik]) > acc_limit, names)
        if acc_limit is not None else {"count": 0, "first_t_s": None, "joints": []},
        "command_step": first_violation(solve.t_raw[1:], step > step_limit + 1e-9, names),
        "hold": {"count": int(held.sum()), "first_t_s": float(solve.t_raw[np.argmax(held)]) if held.any() else None,
                 "longest": int(runs.max())},
    }
    rep.section("违规 (S1..S4，只记录不中止)")
    labels = {"position": f"位置越限 (URDF 限位内缩 {margin} rad)", "velocity": "速度超 URDF velocity",
              "acceleration": "加速度超 robot.yaml max_acc", "command_step": "命令步长超 max_joint_velocity_rad_s",
              "hold": "IK hold"}
    for key, v in violations.items():
        first = "-" if v["first_t_s"] is None else f"{v['first_t_s']:.3f} s"
        extra = f", 最长连续 {v['longest']}" if key == "hold" else (f", 关节 {v['joints']}" if v["joints"] else "")
        rep.add(f"  {labels[key]:40s} {v['count']:6d} 次, 首次 {first}{extra}")
        m[f"violation.{key}.count"] = v["count"]

    # One limit for all joints (the PD velocity feedforward of Marvin), or without it the URDF limit of each joint. The
    # precheck reports the joint closest to its limit.
    limits = np.full(len(names), cfg["solve"]["precheck_max_joint_vel_deg_s"]) \
        if "precheck_max_joint_vel_deg_s" in cfg["solve"] else deg(vel_limit)
    peaks = deg(np.abs(solve.dq[ik]).max(axis=0))
    j = int(np.argmax(peaks / limits))
    peak, limit = float(peaks[j]), float(limits[j])
    reasons = []
    if peak >= limit:
        reasons.append(f"{names[j]} 速度峰值 {peak:.1f} deg/s 不低于 {limit:.0f} deg/s")
    if violations["position"]["count"]:
        reasons.append(f"位置越限 {violations['position']['count']} 次")
    if solve.info["status"] != "ok":
        reasons.append(f"求解失败: {solve.info['reason']}")
    precheck = {"pass": not reasons, "max_joint_vel_deg_s": peak, "limit_deg_s": limit, "joint": names[j],
                "position_violations": violations["position"]["count"], "reason": "; ".join(reasons)}
    rep.section("真机预检")
    rep.add(f"  S1..S4 关节速度最接近限值: {names[j]} {peak:.1f} deg/s (限 {limit:.0f}), 位置越限 "
            f"{violations['position']['count']} 次 -> {'通过' if precheck['pass'] else '不通过: ' + precheck['reason']}")
    m["precheck.max_joint_vel_deg_s"] = peak
    return {"metrics": m, "violations": violations, "precheck": precheck}


def shade(ax, solve: Solve) -> None:
    for phase, color in PHASE_COLORS.items():
        mask = solve.phase == phase
        if mask.any():
            ax.axvspan(solve.t[mask][0], solve.t[mask][-1], color=color, lw=0, zorder=0)


def plot(solve: Solve, profile, model: ArmModel, out: Path, dpi: int) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    out.mkdir(exist_ok=True)
    t, names, deg = solve.t, list(profile.joint_order), np.degrees
    for arm, _ in ARMS:
        target, solved = solve.target[arm], solve.solved[arm]
        fig, axes = plt.subplots(5, 1, figsize=(14, 13), sharex=True)
        for k, ax in enumerate("xyz"):
            shade(axes[k], solve)
            axes[k].plot(t, target[:, k], lw=1.0, label="target")
            axes[k].plot(t, solved[:, k], lw=0.8, ls="--", label="solved FK(q_cmd)")
            axes[k].set_ylabel(f"{ax} [m]")
        axes[0].legend(fontsize=8, loc="upper right")
        axes[0].set_title(f"{arm} TCP: target vs solved (shaded: S1 blue, S2/S4 grey)")
        err = (solved[:, :3] - target[:, :3]) * 1e3
        shade(axes[3], solve)
        for k, ax in enumerate("xyz"):
            axes[3].plot(t, err[:, k], lw=0.7, label=ax)
        axes[3].plot(t, np.linalg.norm(err, axis=1), lw=0.9, color="k", label="|e|")
        axes[3].set_ylabel("solved - target [mm]")
        axes[3].legend(fontsize=8, loc="upper right", ncol=4)
        shade(axes[4], solve)
        axes[4].plot(t, quat_angle_deg(target[:, 3:], solved[:, 3:]), lw=0.8)
        axes[4].set_ylabel("orientation error [deg]")
        axes[4].set_xlabel("run time [s]")
        fig.tight_layout()
        fig.savefig(out / f"tcp_{arm}.png", dpi=dpi)
        plt.close(fig)

    # One column per joint group (torso, left, right), one row per joint.
    rows = max(len(g.joints) for g in profile.groups)
    cell = {name: (r, c) for c, g in enumerate(profile.groups) for r, name in enumerate(g.joints)}

    def grid(sharex: bool):
        fig, axes = plt.subplots(rows, len(profile.groups), figsize=(8 * len(profile.groups), 2.1 * rows),
                                 sharex=sharex, squeeze=False)
        for c, g in enumerate(profile.groups):
            for r in range(len(g.joints), rows):
                axes[r, c].set_visible(False)
        return fig, axes

    def joint_grid(title: str, file: str, series: np.ndarray, unit: str, lo=None, hi=None, red=None) -> None:
        fig, axes = grid(sharex=True)
        for j, name in enumerate(names):
            ax = axes[cell[name]]
            shade(ax, solve)
            ax.plot(t, series[:, j], lw=0.7)
            if red is not None and red[:, j].any():
                ax.plot(t[red[:, j]], series[red[:, j], j], ".", ms=2, color="r")
            for bound in (lo, hi):
                if bound is not None:
                    ax.axhline(bound[j], color="r", lw=0.8, ls="--")
            ax.set_ylabel(f"{name} [{unit}]", fontsize=8)
        for c, g in enumerate(profile.groups):
            axes[len(g.joints) - 1, c].set_xlabel("run time [s]")
            axes[len(g.joints) - 1, c].tick_params(labelbottom=True)
        fig.suptitle(title)
        fig.tight_layout()
        fig.savefig(out / file, dpi=dpi)
        plt.close(fig)

    q = deg(solve.q)
    joint_grid("joint command q_cmd and URDF limits (red: beyond the limit)", "joints_q.png", q, "deg",
               deg(model.lower), deg(model.upper), (solve.q < model.lower) | (solve.q > model.upper))
    vel = deg(model.vel_limit)
    joint_grid("joint velocity and URDF limit", "joints_vel.png", deg(solve.dq), "deg/s", -vel, vel,
               np.abs(solve.dq) > model.vel_limit)
    acc = deg(np.array(profile.max_acc)) if profile.max_acc else None
    joint_grid("joint acceleration (low passed) and robot.yaml limit", "joints_acc.png", deg(solve.ddq), "deg/s^2",
               None if acc is None else -acc, acc, None if acc is None else np.abs(solve.ddq) > np.radians(acc))

    def dist(title: str, file: str, series: np.ndarray, unit: str, limit) -> None:
        fig, axes = grid(sharex=False)
        for j, name in enumerate(names):
            ax = axes[cell[name]]
            x = np.abs(series[solve.s3, j])
            ax.hist(x, bins=80, color="#7aa6d2")
            for s, color in (("p50", "g"), ("p95", "orange"), ("p99", "r")):
                ax.axvline(np.percentile(x, int(s[1:])), color=color, lw=1.0, label=s)
            if limit is not None:
                ax.axvline(limit[j], color="k", lw=1.0, ls="--", label="limit")
            ax.set_ylabel(name, fontsize=8)
            ax.set_xlabel(f"|{unit}|", fontsize=8)
        axes[0, 0].legend(fontsize=7)
        fig.suptitle(title + " over S3")
        fig.tight_layout()
        fig.savefig(out / file, dpi=dpi)
        plt.close(fig)

    dist("joint velocity distribution", "dist_vel.png", deg(solve.dq), "deg/s", vel)
    dist("joint acceleration distribution (low passed)", "dist_acc.png", deg(solve.ddq), "deg/s^2", acc)


def write_solve_csv(solve: Solve, profile, path: Path) -> None:
    columns, blocks = ["t_s"], []
    for arm, p in ARMS:
        columns += [f"{p}_target_{f}" for f in POSE] + [f"{p}_solved_{f}" for f in POSE]
        blocks += [solve.target[arm], solve.solved[arm]]
    for stem, values in (("q_cmd", solve.q), ("dq", solve.dq), ("ddq", solve.ddq)):
        columns += [f"{g.prefix}_{stem}_{j}" for g in profile.groups for j in range(1, len(g.joints) + 1)]
        blocks.append(values)
    values = np.column_stack([solve.t, *blocks])
    with open(path, "w") as f:
        f.write(",".join(columns[:1] + ["phase"] + columns[1:]) + "\n")
        for k in range(len(values)):
            f.write(f"{values[k, 0]:.6f},{solve.phase[k]}," + ",".join(f"{v:.9g}" for v in values[k, 1:]) + "\n")


def worst(per_run: list[dict]) -> dict:
    """Worst run per metric: the smallest margin, the largest of everything else."""
    out = {}
    for key in per_run[0]:
        values = [m[key] for m in per_run if m.get(key) is not None]
        if not values or isinstance(values[0], str):
            out[key] = values[0] if values else None
            continue
        out[key] = min(values) if key.startswith("margin.") else max(values)
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path, help="output folder of replay --solve, holding run_*/")
    parser.add_argument("--robot", type=Path, help="robot folder, by default robots/<robot of run_info.json>")
    parser.add_argument("--case", type=Path, help="case folder, by default data/<case of run_info.json>")
    parser.add_argument("--no-fig", action="store_true")
    parser.add_argument("--dpi", type=int, default=110)
    args = parser.parse_args()

    runs = sorted(args.session.glob("run_*"), key=lambda p: int(p.name.split("_")[1]))
    if not runs:
        print(f"error: no run_* in {args.session}", file=sys.stderr)
        return 1
    info = json.loads((runs[0] / "run_info.json").read_text())
    robot_dir = args.robot or REPO_ROOT / "robots" / info["robot"]
    case_dir = args.case or REPO_ROOT / "data" / info["case"]
    profile = load_profile(robot_dir)
    cfg = read_yaml(robot_dir / "robot.yaml")
    files = input_files(robot_dir, case_dir)
    controller = read_yaml(files["robot/" + Path(cfg["controller"]).name])
    model = ArmModel(profile)
    lowpass = float(cfg["solve"].get("acc_lowpass_hz", 20.0))

    rep = Report(echo=True)
    rep.add(f"轨迹求解 {info['case']} on {profile.name}: OCS2 MPC IK 实时求解"
            f"{' (全身: 腰 + 双臂一个 MPC)' if profile.torso else ''}, 理想跟随 (q_meas = q_cmd), "
            f"IK {info['reference_mode']} 参考, {len(runs)} 次")
    rep.add(f"统计只算 S3; 加速度为 {lowpass:.0f} Hz 4 阶零相位低通后差分; 实时求解每次结果略有差异"
            f"{', 汇总取各项最差' if len(runs) > 1 else ''}")
    per_run, statuses, prechecks, violations = [], [], [], []
    for run in runs:
        run_info = json.loads((run / "run_info.json").read_text())
        statuses.append(run_info["status"])
        rep.section(f"{run.name}: {run_info['status']} {run_info['reason']}  (S3 {run_info['s3_start_s']:.2f} .. "
                    f"{run_info['s4_start_s']:.2f} s, 循环超时 {run_info['loop_overruns']} 次)")
        solve = Solve(run, profile, model, lowpass)
        if not solve.s3.any():
            rep.add("  没有 S3 数据, 不统计")
            prechecks.append({"pass": False, "reason": f"{run.name} 没有 S3 数据"})
            continue
        result = analyze(solve, profile, model, cfg, controller, rep)
        per_run.append(result["metrics"])
        prechecks.append(result["precheck"])
        violations.append(result["violations"])
        write_solve_csv(solve, profile, run / "solve.csv")
        if not args.no_fig:
            plot(solve, profile, model, run / "plots", args.dpi)

    snapshot = args.session / "config_snapshot"
    snapshot.mkdir(exist_ok=True)
    for name, path in files.items():
        if name.startswith("robot/") and path.is_file():
            shutil.copy2(path, snapshot / Path(name).name)
    failed = [p for p in prechecks if not p["pass"]]
    precheck = failed[0] if failed else max(prechecks, key=lambda p: p["max_joint_vel_deg_s"])
    if len(failed) > 1:
        precheck = {**precheck, "reason": "; ".join(p["reason"] for p in failed)}
    status = "ok" if all(s == "ok" for s in statuses) and per_run else "failed"
    payload = {
        "case": info["case"], "robot": profile.name, "status": status, "runs": len(runs),
        "precheck": precheck, "inputs_sha256": input_hashes(robot_dir, case_dir),
        "violations": violations, "metrics": worst(per_run) if per_run else {},
    }
    (args.session / "metrics.json").write_text(json.dumps(payload, indent=1, ensure_ascii=False) + "\n")
    (args.session / "report.txt").write_text("\n".join(rep.lines) + "\n")
    print(f"\nreport in {args.session}")
    return 0 if status == "ok" else 2


if __name__ == "__main__":
    sys.exit(main())
