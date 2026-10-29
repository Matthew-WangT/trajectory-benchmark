"""L3: end-effector tracking.

error        FK(q_cmd) - FK(q_meas) per axis, cross-checked against /end_*_in_base
attribution  dx = sum_i J_i dq_i, per-joint contribution to the X / Y / Z error, cond(J)
spectrum     PSD of the EE error per axis, joint errors and the reference -> tracking vs excited oscillation
verdict      decision-table row from delay consistency, error/velocity slope and the regression
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..util import rms
from .base import Analyzer, AnalyzerResult, Context, needs_model
from .joint_tracking import SLOPE_VS_DELAY_MISMATCH, JointResult, delay_inconsistent

COND_SINGULAR = 30.0
REF_ENERGY_RATIO = 3.0
AXES = "xyz"


@dataclass
class EeResult:
    ee: dict  # arm -> {"B": FK(q_meas), "A0": FK(q_cmd)} (N,3)
    err: dict  # arm -> (N,3) A0 - B
    err_rms: dict  # arm -> (3,) mm
    contrib: dict = field(default_factory=dict)  # arm -> (N,3,7) J_i dq_i per axis
    contrib_rms: dict = field(default_factory=dict)  # arm -> (3,7)
    share: dict = field(default_factory=dict)  # arm -> (3,7) energy share
    order: dict = field(default_factory=dict)  # arm -> (3,7) joint indices sorted by contribution
    cond: dict = field(default_factory=dict)  # arm -> (N,)
    sigma_min: dict = field(default_factory=dict)  # arm -> (N,), smallest singular value of J_pos
    spec: dict = field(
        default_factory=dict
    )  # arm -> dict(freq, psd_ee (3,F), psd_dq, psd_q, f_s (3,), ref_has_energy (3,))


class EeTrackingAnalyzer(Analyzer):
    name = "ee"
    stage = "L3"
    title = "Step 3  末端层: 误差 / Jacobian 归因 / 频谱"

    def applicable(self, ctx):
        why = needs_model(ctx)
        if why:
            return why
        if "joint" not in ctx.results:
            return "joint analyzer did not run"
        return None

    def run(self, ctx: Context) -> AnalyzerResult:
        ctx.rep.section(self.title)
        res = self._error(ctx)
        self._attribution(ctx, res)
        self._spectrum(ctx, res)
        self._verdict(ctx, res)
        return AnalyzerResult(data=res, figures=[fig_error, fig_attribution, fig_spectrum])

    def _error(self, ctx: Context) -> EeResult:
        rep, d, model, p = ctx.rep, ctx.joints(), ctx.model, ctx.profile
        fk_meas, fk_cmd = model.fk(d.q_meas), model.fk(d.q_cmd)
        ee = {a.name: {"B": fk_meas[a.name], "A0": fk_cmd[a.name]} for a in p.arms}
        ee_meas_ros, ee_cmd_ros = ctx.ee_ros("state"), ctx.ee_ros("command")
        rep.add("FK 自校验 (pinocchio vs /end_state_in_base, /end_command_in_base), RMS per axis [mm]:")
        for arm in p.arms:
            a = arm.name
            if a in ee_meas_ros:
                e_meas = rms(ee[a]["B"] - ee_meas_ros[a], axis=0) * 1e3
                e_cmd = rms(ee[a]["A0"] - ee_cmd_ros[a], axis=0) * 1e3 if a in ee_cmd_ros else None
                flag = "" if np.all(e_meas < 1.0) else "   <<< FK 不一致, URDF/关节顺序/坐标系有误, 后续结论不可信"
                command_text = "-" if e_cmd is None else str(np.round(e_cmd, 3).tolist())
                rep.add(f"  {a:5s} state {np.round(e_meas, 3).tolist()}  command {command_text}{flag}")
                rep.metric(f"ee.{a}.fk_check_mm", float(e_meas.max()))
        err, err_rms = {}, {}
        rep.add("末端误差 FK(q_cmd) - FK(q_meas) RMS [mm] (X, Y, Z) / 单轴 |e| p95 / 模长 |e| p99:")
        for arm in p.arms:
            a = arm.name
            err[a] = ee[a]["A0"] - ee[a]["B"]
            err_rms[a] = rms(err[a], axis=0) * 1e3
            axis_p95 = np.percentile(np.abs(err[a]), 95, axis=0) * 1e3
            norm = np.linalg.norm(err[a], axis=1) * 1e3
            rep.add(
                f"  {a:5s} {np.round(err_rms[a], 2).tolist()}   单轴 p95 {np.round(axis_p95, 2).tolist()}   "
                f"|e| p99 {np.percentile(norm, 99):.1f} mm"
            )
            for k, ax in enumerate(AXES):
                rep.metric(f"ee.{a}.err_rms_mm.{ax}", err_rms[a][k])
                rep.metric(f"ee.{a}.err_p95_mm.{ax}", axis_p95[k])
            rep.metric(f"ee.{a}.err_rms_mm.norm", float(np.linalg.norm(err_rms[a])))
            rep.metric(f"ee.{a}.err_p99_mm", np.percentile(norm, 99))
        return EeResult(ee, err, err_rms)

    def _attribution(self, ctx: Context, res: EeResult) -> None:
        rep, d, model, p, names = ctx.rep, ctx.joints(), ctx.model, ctx.profile, ctx.joint_names
        rep.add("Jacobian 归因 dx = sum_i J_i(q) dq_i, 各轴贡献 RMS [mm] / 能量占比:")
        for arm in p.arms:
            a, idx = arm.name, p.chain_index(arm)
            J = model.jac_pos_series(d.q_meas, arm)  # (N,3,J), the torso joints too
            contrib = J * d.dq[:, idx][:, None, :]
            lin_err = rms(contrib.sum(axis=2) - res.err[a], axis=0) * 1e3
            c_rms = rms(contrib, axis=0)  # (3,7)
            share = c_rms**2 / np.maximum(np.sum(c_rms**2, axis=1, keepdims=True), 1e-18)
            res.contrib[a], res.contrib_rms[a], res.share[a] = contrib, c_rms, share
            res.order[a] = np.argsort(-c_rms, axis=1)
            sv = np.linalg.svd(J, compute_uv=False)
            res.cond[a] = sv[:, 0] / np.maximum(sv[:, -1], 1e-18)
            res.sigma_min[a] = sv[:, -1]
            rep.add(f"  {a} arm (线性化残差 {np.round(lin_err, 2).tolist()} mm):")
            rep.add("    关节        " + "".join(f"{ax.upper():>16s}" for ax in AXES))
            for k in range(len(idx)):
                cells = "".join(f"  {c_rms[x, k] * 1e3:6.2f} / {100 * share[x, k]:4.0f}%" for x in range(3))
                rep.add(f"    {names[idx[k]]:13s}{cells}")
                for x, ax in enumerate(AXES):
                    rep.metric(f"ee.{a}.contrib_share.{ax}.{names[idx[k]]}", share[x, k])
            for x, ax in enumerate(AXES):
                top = [names[idx[k]] for k in res.order[a][x][:2]]
                rep.add(f"    {ax.upper()} 向前两名 {top}, 合计 {100 * share[x, res.order[a][x][:2]].sum():.0f}%")
            c = res.cond[a]
            smin = res.sigma_min[a]
            rep.add(
                f"    cond(J_pos): median {np.median(c):.1f}, max {c.max():.1f}, "
                f">{COND_SINGULAR} 占比 {100 * np.mean(c > COND_SINGULAR):.1f}%; "
                f"sigma_min: median {np.median(smin):.4g}, min {smin.min():.4g}"
            )
            rep.metric(f"ee.{a}.cond_max", c.max())
            rep.metric(f"ee.{a}.cond_singular_frac", np.mean(c > COND_SINGULAR))
            rep.metric(f"ee.{a}.sigma_min_median", np.median(smin))
            rep.metric(f"ee.{a}.sigma_min_min", smin.min())

    def _spectrum(self, ctx: Context, res: EeResult) -> None:
        from scipy.signal import welch

        rep, d, p, names, o = ctx.rep, ctx.joints(), ctx.profile, ctx.joint_names, ctx.opts
        nper = min(1024, len(d.t))
        psd_dq, psd_q, f = [], [], None
        for i in range(len(names)):
            f, pw = welch(d.dq[:, i] - d.dq[:, i].mean(), fs=d.fs, nperseg=nper)
            psd_dq.append(pw)
            psd_q.append(welch(d.q_cmd[:, i] - d.q_cmd[:, i].mean(), fs=d.fs, nperseg=nper)[1])
        psd_dq, psd_q = np.array(psd_dq), np.array(psd_q)
        m_band = (f >= o.band_hz[0]) & (f <= o.band_hz[1])
        rep.add(f"频域对照: Welch nperseg {nper} (分辨率 {d.fs / nper:.3f} Hz), 搜索频段 {o.band_hz} Hz")
        for arm in p.arms:
            a, idx = arm.name, p.chain_index(arm)
            psd_ee = np.zeros((3, len(f)))
            f_s, ref_energy = np.zeros(3), np.zeros(3, dtype=bool)
            for x, ax in enumerate(AXES):
                e = res.err[a][:, x]
                psd_ee[x] = welch(e - e.mean(), fs=d.fs, nperseg=nper)[1]
                k = np.flatnonzero(m_band)[np.argmax(psd_ee[x][m_band])]
                f_s[x] = f[k]
                win = np.abs(f - f_s[x]) <= 0.2
                neigh = (np.abs(f - f_s[x]) > 0.2) & (np.abs(f - f_s[x]) <= 1.0)
                ratios = np.array([psd_q[i][win].max() / max(np.median(psd_q[i][neigh]), 1e-30) for i in idx])
                ref_energy[x] = bool(np.any(ratios > REF_ENERGY_RATIO))
                rep.add(
                    f"  {a} arm {ax.upper()}: 误差主频 f_s = {f_s[x]:.2f} Hz, 参考 q_cmd 在该处"
                    f"{'有' if ref_energy[x] else '无'}显著能量 -> "
                    f"{'跟踪问题 (滞后/衰减)' if ref_energy[x] else '激发出来的振荡 (欠阻尼/摩擦极限环/共振)'}"
                )
                rep.metric(f"ee.{a}.shake_freq_hz.{ax}", f_s[x])
                rep.metric(f"ee.{a}.ref_has_energy.{ax}", float(ref_energy[x]))
            res.spec[a] = {
                "freq": f,
                "psd_ee": psd_ee,
                "psd_dq": psd_dq,
                "psd_q": psd_q,
                "f_s": f_s,
                "ref_has_energy": ref_energy,
            }

    def _verdict(self, ctx: Context, res: EeResult) -> None:
        rep, p, names = ctx.rep, ctx.profile, ctx.joint_names
        jr: JointResult = ctx.results["joint"].data
        rep.add("判别表落点:")
        for arm in p.arms:
            a, idx = arm.name, p.chain_index(arm)
            tx = jr.delay.tau_xcorr[idx] * 1e3
            inconsistent = delay_inconsistent(tx)
            worst = int(np.argmax(res.err_rms[a]))
            top = [idx[k] for k in res.order[a][worst][:2]]
            rep.add(
                f"  [{a} arm] 误差最大的轴 {AXES[worst].upper()} ({res.err_rms[a][worst]:.1f} mm), "
                f"贡献最大的关节 {[names[i] for i in top]}; "
                f"tau 均值 {tx.mean():.0f} ms, 极差 {tx.max() - tx.min():.0f} ms"
            )

            def delay_like(i):
                x = jr.delay.tau_xcorr[i] * 1e3
                mismatch = abs(jr.slope_ms[i] - x) / max(abs(x), 1e-3)
                return mismatch <= SLOPE_VS_DELAY_MISMATCH and jr.reg.dominant[i] == "qd"

            if all(delay_like(i) for i in top) and not inconsistent:
                rep.add(f"    -> 一致延迟主导: 误差≈tau*qd 且各关节 tau 一致. 对策: 统一提前量 {tx.mean():.0f} ms")
            elif all(delay_like(i) for i in top):
                rep.add("    -> 延迟不一致: 误差≈tau*qd 但各关节 tau 离散. 对策: 逐关节提前量或延迟对齐")
            else:
                rep.add("    -> 非延迟主导, 关节归因:")
                for i in top:
                    pr, tag = jr.reg.partial_r2[i], jr.reg.dominant[i]
                    if tag == "g(q)":
                        rep.add(f"       {names[i]}: 重力主导 (偏R2 g={pr[1]:.2f}) -> 重力前馈")
                    elif tag == "qd":
                        rep.add(
                            f"       {names[i]}: 速度相关但斜率与互相关延迟不符 (偏R2 qd={pr[0]:.2f}, "
                            f"slope {jr.slope_ms[i]:.0f} ms vs xcorr {jr.delay.tau_xcorr[i] * 1e3:.0f} ms) "
                            "-> 阻尼/带宽差异, 增益整定或延迟对齐"
                        )
                    elif tag == "sign(qd)":
                        rep.add(f"       {names[i]}: 摩擦 stick-slip (偏R2 sign={pr[3]:.2f}) -> 摩擦前馈")
                    elif tag == "qdd":
                        rep.add(f"       {names[i]}: 惯性项主导 (偏R2 qdd={pr[2]:.2f}) -> 惯量前馈")
                    else:
                        rep.add(
                            f"       {names[i]}: {tag}, R2={jr.reg.r2[i]:.2f}, "
                            f"残差主峰 {jr.reg.resid_peak_hz[i]:.2f} Hz -> 看残差谱 (间隙/柔性/通信抖动)"
                        )
            if np.mean(res.cond[a] > COND_SINGULAR) > 0.05:
                rep.add(
                    f"    ! cond(J) > {COND_SINGULAR} 的时段占 {100 * np.mean(res.cond[a] > COND_SINGULAR):.0f}%, "
                    "优先考虑改 IK / 任务姿态"
                )
            s = res.spec[a]
            excited = [f"{AXES[x].upper()}@{s['f_s'][x]:.1f}Hz" for x in range(3) if not s["ref_has_energy"][x]]
            if excited:
                rep.add(f"    ! {excited} 处参考无能量: 该频率的抖动是激发出来的, 加提前量只会更糟, 应降增益或加阻尼")


# ------------------------------------------------------------------------- figures


def fig_error(ctx: Context) -> None:
    plt = ctx.plt()
    res: EeResult = ctx.results["ee"].data
    t, arms = ctx.joints().t, ctx.profile.arms
    fig, axes = plt.subplots(3 * len(arms), 2, figsize=(17, 3.0 * 3 * len(arms)), squeeze=False)
    for r, arm in enumerate(arms):
        a = arm.name
        for x, axis in enumerate("XYZ"):
            ax = axes[3 * r + x, 0]
            ax.plot(t, res.ee[a]["B"][:, x] * 1e3, "k-", lw=1, label="FK(q_meas)")
            ax.plot(t, res.ee[a]["A0"][:, x] * 1e3, "c-", lw=1, label="FK(q_cmd)")
            ax.set_ylabel(f"{a} EE {axis} [mm]")
            if x == 0:
                ax.set_title(f"{a} arm: end-effector trajectory")
                ax.legend(ncol=2, fontsize=8)
            ax = axes[3 * r + x, 1]
            ax.plot(t, res.err[a][:, x] * 1e3, "r-", lw=0.9)
            ax.set_ylabel(f"error {axis} [mm]")
            ax.set_title(f"FK(q_cmd) - FK(q_meas), RMS {res.err_rms[a][x]:.2f} mm", fontsize=9)
    for ax in axes[-1]:
        ax.set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(ctx.out / "fig_ee_error.png")
    plt.close(fig)


def fig_attribution(ctx: Context) -> None:
    plt = ctx.plt()
    res: EeResult = ctx.results["ee"].data
    t, arms, names = ctx.joints().t, ctx.profile.arms, ctx.joint_names
    fig, axes = plt.subplots(4, len(arms), figsize=(9 * len(arms), 14), squeeze=False)
    for c, arm in enumerate(arms):
        a, idx = arm.name, ctx.profile.chain_index(arm)
        labels = [names[i] for i in idx]
        x = np.arange(len(idx))
        for row, axis in enumerate("XYZ"):
            ax = axes[row, c]
            ax.bar(x, res.contrib_rms[a][row] * 1e3, 0.6)
            ax.axhline(res.err_rms[a][row], color="k", ls=":", label=f"EE {axis} error RMS")
            ax.set_xticks(x)
            ax.set_xticklabels(labels, rotation=30)
            ax.set_ylabel(f"RMS of J_i*dq_i, {axis} [mm]")
            ax.set_title(f"{a} arm: joint contribution to end-effector {axis} error")
            ax.legend(fontsize=7)
        worst = int(np.argmax(res.err_rms[a]))
        ax = axes[3, c]
        ax.plot(t, res.err[a][:, worst] * 1e3, "k-", lw=1.2, label=f"EE {'XYZ'[worst]} error")
        for k in res.order[a][worst][:3]:
            ax.plot(t, res.contrib[a][:, worst, k] * 1e3, lw=0.9, label=labels[k])
        ax.set_ylabel(f"{'XYZ'[worst]} [mm]")
        ax.set_xlabel("t [s]")
        ax.legend(fontsize=7, loc="upper left")
        ax2 = ax.twinx()
        ax2.plot(t, res.cond[a], color="tab:purple", lw=0.7, alpha=0.6)
        ax2.axhline(COND_SINGULAR, color="tab:purple", ls=":", lw=0.8)
        ax2.set_ylabel("cond(J_pos) (purple)")
        ax2.grid(False)
    fig.tight_layout()
    fig.savefig(ctx.out / "fig_ee_jacobian.png")
    plt.close(fig)


def fig_spectrum(ctx: Context) -> None:
    plt = ctx.plt()
    res: EeResult = ctx.results["ee"].data
    arms, names, band = ctx.profile.arms, ctx.joint_names, ctx.opts.band_hz
    colors = ("tab:red", "tab:green", "tab:blue")
    fig, axes = plt.subplots(3, len(arms), figsize=(9 * len(arms), 11), squeeze=False)
    for c, arm in enumerate(arms):
        a, idx = arm.name, ctx.profile.chain_index(arm)
        s = res.spec[a]
        f = s["freq"]
        m = (f > 0) & (f <= max(10.0, band[1]))
        ax = axes[0, c]
        for x, axis in enumerate("XYZ"):
            ax.semilogy(f[m], s["psd_ee"][x][m], color=colors[x], lw=1, label=f"{axis}  f_s={s['f_s'][x]:.2f} Hz")
            ax.axvline(s["f_s"][x], color=colors[x], ls=":", lw=0.8)
        ax.set_title(f"{a} arm: PSD of end-effector error")
        ax.set_ylabel("m^2/Hz")
        ax.legend(fontsize=8)
        for row, key, title in (
            (1, "psd_dq", "PSD of joint tracking error dq_i"),
            (2, "psd_q", "PSD of reference q_cmd,i"),
        ):
            ax = axes[row, c]
            for i in idx:
                ax.semilogy(f[m], s[key][i][m], lw=0.9, label=names[i])
            for x in range(3):
                ax.axvline(s["f_s"][x], color=colors[x], ls=":", lw=0.8)
            ax.set_title(title)
            ax.set_ylabel("rad^2/Hz")
            ax.legend(fontsize=7, ncol=2)
        axes[2, c].set_xlabel("f [Hz]")
    fig.tight_layout()
    fig.savefig(ctx.out / "fig_ee_spectrum.png")
    plt.close(fig)
