#!/usr/bin/env python3
"""Replay a case on the Marvin, evaluate it and print the results, in one terminal session.

Run it through ./benchmark.sh, which builds the image and the code and starts this script in the container.
Options not given on the command line are picked with arrow-key menus: the mode (real robot / dry run), the case
under data/, the config under config/ and what to run (the full case or one of the step-by-step bring-up stages).
Then, without further input:
  1. a real run without a dry run of the case runs the dry run first (it checks the joint velocities)
  2. build/bin/replay runs; its full log goes to <session>/replay.log, the key lines to the terminal
  3. for each run: scripts/make_delivery_csv.py and scripts/evaluate.py
  4. scripts/summarize.py over the runs, and a compact table of the headline metrics

Ctrl-C during the replay aborts it the way the replay does on any violation: the command stops and the servos are
disabled, the brakes hold the arms. The records are kept and evaluated when a run completed.

  ./benchmark.sh                                       # all interactive
  ./benchmark.sh --mode dry --case case02_fold_clothes_spd50 --runs 3 -y
"""

from __future__ import annotations

import argparse
import json
import os
import re
import signal
import subprocess
import sys
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
REPLAY = REPO / "build/bin/replay"
STAGES = {
    "full": "完整用例",
    "connect": "只连接并检查（robot.ini、反馈、初始偏差）",
    "S0": "只跑 S0：关节插值到 q_pd 并保持",
    "S2": "只跑 S0–S2：慢速移到起始位姿并静止，不跑轨迹",
}
# Replay log lines worth showing on the terminal.
KEY_LINES = re.compile(r"Case |Connected|robot\.ini|Measured joints|Entered the joint|Run run_|Run \d|Abort|peaks|"
                       r"Records in|Error|error|failed|Refusing|Missing")

BOLD, CYAN, GREEN, RED, YELLOW, RESET = "\x1b[1m", "\x1b[36m", "\x1b[32m", "\x1b[31m", "\x1b[33m", "\x1b[0m"


def info(text: str) -> None:
    print(f"{GREEN}[benchmark]{RESET} {text}", flush=True)


def warn(text: str) -> None:
    print(f"{YELLOW}[benchmark] {text}{RESET}", flush=True)


def select_menu(title: str, options: list[str], default: int = 0) -> int:
    """Arrow-key menu on the terminal, from mindon_robotics scripts/bag_analysis; returns the picked index."""
    if not sys.stdin.isatty():
        sys.exit(f"{title}: stdin is not a tty, pass the value on the command line instead")
    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    idx = default
    print(f"{BOLD}{title}{RESET} (↑/↓ or j/k, Enter to confirm, q to abort)", flush=True)

    def render(first: bool) -> None:
        if not first:
            sys.stdout.write(f"\x1b[{len(options)}A")
        for i, option in enumerate(options):
            line = f"  ❯ {option}" if i == idx else f"    {option}"
            if i == idx:
                line = f"{CYAN}{line}{RESET}"
            sys.stdout.write("\x1b[2K" + line + "\n")
        sys.stdout.flush()

    try:
        tty.setcbreak(fd)
        render(first=True)
        while True:
            ch = os.read(fd, 1)
            if ch == b"\x1b":
                seq = os.read(fd, 2)
                if seq == b"[A":
                    idx = (idx - 1) % len(options)
                elif seq == b"[B":
                    idx = (idx + 1) % len(options)
            elif ch == b"k":
                idx = (idx - 1) % len(options)
            elif ch == b"j":
                idx = (idx + 1) % len(options)
            elif ch in (b"\r", b"\n"):
                return idx
            elif ch == b"q":
                raise KeyboardInterrupt
            render(first=False)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


def list_cases() -> list[Path]:
    return sorted(p for p in (REPO / "data").iterdir() if (p / "eef_command.csv").is_file())


def list_configs() -> list[Path]:
    return sorted(p for p in (REPO / "config").glob("*.yaml"))


def case_label(case: Path) -> str:
    meta = json.loads((case / "meta.json").read_text())
    return f"{case.name}  ({meta['duration_s']:.1f} s)"


def config_summary(path: Path) -> list[str]:
    """Key settings of the replay config, as the replay parses them."""
    out = subprocess.run([str(REPLAY), "--config", str(path), "--show-config"], cwd=REPO, capture_output=True,
                         text=True)
    if out.returncode != 0:
        sys.exit(f"cannot read {path}:\n{out.stderr}")
    return out.stdout.splitlines()


@dataclass
class Plan:
    dry: bool
    case: Path
    config: Path
    stage: str
    runs: int


def resolve(args: argparse.Namespace) -> Plan:
    if args.mode is None:
        args.mode = ("real", "dry")[select_menu("模式", ["真机 (real robot)", "dry-run（不连机器人）"])]
        print()
    cases = list_cases()
    if args.case is None:
        case = cases[select_menu("用例", [case_label(c) for c in cases])]
        print()
    else:
        case = Path(args.case) if Path(args.case).is_dir() else REPO / "data" / args.case
        if not (case / "eef_command.csv").is_file():
            sys.exit(f"unknown case {args.case}; known: {[c.name for c in cases]}")
    configs = list_configs()
    if args.config is None:
        default = next((i for i, c in enumerate(configs) if c.name == "marvin.yaml"), 0)
        config = configs[select_menu("配置", [c.name for c in configs], default)] if len(configs) > 1 else configs[0]
        if len(configs) > 1:
            print()
    else:
        config = Path(args.config) if Path(args.config).is_file() else REPO / "config" / args.config
        if not config.is_file():
            sys.exit(f"unknown config {args.config}; known: {[c.name for c in configs]}")
    stage, runs = args.stage, args.runs
    if stage is None and runs is not None:
        stage = "full"
    if stage is None:
        options = [("full", 3), ("full", 1), ("connect", 1), ("S0", 1), ("S2", 1)]
        labels = [f"{STAGES[s]}，{r} 次" if s == "full" else STAGES[s] for s, r in options]
        stage, picked_runs = options[select_menu("运行", labels)]
        runs = runs or picked_runs
        print()
    return Plan(args.mode == "dry", case, config, stage, runs or (3 if stage == "full" else 1))


def replay_command(plan: Plan, out: Path, dry: bool | None = None, stage: str | None = None,
                   runs: int | None = None) -> list[str]:
    dry = plan.dry if dry is None else dry
    stage = plan.stage if stage is None else stage
    cmd = [str(REPLAY), "--config", str(plan.config), "--case", str(plan.case), "--out", str(out),
           "--runs", str(plan.runs if runs is None else runs)]
    if dry:
        cmd.append("--dry-run")
    if stage != "full":
        # The bring-up stages stop before the trajectory, the joint velocity check of the dry run is about it.
        cmd += ["--stop-after", stage, "--skip-vel-check"]
    return cmd


def run_replay(cmd: list[str], log: Path) -> int:
    """Runs the replay, its full log to `log` and the key lines to the terminal. Ctrl-C is left to the replay."""
    info("$ " + " ".join(os.path.relpath(c, REPO) if c.startswith(str(REPO)) else c for c in cmd))
    log.parent.mkdir(parents=True, exist_ok=True)
    with open(log, "a") as f:
        proc = subprocess.Popen(cmd, cwd=REPO, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                bufsize=1)
        # The terminal's Ctrl-C reaches the replay too (same process group); it aborts, disables the arms and
        # exits by itself, so this script only keeps reading.
        previous = signal.signal(signal.SIGINT, lambda *_: warn("Ctrl-C: the replay aborts and disables the arms"))
        try:
            for line in proc.stdout:
                f.write(line)
                if KEY_LINES.search(line) and "####" not in line:
                    text = re.sub(r"^[IWEF]\d{4} [\d:.]+\s+\d+ \S+:\d+\] ", "", line.rstrip())
                    color = RED if line.startswith("E") or "Abort" in line else ""
                    print(f"  {color}{text}{RESET if color else ''}", flush=True)
            return proc.wait()
        finally:
            signal.signal(signal.SIGINT, previous)


def run_step(cmd: list[str], log: Path) -> bool:
    with open(log, "a") as f:
        f.write("$ " + " ".join(cmd) + "\n")
        f.flush()
        return subprocess.run(cmd, cwd=REPO, stdout=f, stderr=subprocess.STDOUT).returncode == 0


def evaluate(session: Path, log: Path) -> list[Path]:
    """Delivery CSV and evaluation of each completed run; returns the evaluated runs."""
    done = []
    for run in sorted(session.glob("run_*")):
        status = json.loads((run / "run_info.json").read_text())
        if status["status"] != "ok":
            warn(f"{run.name}: {status['status']} ({status['reason']}), not evaluated")
            continue
        ok = run_step([sys.executable, "scripts/make_delivery_csv.py", str(run)], log) and run_step(
            [sys.executable, "scripts/evaluate.py", str(run)], log)
        if ok:
            info(f"{run.name}: evaluated, report in {os.path.relpath(run / 'report', REPO)}")
            done.append(run)
        else:
            warn(f"{run.name}: evaluation failed, see {os.path.relpath(log, REPO)}")
    if done:
        run_step([sys.executable, "scripts/summarize.py", str(session)], log)
    return done


TABLE = [
    ("关节误差 RMS 均值 [deg]", "joint_error.{arm}.rms_deg_mean"),
    ("关节误差 RMS 最差关节 [deg]", "joint_error.{arm}.rms_deg_max"),
    ("关节误差 max [deg]", "joint_error.{arm}.max_deg"),
    ("关节时延 p50 [ms]", "joint.{arm}.delay_p50_ms"),
    ("关节时延 p95 [ms]", "joint.{arm}.delay_p95_ms"),
    ("TCP 位置误差 RMS [mm]", "tcp.{arm}.pos_rms_mm.norm"),
    ("TCP 位置误差 max [mm]", "tcp.{arm}.pos_max_mm.norm"),
    ("TCP 姿态误差 RMS [deg]", "tcp.{arm}.rot_rms_deg"),
    ("TCP 等效时延 [ms]", "tcp.{arm}.equiv_delay_ms"),
]


def pad(text: str, width: int) -> str:
    """Left-justifies to a terminal width, the CJK characters taking two columns."""
    import unicodedata

    used = sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in text)
    return text + " " * max(width - used, 1)


def print_table(runs: list[Path]) -> None:
    metrics = [json.loads((r / "report/metrics.json").read_text())["metrics"] for r in runs]

    def cell(key: str) -> str:
        values = [m[key] for m in metrics if m.get(key) is not None]
        if not values:
            return "-"
        mean = sum(values) / len(values)
        return f"{mean:7.2f}" if len(values) == 1 else f"{mean:7.2f} [{min(values):.2f}, {max(values):.2f}]"

    print(f"\n{BOLD}结果 ({len(runs)} 次{'，均值 [min, max]' if len(runs) > 1 else ''}){RESET}")
    print("  " + pad("", 30) + pad("左臂", 28) + "右臂")
    for title, key in TABLE:
        print("  " + pad(title, 30) + pad(cell(key.format(arm="left")), 28) + cell(key.format(arm="right")))
    health = [("IK hold 占比", "health.hold_frac"), ("下发→控制器收到 [ms]", "health.send_to_cmd_rx_lag_ms"),
              ("下发→实测 [ms]", "health.send_to_meas_lag_ms"), ("控制循环超时次数", "health.loop_overruns")]
    print("  " + "   ".join(f"{t} {cell(k).strip()}" for t, k in health))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("real", "dry"))
    parser.add_argument("--case", help="case folder name under data/ (or a path)")
    parser.add_argument("--config", help="config file name under config/ (or a path)")
    parser.add_argument("--stage", choices=tuple(STAGES), help="full, or stop after a bring-up stage")
    parser.add_argument("--runs", type=int, help="number of runs of the case")
    parser.add_argument("-y", "--yes", action="store_true", help="do not ask for confirmation before a real run")
    args = parser.parse_args()
    if not REPLAY.is_file():
        sys.exit(f"{REPLAY} missing, start this through ./benchmark.sh which builds it")

    try:
        plan = resolve(args)
    except KeyboardInterrupt:
        print("aborted")
        return 1
    mode = "dry" if plan.dry else "real"
    session = REPO / "runs" / plan.case.name / f"{mode}_{time.strftime('%Y%m%d_%H%M%S')}"
    log = session / "replay.log"
    print(f"{BOLD}计划{RESET}: {mode}  {plan.case.name}  {plan.config.name}  {STAGES[plan.stage]}"
          f"{f'，{plan.runs} 次' if plan.stage == 'full' else ''}")
    for line in config_summary(plan.config):
        print(f"    {line}")
    print(f"    输出                 {os.path.relpath(session, REPO)}")

    if not plan.dry:
        vel_check = REPO / "runs" / plan.case.name / "dry" / "vel_check.json"
        if plan.stage == "full" and not vel_check.is_file():
            info("这个用例还没有 dry-run 结果，先跑一次 dry-run（检查关节速度）")
            if run_replay(replay_command(plan, session.with_name(f"dry_{session.name[5:]}"), dry=True, stage="full",
                                         runs=1), log) != 0:
                warn("dry-run 失败，停止")
                return 1
        if not args.yes:
            try:
                input(f"{BOLD}{RED}真机即将运动，确认周围安全后按 Enter 开始，Ctrl-C 取消 ...{RESET}")
            except KeyboardInterrupt:
                print("\naborted")
                return 1

    code = run_replay(replay_command(plan, session), log)
    if code != 0:
        warn(f"replay 退出码 {code}，完整日志 {os.path.relpath(log, REPO)}")
    if plan.stage != "full":
        for run in sorted(session.glob("run_*")):
            status = json.loads((run / "run_info.json").read_text())
            delay = status.get("max_command_delay_s", -1)
            info(f"{run.name}: {status['status']} {status['reason']}  命令延迟最大 "
                 f"{'-' if delay < 0 else f'{delay * 1e3:.1f} ms'}")
        return code
    runs = evaluate(session, log)
    if runs:
        print_table(runs)
        info(f"汇总 {os.path.relpath(session / 'summary.txt', REPO)}，交付 CSV 在各 run_*/delivery.csv")
    return code if runs else (code or 1)


if __name__ == "__main__":
    sys.exit(main())
