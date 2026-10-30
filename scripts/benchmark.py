#!/usr/bin/env python3
"""Replay a case on the Marvin, evaluate it and print the results, in one terminal session; or solve a case on a
robot of config/robots/ with the IK controller alone.

Run it through ./benchmark.sh, which builds the image and the code and starts this script in the container.
Options not given on the command line are picked with arrow-key menus: the mode (real robot / dry run / trajectory
solve), the case under data/, then the config under config/ and what to run (the full case or one of the
step-by-step bring-up stages), or for the solve the robot under config/robots/. Then, without further input:
  solve:
  1. build/bin/replay --solve runs the IK controller in real time against an ideal robot
  2. scripts/analyze_solve.py: solve error, joint limit margins, velocity / acceleration distributions, a table
  3. scripts/view_solve.py: 3D playback of the robot and the target / solved TCP paths in the browser (meshcat)
  real / dry run:
  1. a full real run checks the latest trajectory solve of the case whose inputs match the current files (joint
     velocity peak, joint limits), and runs the solve first when there is none
  2. build/bin/replay runs; its full log goes to <session>/replay.log, the key lines to the terminal
  3. for each run: scripts/make_delivery_csv.py and scripts/evaluate.py
  4. scripts/summarize.py over the runs, and a compact table of the headline metrics

Ctrl-C during the replay aborts it the way the replay does on any violation: the command stops and the servos are
disabled, the brakes hold the arms. The records are kept and evaluated when a run completed.

  ./benchmark.sh                                       # all interactive
  ./benchmark.sh --mode dry --case case02_fold_clothes_spd50 --runs 3 -y
  ./benchmark.sh --mode solve --case case02_fold_clothes_spd50 --robot marvin
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
sys.path.insert(0, str(REPO / "python"))

from trackeval.robots import read_yaml  # noqa: E402
from trackeval.solve_inputs import input_hashes  # noqa: E402

STAGES = {
    "full": "完整用例",
    "connect": "只连接并检查（robot.ini、反馈、初始偏差）",
    "S0": "只跑 S0：关节插值到 q_pd 并保持",
    "S2": "只跑 S0–S2：慢速移到起始位姿并静止，不跑轨迹",
}
# Replay log lines worth showing on the terminal.
KEY_LINES = re.compile(r"Case |Connected|robot\.ini|Measured joints|Entered the joint|Run run_|Run \d|Run:|Abort|peaks|"
                       r"Records in|Error|error|failed|Refusing|Missing|Sidecar|Waiting up to|disabled")

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


def list_robots() -> list[Path]:
    return sorted(p.parent for p in (REPO / "config" / "robots").glob("*/robot.yaml"))


# Exit code that asks ./benchmark.sh to run the same plan again in the s1 image.
S1_RERUN_EXIT = 75


def is_s1(config: Path) -> bool:
    """An S1 replay config: the robot is driven by the SDK sidecar scripts/s1_bridge.py."""
    return "bridge" in read_yaml(config)


def robot_of(config: Path) -> Path:
    """Robot folder named by a replay config."""
    return (config.parent / read_yaml(config)["robot_dir"]).resolve()


def case_label(case: Path) -> str:
    meta = json.loads((case / "meta.json").read_text())
    return f"{case.name}  ({meta['duration_s']:.1f} s)"


def config_summary(path: Path) -> list[str]:
    """Key settings of the replay config, as the replay parses them."""
    if is_s1(path):
        cfg = read_yaml(path)
        disabled = [k for k, v in cfg.get("safety", {}).items() if isinstance(v, dict) and not v.get("enable", True)]
        return [f"S1 SDK sidecar         127.0.0.1:{cfg['bridge']['port']}, {cfg['bridge']['freq_hz']:.0f} Hz, "
                f"S0 set_joints_position / S1.. set_cartesian_pose (direct, use_wbc)",
                f"腰                     {cfg.get('torso_command', 'free')}",
                f"安全检查               {'全部启用' if not disabled else '禁用 ' + ', '.join(disabled)}"]
    out = subprocess.run([str(REPLAY), "--config", str(path), "--show-config"], cwd=REPO, capture_output=True,
                         text=True)
    if out.returncode != 0:
        sys.exit(f"cannot read {path}:\n{out.stderr}")
    return out.stdout.splitlines()


@dataclass
class Plan:
    mode: str  # real / dry / solve
    case: Path
    config: Path | None  # replay config, None for the solve
    robot: Path  # robot folder
    stage: str
    runs: int

    @property
    def dry(self) -> bool:
        return self.mode == "dry"


def resolve(args: argparse.Namespace) -> Plan:
    if args.mode is None:
        args.mode = ("real", "dry", "solve")[select_menu(
            "模式", ["真机 (real robot)", "dry-run（不连机器人）", "轨迹求解 (solve)：只跑 IK，分析误差、余量与速度/加速度分布"])]
        print()
    cases = list_cases()
    if args.case is None:
        case = cases[select_menu("用例", [case_label(c) for c in cases])]
        print()
    else:
        case = Path(args.case) if Path(args.case).is_dir() else REPO / "data" / args.case
        if not (case / "eef_command.csv").is_file():
            sys.exit(f"unknown case {args.case}; known: {[c.name for c in cases]}")
    if args.mode == "solve":
        robots = list_robots()
        if args.robot is None:
            robot = robots[select_menu("机器人", [r.name for r in robots])] if len(robots) > 1 else robots[0]
            if len(robots) > 1:
                print()
        else:
            robot = Path(args.robot) if (Path(args.robot) / "robot.yaml").is_file() else REPO / "config" / "robots" / args.robot
            if not (robot / "robot.yaml").is_file():
                sys.exit(f"unknown robot {args.robot}; known: {[r.name for r in robots]}")
        return Plan("solve", case, None, robot.resolve(), "full", args.runs or 1)
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
    return Plan(args.mode, case, config, robot_of(config), stage, runs or (3 if stage == "full" else 1))


def replay_command(plan: Plan, out: Path, solve_result: Path | None = None) -> list[str]:
    cmd = [str(REPLAY), "--config", str(plan.config), "--case", str(plan.case), "--out", str(out),
           "--runs", str(plan.runs)]
    if plan.dry and not is_s1(plan.config):
        cmd.append("--dry-run")
    elif plan.dry:
        # The S1 dry run is the real replay against the fake sidecar (start_fake_bridge), the precheck of the
        # trajectory solve is for the robot.
        cmd.append("--skip-vel-check")
    if plan.stage != "full":
        # The bring-up stages stop before the trajectory, the precheck of the trajectory solve is about it.
        cmd += ["--stop-after", plan.stage]
        if "--skip-vel-check" not in cmd:
            cmd.append("--skip-vel-check")
    elif solve_result is not None:
        cmd += ["--solve-result", str(solve_result)]
    return cmd


def start_fake_bridge(config: Path, log: Path) -> subprocess.Popen:
    """The S1 sidecar with its kinematic stand-in of the robot, in this container, for the S1 dry run."""
    log.parent.mkdir(parents=True, exist_ok=True)
    info(f"dry-run: 启动 S1 sidecar 的仿真机器人 (scripts/s1_bridge.py --fake)，日志 {os.path.relpath(log, REPO)}")
    return subprocess.Popen([sys.executable, "scripts/s1_bridge.py", "--config", str(config), "--fake"], cwd=REPO,
                            stdout=open(log, "w"), stderr=subprocess.STDOUT)


def start_sdk_bridge(config: Path, log: Path) -> subprocess.Popen:
    """The S1 sidecar on the real robot through the Astribot SDK, in this container (the s1 image)."""
    log.parent.mkdir(parents=True, exist_ok=True)
    info(f"S1 真机: 启动 SDK sidecar (scripts/s1_bridge.py)，日志 {os.path.relpath(log, REPO)}")
    return subprocess.Popen(["docker/s1_sdk_env.sh", "python3", "scripts/s1_bridge.py", "--config", str(config)],
                            cwd=REPO, stdout=open(log, "w"), stderr=subprocess.STDOUT)


def solve_session(case: Path, robot: Path) -> Path:
    return REPO / "runs" / case.name / f"solve_{robot.name}_{time.strftime('%Y%m%d_%H%M%S')}"


def run_solve(case: Path, robot: Path, runs: int, session: Path, log: Path) -> Path | None:
    """Trajectory solve of the case and its analysis; returns the metrics.json, None when either failed."""
    cmd = [str(REPLAY), "--solve", "--robot", str(robot), "--case", str(case), "--out", str(session),
           "--runs", str(runs)]
    code = run_replay(cmd, log)
    if code != 0:
        warn(f"solve 退出码 {code}，完整日志 {os.path.relpath(log, REPO)}")
        if not list(session.glob("run_*/raw_cmd.csv")):
            return None
    info("分析求解结果（误差、余量、速度/加速度分布、曲线）")
    if not run_step([sys.executable, "scripts/analyze_solve.py", str(session), "--robot", str(robot), "--case",
                     str(case)], log):
        warn(f"分析失败，见 {os.path.relpath(log, REPO)}")
    metrics = session / "metrics.json"
    return metrics if metrics.is_file() else None


def find_solve(case: Path, robot: Path) -> Path | None:
    """metrics.json of the latest completed solve of the case on the robot whose inputs match the current files."""
    current = input_hashes(robot, case)
    for session in sorted((REPO / "runs" / case.name).glob(f"solve_{robot.name}_*"), reverse=True):
        metrics = session / "metrics.json"
        if not metrics.is_file():
            continue
        result = json.loads(metrics.read_text())
        if result.get("status") == "ok" and result.get("inputs_sha256") == current:
            return metrics
    return None


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
            return f"{'-':>7s}"
        mean = sum(values) / len(values)
        return f"{mean:7.2f}" if len(values) == 1 else f"{mean:7.2f} [{min(values):.2f}, {max(values):.2f}]"

    print(f"\n{BOLD}结果 ({len(runs)} 次{'，均值 [min, max]' if len(runs) > 1 else ''}){RESET}")
    # A robot with a torso gets a torso column, joint numbers only.
    groups = [("torso", "腰")] if any("joint_error.torso.rms_deg_mean" in m for m in metrics) else []
    groups += [("left", "左臂"), ("right", "右臂")]
    print("  " + pad("", 30) + "".join(pad(label, 28) for _, label in groups))
    for title, key in TABLE:
        print("  " + pad(title, 30) + "".join(pad(cell(key.format(arm=g)), 28) for g, _ in groups))
    infos = [json.loads((r / "run_info.json").read_text()) for r in runs]
    if all(i.get("backend") == "s1_sdk" for i in infos):
        # The S1 runs: the link to the SDK sidecar instead of the Marvin replay chain.
        bridge = [i["bridge"] for i in infos]
        print(f"  sidecar 未应答周期 {sum(b['total_miss'] for b in bridge)}   最长连续 "
              f"{max(b['max_consecutive_miss'] for b in bridge)}   SDK 读取 p50/max [ms] "
              f"{max(b['read_ms_p50'] for b in bridge):.2f}/{max(b['read_ms_max'] for b in bridge):.2f}   "
              f"SDK 命令 p50/max [ms] {max(b['command_ms_p50'] for b in bridge):.2f}/"
              f"{max(b['command_ms_max'] for b in bridge):.2f}")
        disabled = sorted({c for i in infos for c in i.get("safety_disabled", [])})
        if disabled:
            warn(f"安全检查被禁用: {', '.join(disabled)}")
        return
    health = [("IK hold 占比", "health.hold_frac"), ("下发→控制器收到 [ms]", "health.send_to_cmd_rx_lag_ms"),
              ("下发→实测 [ms]", "health.send_to_meas_lag_ms"), ("控制循环超时次数", "health.loop_overruns")]
    print("  " + "   ".join(f"{t} {cell(k).strip()}" for t, k in health))


SOLVE_TABLE = [
    ("TCP 误差 RMS [mm]", "tcp.{arm}.pos_rms_mm.norm"),
    ("TCP 误差 p95 [mm]", "tcp.{arm}.pos_p95_mm.norm"),
    ("TCP 误差 max [mm]", "tcp.{arm}.pos_max_mm.norm"),
    ("TCP 姿态误差 RMS [deg]", "tcp.{arm}.rot_rms_deg"),
    ("等效时延 [ms]", "tcp.{arm}.equiv_delay_ms"),
    ("位置余量最小 [deg]", "margin.{arm}.pos_deg_min"),
    ("速度 p99 最大关节 [deg/s]", "vel.{arm}.p99_deg_s_max"),
    ("速度 max [deg/s]", "vel.{arm}.max_deg_s"),
    ("速度余量最小 [%]", "margin.{arm}.vel_pct_min"),
    ("加速度 p99 最大关节 [deg/s²]", "acc.{arm}.p99_deg_s2_max"),
    ("加速度 max [deg/s²]", "acc.{arm}.max_deg_s2"),
    ("加速度余量最小 [%]", "margin.{arm}.acc_pct_min"),
]


def print_solve_table(metrics_path: Path) -> None:
    result = json.loads(metrics_path.read_text())
    m = result["metrics"]

    def cell(key: str) -> str:
        value = m.get(key)
        return f"{'-':>8s}" if value is None else f"{value:8.2f}"

    runs = f"，{result['runs']} 次取最差" if result["runs"] > 1 else ""
    print(f"\n{BOLD}求解结果 ({result['case']} on {result['robot']}, S3{runs}){RESET}")
    # A robot with a torso gets a torso column: its joints have margins and distributions but no TCP.
    groups = [("torso", "腰")] if "margin.torso.pos_deg_min" in m else []
    groups += [("left", "左臂"), ("right", "右臂")]
    print("  " + pad("", 32) + "".join(pad(label, 14) for _, label in groups))
    for title, key in SOLVE_TABLE:
        cells = [cell(key.format(arm=g)) for g, _ in groups]
        joints = ""
        if "pos_deg_min" in key:
            joints = "   (" + ", ".join(str(m.get(f"margin.{g}.pos_joint_min")) for g, _ in groups) + ")"
        print("  " + pad(title, 32) + "".join(pad(c, 14) for c in cells) + joints)
    counts = "  ".join(f"{k} {m.get(f'violation.{k}.count', 0):.0f}"
                       for k in ("position", "velocity", "acceleration", "command_step", "hold"))
    print(f"  违规次数 (S1..S4): {counts}")
    precheck = result["precheck"]
    verdict = f"{GREEN}通过{RESET}" if precheck["pass"] else f"{RED}不通过: {precheck['reason']}{RESET}"
    joint = f"{precheck['joint']} " if precheck.get("joint") else ""
    print(f"  真机预检: {joint}速度峰值 {precheck['max_joint_vel_deg_s']:.1f} deg/s (限 {precheck['limit_deg_s']:.0f}) -> "
          f"{verdict}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--mode", choices=("real", "dry", "solve"))
    parser.add_argument("--case", help="case folder name under data/ (or a path)")
    parser.add_argument("--config", help="config file name under config/ (or a path)")
    parser.add_argument("--robot", help="solve: robot folder name under config/robots/ (or a path)")
    parser.add_argument("--no-view", action="store_true", help="solve: skip the 3D playback in the browser")
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
    if plan.mode == "solve":
        session = solve_session(plan.case, plan.robot)
        log = session / "replay.log"
        print(f"{BOLD}计划{RESET}: 轨迹求解  {plan.case.name}  机器人 {plan.robot.name}  {plan.runs} 次")
        print(f"    输出                 {os.path.relpath(session, REPO)}")
        metrics = run_solve(plan.case, plan.robot, plan.runs, session, log)
        if metrics is None:
            return 1
        print_solve_table(metrics)
        info(f"报告 {os.path.relpath(session / 'report.txt', REPO)}，曲线在各 run_*/plots/，数据 run_*/solve.csv")
        if not args.no_view and sys.stdin.isatty() and (session / "run_1/solve.csv").is_file():
            info("3D 回放（浏览器打开下面的地址；远程机器先 ssh -L 7000:localhost:7000 <host>）")
            subprocess.run([sys.executable, "scripts/view_solve.py", str(session)], cwd=REPO)
        return 0 if json.loads(metrics.read_text())["status"] == "ok" else 2

    session = REPO / "runs" / plan.case.name / f"{plan.mode}_{time.strftime('%Y%m%d_%H%M%S')}"
    log = session / "replay.log"
    print(f"{BOLD}计划{RESET}: {plan.mode}  {plan.case.name}  {plan.config.name}  {STAGES[plan.stage]}"
          f"{f'，{plan.runs} 次' if plan.stage == 'full' else ''}")
    for line in config_summary(plan.config):
        print(f"    {line}")
    print(f"    输出                 {os.path.relpath(session, REPO)}")

    solve_result = None
    if plan.mode == "real" and is_s1(plan.config):
        if not Path("/opt/ros/humble").is_dir():
            # The base image: ./benchmark.sh runs the same plan again in the s1 image.
            rerun = ["--mode", "real", "--case", str(plan.case), "--config", str(plan.config), "--stage", plan.stage,
                     "--runs", str(plan.runs)] + (["-y"] if args.yes else [])
            (REPO / "runs/.s1_rerun").write_text("\n".join(rerun) + "\n")
            info("S1 真机要在 s1 镜像里运行（ROS 2 与 SDK 依赖），./benchmark.sh 会切换过去")
            return S1_RERUN_EXIT
        if not Path("/opt/astribot_sdk/astribot_sdk").is_dir():
            warn("容器里没有挂载 Astribot SDK（/opt/astribot_sdk），见 docs/replay.md")
            return 1
    if plan.mode == "real":
        if plan.stage == "full":
            solve_result = find_solve(plan.case, plan.robot)
            if solve_result is None:
                info("这个用例没有和当前配置一致的轨迹求解结果，先跑一次求解（检查关节速度与限位）")
                solve_result = run_solve(plan.case, plan.robot, 1, solve_session(plan.case, plan.robot), log)
                if solve_result is None:
                    warn("轨迹求解失败，停止")
                    return 1
            else:
                info(f"使用轨迹求解结果 {os.path.relpath(solve_result, REPO)}")
            precheck = json.loads(solve_result.read_text())["precheck"]
            if not precheck["pass"]:
                warn(f"真机预检不通过: {precheck['reason']}，停止")
                return 1
            info(f"真机预检通过: {precheck.get('joint', '关节')} 速度峰值 {precheck['max_joint_vel_deg_s']:.1f} deg/s，"
                 f"无位置越限")
            if is_s1(plan.config):
                info("注意: 预检用的是 OCS2 全身 IK 的求解，S1 真机由 SDK 的 WBC 求解，预检只说明可达与余量")
        if not args.yes:
            try:
                input(f"{BOLD}{RED}真机即将运动，确认周围安全后按 Enter 开始，Ctrl-C 取消 ...{RESET}")
            except KeyboardInterrupt:
                print("\naborted")
                return 1

    bridge = None
    if is_s1(plan.config):
        start = start_fake_bridge if plan.dry else start_sdk_bridge
        bridge = start(plan.config, session / "s1_bridge.log")
    try:
        code = run_replay(replay_command(plan, session, solve_result), log)
    finally:
        if bridge is not None:
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.terminate()
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
