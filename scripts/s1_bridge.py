#!/usr/bin/env python3
"""Sidecar of the S1 replay: drives the Astribot S1 through its SDK, one command per SDK period, for the replay
(build/bin/replay --config config/s1.yaml) that answers each feedback frame over UDP. See src/s1/bridge_protocol.h.

    # In the s1 image, next to the robot (scripts/benchmark.py starts it):
    docker/s1_sdk_env.sh python3 scripts/s1_bridge.py --config config/s1.yaml
    # Without the robot and the SDK, in the benchmark image: a kinematic stand-in of the S1 (ideal joints, one
    # damped least squares step toward the cartesian targets per period, like one WBC iteration):
    python3 scripts/s1_bridge.py --config config/s1.yaml --fake

Every period: read the robot, send a Feedback to the replay, wait up to bridge.reply_timeout_s for the Command with
the same seq, apply it (or repeat the previous one and count a miss), sleep to the next period. Until the replay
answers the first time nothing is commanded and nothing counts as a miss. Stops commanding (the robot holds its last
desired position, no stop_robot) on a stop command, on bridge.max_consecutive_miss misses in a row, or on Ctrl-C.
"""

from __future__ import annotations

import argparse
import socket
import struct
import sys
import time
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[1]

JOINTS, POSE = 18, 7
FEEDBACK_MAGIC, COMMAND_MAGIC, VERSION = 0x42463153, 0x4D433153, 1
FLAG_FAKE, FLAG_STOPPED = 1, 2
IDLE, JOINT, CARTESIAN, STOP = 0, 1, 2, 3
# Feedback: magic, version, flags, seq, consecutive_miss, total_miss, reserved, t_mono_s, read_s, command_s,
# q, dq, tau, q_desired (18 each), ee_left, ee_right (7 each).
FEEDBACK = struct.Struct(f"<IHHIIII3d{4 * JOINTS}d{2 * POSE}d")
# Command: magic, version, mode, seq, reserved, q (18), pose_left, pose_right (7 each).
COMMAND = struct.Struct(f"<IHHII{JOINTS}d{2 * POSE}d")


def read_yaml(path: Path) -> dict:
    try:
        import yaml

        return yaml.safe_load(path.read_text())
    except ImportError:
        from ruamel.yaml import YAML

        return YAML(typ="safe").load(path.read_text())


class SdkRobot:
    """The S1 through the Astribot SDK: torso, left arm, right arm joints and the arm end effectors (chassis frame)."""

    fake = False

    def __init__(self, freq_hz: float, torso_command: str):
        from astribot_sdk.core.astribot_api.astribot_client import Astribot
        import astribot_ros_middleware as middleware

        self.bot = Astribot(freq=freq_hz, node_name="eef_benchmark_s1_bridge")
        self.rate = middleware.Rate(freq_hz)
        self.parts = [self.bot.torso_name, self.bot.arm_left_name, self.bot.arm_right_name]
        self.arms = [self.bot.arm_left_name, self.bot.arm_right_name]
        self.hold_torso = torso_command == "hold"
        self.torso_pose = None

    def read(self):
        b, parts = self.bot, self.parts
        flat = lambda groups: np.concatenate([np.asarray(g, dtype=float) for g in groups])  # noqa: E731
        q = flat(b.get_current_joints_position(parts))
        dq = flat(b.get_current_joints_velocity(parts))
        tau = flat(b.get_current_joints_torque(parts))
        q_desired = flat(b.get_desired_joints_position(parts))
        ee = b.get_current_cartesian_pose(self.arms, frame="chassis")
        return q, dq, tau, q_desired, np.asarray(ee[0], dtype=float), np.asarray(ee[1], dtype=float)

    def command(self, mode: int, q: np.ndarray, pose_left: np.ndarray, pose_right: np.ndarray) -> None:
        if mode == JOINT:
            self.torso_pose = None
            self.bot.set_joints_position(self.parts, [list(q[:4]), list(q[4:11]), list(q[11:])],
                                         control_way="direct", use_wbc=False, add_default_torso=False)
        elif mode == CARTESIAN:
            if self.hold_torso:
                if self.torso_pose is None:
                    self.torso_pose = self.bot.get_current_cartesian_pose([self.bot.torso_name], frame="chassis")[0]
                self.bot.set_cartesian_pose(self.parts, [list(self.torso_pose), list(pose_left), list(pose_right)],
                                            control_way="direct", use_wbc=True, add_default_torso=False)
            else:
                self.bot.set_cartesian_pose(self.arms, [list(pose_left), list(pose_right)], control_way="direct",
                                            use_wbc=True, add_default_torso=False)

    def sleep(self) -> None:
        self.rate.sleep()


class FakeRobot:
    """Kinematic stand-in of the S1 on the robots/s1 URDF: joint commands are followed exactly, a cartesian command
    moves the joints by a few damped least squares steps toward the SDK end-effector targets (all 18 joints, the
    torso weighted down, a pull toward q_pd in the null space), like the WBC iterating once per period."""

    fake = True

    def __init__(self, cfg: dict, robot_dir: Path, freq_hz: float):
        import pinocchio as pin

        robot = read_yaml(robot_dir / "robot.yaml")
        self.pin = pin
        self.model = pin.buildModelFromUrdf(str(robot_dir / robot["urdf"]))
        self.data = self.model.createData()
        names = robot["joints"]["torso"] + robot["joints"]["left"] + robot["joints"]["right"]
        self.idx_q = [self.model.joints[self.model.getJointId(n)].idx_q for n in names]
        self.idx_v = [self.model.joints[self.model.getJointId(n)].idx_v for n in names]
        frames = cfg["sdk_frame"]["ee_frames"]
        self.ee = [self.model.getFrameId(frames["left"]), self.model.getFrameId(frames["right"])]
        self.lower = np.array(self.model.lowerPositionLimit)[self.idx_q]
        self.upper = np.array(self.model.upperPositionLimit)[self.idx_q]
        q_pd = robot["startup"]["q_pd"]
        self.q = np.array(q_pd["torso"] + q_pd["left"] + q_pd["right"], dtype=float)
        self.q_pd = self.q.copy()
        self.q_prev = self.q.copy()
        self.dt = 1.0 / freq_hz
        # benchmark_base in the chassis frame: astribot_torso_base sits torso_base_in_chassis above the chassis.
        self.full = pin.neutral(self.model)
        self.fk()
        torso_base = self.data.oMf[self.model.getFrameId("astribot_torso_base")]
        chassis_torso_base = pin.SE3(np.eye(3), np.array(cfg["sdk_frame"]["torso_base_in_chassis"], dtype=float))
        self.chassis_bb = chassis_torso_base * torso_base.inverse()
        self.next_tick = time.monotonic()

    def fk(self) -> None:
        self.full[self.idx_q] = self.q
        self.pin.forwardKinematics(self.model, self.data, self.full)
        self.pin.updateFramePlacements(self.model, self.data)

    def pose7(self, placement) -> np.ndarray:
        quat = self.pin.Quaternion(placement.rotation)
        return np.r_[placement.translation, quat.x, quat.y, quat.z, quat.w]

    def read(self):
        self.fk()
        dq = (self.q - self.q_prev) / self.dt
        ee = [self.pose7(self.chassis_bb * self.data.oMf[f]) for f in self.ee]
        return self.q.copy(), dq, np.zeros(JOINTS), self.q.copy(), ee[0], ee[1]

    def command(self, mode: int, q: np.ndarray, pose_left: np.ndarray, pose_right: np.ndarray) -> None:
        pin = self.pin
        self.q_prev = self.q.copy()
        if mode == JOINT:
            self.q = np.clip(q, self.lower, self.upper)
        elif mode == CARTESIAN:
            targets = [self.chassis_bb.inverse() * pin.XYZQUATToSE3(np.r_[p[:3], p[3:]])
                       for p in (pose_left, pose_right)]
            weight = np.r_[np.full(4, 0.2), np.ones(14)]
            for _ in range(3):
                self.fk()
                errors, jacobians = [], []
                for fid, target in zip(self.ee, targets):
                    current = self.data.oMf[fid]
                    # World aligned error at the frame: position, then rotation.
                    err = np.r_[target.translation - current.translation,
                                current.rotation @ pin.log3(current.rotation.T @ target.rotation)]
                    J = pin.computeFrameJacobian(self.model, self.data, self.full, fid, pin.LOCAL_WORLD_ALIGNED)
                    errors.append(err)
                    jacobians.append(J[:, self.idx_v])
                e, J = np.concatenate(errors), np.vstack(jacobians) * weight
                J_pinv = J.T @ np.linalg.inv(J @ J.T + 1e-4 * np.eye(12))
                # Task step, and a pull toward q_pd in its null space so that the redundancy does not drift to a limit.
                null = (np.eye(JOINTS) - J_pinv @ J) @ (0.05 * (self.q_pd - self.q) / weight)
                step = weight * (J_pinv @ e + null)
                self.q = np.clip(self.q + step, self.lower, self.upper)

    def sleep(self) -> None:
        self.next_tick += self.dt
        delay = self.next_tick - time.monotonic()
        if delay > 0:
            time.sleep(delay)
        else:
            self.next_tick = time.monotonic()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--config", type=Path, default=REPO_ROOT / "config" / "s1.yaml")
    parser.add_argument("--fake", action="store_true", help="kinematic stand-in of the robot, no SDK")
    parser.add_argument("--host", default="127.0.0.1", help="host of the replay")
    args = parser.parse_args()

    cfg = read_yaml(args.config)
    bridge = cfg["bridge"]
    freq = float(bridge.get("freq_hz", 250.0))
    reply_timeout = float(bridge.get("reply_timeout_s", 0.002))
    max_miss = int(bridge.get("max_consecutive_miss", 10))
    robot_dir = (args.config.parent / cfg["robot_dir"]).resolve()
    robot = FakeRobot(cfg, robot_dir, freq) if args.fake else SdkRobot(freq, cfg.get("torso_command", "free"))
    flags = FLAG_FAKE if robot.fake else 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    replay = (args.host, int(bridge.get("port", 47011)))
    print(f"[s1_bridge] {'fake robot' if robot.fake else 'SDK'}, {freq:.0f} Hz, replay at {replay[0]}:{replay[1]}",
          flush=True)

    seq, consecutive, total = 0, 0, 0
    active = False
    last = (IDLE, np.zeros(JOINTS), np.zeros(POSE), np.zeros(POSE))
    command_s = 0.0
    stopped = False
    try:
        while True:
            start = time.monotonic()
            q, dq, tau, q_desired, ee_left, ee_right = robot.read()
            read_s = time.monotonic() - start
            frame = FEEDBACK.pack(FEEDBACK_MAGIC, VERSION, flags | (FLAG_STOPPED if stopped else 0), seq, consecutive,
                                  total, 0, start, read_s, command_s, *q, *dq, *tau, *q_desired, *ee_left, *ee_right)
            # Drop answers to earlier frames that came in late.
            while True:
                try:
                    sock.recv(COMMAND.size)
                except (BlockingIOError, ConnectionRefusedError):
                    break
            try:
                sock.sendto(frame, replay)
            except ConnectionRefusedError:
                pass
            if stopped:
                break
            answer = None
            deadline = time.monotonic() + reply_timeout
            while answer is None and time.monotonic() < deadline:
                try:
                    data = sock.recv(COMMAND.size)
                except (BlockingIOError, ConnectionRefusedError):
                    time.sleep(5e-5)
                    continue
                if len(data) == COMMAND.size:
                    fields = COMMAND.unpack(data)
                    if fields[0] == COMMAND_MAGIC and fields[1] == VERSION and fields[3] == seq:
                        answer = fields
            if answer is not None:
                mode = answer[2]
                values = np.array(answer[5:])
                last = (mode, values[:JOINTS], values[JOINTS:JOINTS + POSE], values[JOINTS + POSE:])
                consecutive = 0
                if not active:
                    print("[s1_bridge] replay answered, commanding", flush=True)
                active = True
            elif active:
                consecutive += 1
                total += 1
            if active:
                seq += 1
            if last[0] == STOP:
                print("[s1_bridge] stop received", flush=True)
                stopped = True
            elif consecutive > max_miss:
                print(f"[s1_bridge] {consecutive} periods without an answer, stopped commanding", flush=True)
                stopped = True
            elif active:
                # Unanswered: the previous command again, so that the SDK sees no step to zero.
                start = time.monotonic()
                robot.command(*last)
                command_s = time.monotonic() - start
            robot.sleep()
    except KeyboardInterrupt:
        print("[s1_bridge] interrupted, stopped commanding", flush=True)
    print(f"[s1_bridge] {seq} periods, {total} missed answers", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
