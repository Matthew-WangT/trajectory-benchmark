"""Robot profiles: the arms, their joint order and end-effector frames as the analyzers see them."""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
ROBOTS_DIR = REPO_ROOT / "robots"


@dataclass(frozen=True)
class ArmSpec:
    """An arm, or the torso as a group of joints without an end effector."""

    name: str  # "left" / "right" / "torso"
    prefix: str  # column prefix of the delivery CSV, "l" / "r" / "t"
    joints: tuple[str, ...]  # joint names in the order of the CSV joint columns
    ee_frame: str | None = None  # URDF frame of the TCP, None without a URDF


@dataclass(frozen=True)
class RobotProfile:
    name: str
    urdf: Path | None
    arms: tuple[ArmSpec, ...]
    joint_order: tuple[str, ...]  # the torso joints (if any), then both arms side by side, the left arm first
    robot_dir: Path | None = None  # robots/<name>/, None for a profile without one
    max_acc: tuple[float, ...] | None = None  # rad/s^2 per joint of joint_order, the URDF has none
    torso: ArmSpec | None = None  # the torso joints shared by both arms, None on a fixed-base dual arm

    @property
    def groups(self) -> tuple[ArmSpec, ...]:
        """The joint groups in the order of joint_order: the torso (if any) and the arms."""
        return ((self.torso,) if self.torso else ()) + self.arms

    def arm(self, name: str) -> ArmSpec:
        for a in self.arms:
            if a.name == name:
                return a
        raise KeyError(name)

    def joint_index(self, arm: ArmSpec) -> list[int]:
        return [self.joint_order.index(j) for j in arm.joints]

    def chain_index(self, arm: ArmSpec) -> list[int]:
        """The joints that move the arm's end effector: the torso joints (if any) and the arm."""
        return (self.joint_index(self.torso) if self.torso else []) + self.joint_index(arm)


def _dual_arm(name: str, urdf: Path | None, left_joints, right_joints, ee_l=None, ee_r=None, torso_joints=(),
              **extra) -> RobotProfile:
    left = ArmSpec("left", "l", tuple(left_joints), ee_l)
    right = ArmSpec("right", "r", tuple(right_joints), ee_r)
    torso = ArmSpec("torso", "t", tuple(torso_joints)) if torso_joints else None
    return RobotProfile(name=name, urdf=urdf, arms=(left, right),
                        joint_order=(torso.joints if torso else ()) + left.joints + right.joints, torso=torso, **extra)


def read_yaml(path: Path) -> dict:
    from ruamel.yaml import YAML

    return YAML(typ="safe").load(path.read_text())


def load_profile(robot_dir: Path) -> RobotProfile:
    """Profile of robots/<name>/robot.yaml, the paths in it relative to that folder."""
    robot_dir = Path(robot_dir).resolve()
    cfg = read_yaml(robot_dir / "robot.yaml")
    groups = [g for g in ("torso", "left", "right") if cfg["joints"].get(g)]
    max_acc = cfg.get("max_acc_deg_s2")
    # Only with a limit for every joint group.
    if max_acc and not all(g in max_acc for g in groups):
        max_acc = None
    return _dual_arm(
        cfg.get("name", robot_dir.name),
        (robot_dir / cfg["urdf"]).resolve(),
        cfg["joints"]["left"],
        cfg["joints"]["right"],
        cfg["ee_frames"]["left"],
        cfg["ee_frames"]["right"],
        torso_joints=cfg["joints"].get("torso") or (),
        robot_dir=robot_dir,
        max_acc=tuple(math.radians(v) for g in groups for v in max_acc[g]) if max_acc else None,
    )


def generic(n_left: int, n_right: int) -> RobotProfile:
    """Profile of a robot without a URDF, its joints named after the CSV columns."""
    return _dual_arm(
        "generic", None, (f"l_joint{i}" for i in range(1, n_left + 1)), (f"r_joint{i}" for i in range(1, n_right + 1))
    )


# The robots of robots/*/robot.yaml by name.
PROFILES = {p.name: p for p in (load_profile(d.parent) for d in sorted(ROBOTS_DIR.glob("*/robot.yaml")))}
MARVIN = PROFILES["marvin"]
