"""Robot profiles: the arms, their joint order and end-effector frames as the analyzers see them."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


@dataclass(frozen=True)
class ArmSpec:
    name: str  # "left" / "right"
    prefix: str  # column prefix of the delivery CSV, "l" / "r"
    joints: tuple[str, ...]  # joint names in the order of the CSV joint columns
    ee_frame: str | None = None  # URDF frame of the TCP, None without a URDF


@dataclass(frozen=True)
class RobotProfile:
    name: str
    urdf: Path | None
    arms: tuple[ArmSpec, ...]
    joint_order: tuple[str, ...]  # both arms side by side, the left arm first

    def arm(self, name: str) -> ArmSpec:
        for a in self.arms:
            if a.name == name:
                return a
        raise KeyError(name)

    def joint_index(self, arm: ArmSpec) -> list[int]:
        return [self.joint_order.index(j) for j in arm.joints]


def _dual_arm(name: str, urdf: Path | None, left_joints, right_joints, ee_l=None, ee_r=None) -> RobotProfile:
    left = ArmSpec("left", "l", tuple(left_joints), ee_l)
    right = ArmSpec("right", "r", tuple(right_joints), ee_r)
    return RobotProfile(name=name, urdf=urdf, arms=(left, right), joint_order=left.joints + right.joints)


MARVIN = _dual_arm(
    "marvin",
    REPO_ROOT / "assets/urdf/serial_marvin_with_gripper.urdf",
    (f"Joint{i}_L" for i in range(1, 8)),
    (f"Joint{i}_R" for i in range(1, 8)),
    "left_gripper_f90c_end_effector_link",
    "right_gripper_f90c_end_effector_link",
)


def generic(n_left: int, n_right: int) -> RobotProfile:
    """Profile of a robot without a URDF, its joints named after the CSV columns."""
    return _dual_arm(
        "generic", None, (f"l_joint{i}" for i in range(1, n_left + 1)), (f"r_joint{i}" for i in range(1, n_right + 1))
    )


PROFILES = {"marvin": MARVIN}
