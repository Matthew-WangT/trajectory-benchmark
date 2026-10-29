"""The files a trajectory solve depends on, hashed so that a stale solve result is not taken for the current one."""

from __future__ import annotations

import hashlib
from pathlib import Path

from .robots import read_yaml


def input_files(robot_dir: Path, case_dir: Path) -> dict[str, Path]:
    """Name -> path of the case and robot files the solve result depends on."""
    robot_dir, case_dir = Path(robot_dir).resolve(), Path(case_dir).resolve()
    robot = read_yaml(robot_dir / "robot.yaml")
    controller = (robot_dir / robot["controller"]).resolve()
    ctrl = read_yaml(controller)
    files = {
        "case/eef_command.csv": case_dir / "eef_command.csv",
        "case/meta.json": case_dir / "meta.json",
        "robot/robot.yaml": robot_dir / "robot.yaml",
        "robot/" + Path(robot["urdf"]).name: (robot_dir / robot["urdf"]).resolve(),
        "robot/" + controller.name: controller,
    }
    # One task .info per arm (dual_arm) or one for the whole body.
    for key in ("left_info", "right_info", "info"):
        if key in ctrl:
            path = (controller.parent / ctrl[key]).resolve()
            files["robot/" + path.name] = path
    return files


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with open(path, "rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def input_hashes(robot_dir: Path, case_dir: Path) -> dict[str, str]:
    return {name: sha256(path) for name, path in input_files(robot_dir, case_dir).items() if path.is_file()}
