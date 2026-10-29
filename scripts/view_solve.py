#!/usr/bin/env python3
"""3D playback of a trajectory solve in the browser (meshcat): the robot following q_cmd, the target and the solved
TCP paths of both arms, and markers of the current target / solved TCP.

    python3 scripts/view_solve.py runs/case02_fold_clothes_spd50/solve_marvin_20261029_101500 [--run 2]

Prints the viewer URL and keeps serving it until Enter or Ctrl-C. The animation controls (play / pause, time slider,
time scale) are in the "Animations" folder of the meshcat panel. In the container the URL is reachable from the host
(host networking); on a remote machine forward the port first, e.g. `ssh -L 7000:localhost:7000 infer-10`.
Reads run_<i>/solve.csv of scripts/analyze_solve.py and the robot folder named by run_info.json.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

from trackeval.csvio import read_csv  # noqa: E402
from trackeval.robots import REPO_ROOT, load_profile  # noqa: E402

POSE = ("x", "y", "z", "qx", "qy", "qz", "qw")
ARMS = (("left", "l"), ("right", "r"))
# Target / solved colors of the paths and the markers.
COLORS = {"target": 0x1f77b4, "solved": 0xff7f0e}


def resolve_meshes(urdf: Path, robot_dir: Path) -> Path:
    """URDF copy with absolute mesh paths: relative ones against the URDF folder, package://<path> against the robot
    folder (or the URDF folder), so that no ROS package path is needed."""
    text = urdf.read_text()

    def absolute(match: re.Match) -> str:
        path = match.group(1)
        if path.startswith("package://"):
            rest = path[len("package://"):]
            candidates = [robot_dir / rest, urdf.parent / rest]
            found = next((c for c in candidates if c.is_file()), None)
            return f'filename="{found.resolve() if found else path}"'
        if path.startswith("file://"):
            path = path[len("file://"):]
        return f'filename="{(urdf.parent / path).resolve()}"'

    out = Path(tempfile.mkdtemp(prefix="view_solve_")) / urdf.name
    out.write_text(re.sub(r'filename="([^"]+)"', absolute, text))
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("path", type=Path, help="solve session folder (holding run_*/) or one run folder")
    parser.add_argument("--run", type=int, default=1, help="run of the session to show")
    parser.add_argument("--robot", type=Path, help="robot folder, by default robots/<robot of run_info.json>")
    parser.add_argument("--fps", type=float, default=30.0, help="animation frame rate")
    parser.add_argument("--from-phase", default="S1", help="first phase shown (S0 is the hold at q_pd)")
    parser.add_argument("--no-wait", action="store_true", help="exit after loading (the viewer goes with it)")
    args = parser.parse_args()

    import meshcat.animation
    import meshcat.geometry as mg
    import meshcat.transformations as tf
    import pinocchio as pin
    from pinocchio.visualize import MeshcatVisualizer

    run = args.path if (args.path / "run_info.json").is_file() else args.path / f"run_{args.run}"
    if not (run / "solve.csv").is_file():
        print(f"error: {run / 'solve.csv'} missing, run scripts/analyze_solve.py first", file=sys.stderr)
        return 1
    info = json.loads((run / "run_info.json").read_text())
    robot_dir = (args.robot or REPO_ROOT / "robots" / info["robot"]).resolve()
    profile = load_profile(robot_dir)

    header, values, text = read_csv(run / "solve.csv")
    col = {h: i for i, h in enumerate(header)}
    phase = np.array(text["phase"])
    phases = ["S0", "S1", "S2", "S3", "S4"]
    keep = np.isin(phase, phases[phases.index(args.from_phase):]) if args.from_phase in phases else phase != "S0"
    values, phase = values[keep], phase[keep]
    t = values[:, col["t_s"]]
    q_arms = values[:, [col[f"{p}_q_cmd_{j}"] for _, p in ARMS for j in range(1, 8)]]
    paths = {(arm, kind): values[:, [col[f"{p}_{kind}_{f}"] for f in POSE[:3]]]
             for arm, p in ARMS for kind in ("target", "solved")}

    urdf = resolve_meshes(profile.urdf, robot_dir)
    model, collision_model, visual_model = pin.buildModelsFromUrdf(str(urdf))
    idx_q = [model.joints[model.getJointId(name)].idx_q for name in profile.joint_order]
    q_full = pin.neutral(model)

    viz = MeshcatVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=False)
    viz.loadViewerModel()
    viewer = viz.viewer
    viewer["paths"].delete()
    for (arm, kind), xyz in paths.items():
        points = xyz[:: max(1, len(xyz) // 4000)].T.astype(np.float32)
        viewer[f"paths/{arm}_{kind}"].set_object(
            mg.Line(mg.PointsGeometry(points), mg.LineBasicMaterial(color=COLORS[kind], linewidth=2)))
        viewer[f"markers/{arm}_{kind}"].set_object(
            mg.Sphere(0.012), mg.MeshLambertMaterial(color=COLORS[kind], opacity=0.9))

    names = [viz.getViewerNodeName(g, pin.GeometryType.VISUAL) for g in visual_model.geometryObjects]
    scales = [g.meshScale for g in visual_model.geometryObjects]

    def pose_robot(k: int, target) -> None:
        q_full[idx_q] = q_arms[k]
        pin.forwardKinematics(model, viz.data, q_full)
        pin.updateGeometryPlacements(model, viz.data, visual_model, viz.visual_data)
        for g, (name, scale) in enumerate(zip(names, scales)):
            T = viz.visual_data.oMg[g].homogeneous.copy()
            T[:3, :3] = T[:3, :3] @ np.diag(scale)
            target[name].set_transform(T)
        for (arm, kind), xyz in paths.items():
            target[f"markers/{arm}_{kind}"].set_transform(tf.translation_matrix(xyz[k]))

    # meshcat animations count frames; the frame times follow the run clock from the first shown sample.
    frames = np.arange(t[0], t[-1], 1.0 / args.fps)
    index = np.clip(np.searchsorted(t, frames), 0, len(t) - 1)
    anim = meshcat.animation.Animation(default_framerate=args.fps)
    for i, k in enumerate(index):
        with anim.at_frame(viewer, i) as frame:
            pose_robot(k, frame)
    pose_robot(index[0], viewer)
    viewer.set_animation(anim, play=False)

    s3 = np.flatnonzero(phase == "S3")
    print(f"{info['case']} on {profile.name}, {run.name}: {t[-1] - t[0]:.1f} s from {phase[0]}"
          + (f", S3 at {t[s3[0]] - t[0]:.1f} s of the animation" if len(s3) else ""))
    print("blue: target TCP path, orange: solved FK(q_cmd); play in the panel under Animations")
    print(f"viewer: {viewer.url()}")
    if args.no_wait:
        return 0
    try:
        input("Enter to close the viewer ...")
    except (KeyboardInterrupt, EOFError):
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
