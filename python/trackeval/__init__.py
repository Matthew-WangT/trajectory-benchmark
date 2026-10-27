"""Offline evaluation of the dual-arm trajectory benchmark, adapted from mindon_robotics python/trackeval.

Layers: robots (profiles) -> csvio (delivery CSV of docs/test_requirements.md section 4) -> model (pinocchio, only
for robots with a URDF) -> analyzers (one per stage) -> report (text + metrics JSON). The analyzers only need the
columns the requirements ask for, so the CSV of any robot can be evaluated; the stages that need a kinematic model
are skipped without one.
"""


class TrackEvalError(Exception):
    """Unusable input: missing columns, unknown URDF joint, profile that does not fit the CSV."""
