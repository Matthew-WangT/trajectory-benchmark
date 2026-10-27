"""pinocchio wrapper: FK, position Jacobian, gravity and inverse dynamics on a RobotProfile."""

from __future__ import annotations

from pathlib import Path

import numpy as np

from . import TrackEvalError
from .robots import ArmSpec, RobotProfile


class ArmModel:
    def __init__(self, profile: RobotProfile, urdf: Path | None = None):
        import pinocchio as pin

        self.pin = pin
        self.profile = profile
        path = urdf or profile.urdf
        if not path.is_file():
            raise TrackEvalError(f"urdf not found: {path}")
        self.model = pin.buildModelFromUrdf(str(path))
        self.data = self.model.createData()
        self.idx_q = np.zeros(len(profile.joint_order), dtype=int)
        self.idx_v = np.zeros(len(profile.joint_order), dtype=int)
        for k, name in enumerate(profile.joint_order):
            if not self.model.existJointName(name):
                raise TrackEvalError(f"joint {name} not in urdf {path}")
            joint = self.model.joints[self.model.getJointId(name)]
            if joint.nq != 1:
                raise TrackEvalError(f"joint {name} has nq={joint.nq}, expected 1")
            self.idx_q[k], self.idx_v[k] = joint.idx_q, joint.idx_v
        self.frame_ids = {}
        for arm in profile.arms:
            if not self.model.existFrame(arm.ee_frame):
                raise TrackEvalError(f"frame {arm.ee_frame} not in urdf {path}")
            self.frame_ids[arm.name] = self.model.getFrameId(arm.ee_frame)
        self.q_neutral = pin.neutral(self.model)
        self.lower = np.array(self.model.lowerPositionLimit)[self.idx_q]
        self.upper = np.array(self.model.upperPositionLimit)[self.idx_q]
        self.vel_limit = np.array(self.model.velocityLimit)[self.idx_v]
        self.effort_limit = np.array(self.model.effortLimit)[self.idx_v]

    @property
    def n(self) -> int:
        return len(self.idx_q)

    def full_q(self, q: np.ndarray) -> np.ndarray:
        qf = self.q_neutral.copy()
        qf[self.idx_q] = q
        return qf

    def full_v(self, v: np.ndarray) -> np.ndarray:
        vf = np.zeros(self.model.nv)
        vf[self.idx_v] = v
        return vf

    def fk(self, q_series: np.ndarray) -> dict[str, np.ndarray]:
        """End-effector positions (N,3) per arm in the URDF root (= base) frame."""
        out = {arm: np.zeros((len(q_series), 3)) for arm in self.frame_ids}
        for n, q in enumerate(q_series):
            self.pin.forwardKinematics(self.model, self.data, self.full_q(q))
            self.pin.updateFramePlacements(self.model, self.data)
            for arm, fid in self.frame_ids.items():
                out[arm][n] = self.data.oMf[fid].translation
        return out

    def frame_poses(self, q_series: np.ndarray, frames) -> dict[str, np.ndarray]:
        """(N,7) x y z qx qy qz qw per frame, in the URDF root frame -- fk() only gives translations."""
        ids = {}
        for frame in frames:
            if not self.model.existFrame(frame):
                raise TrackEvalError(f"frame {frame} not in urdf {self.profile.urdf}")
            ids[frame] = self.model.getFrameId(frame)
        out = {frame: np.zeros((len(q_series), 7)) for frame in ids}
        for n, q in enumerate(q_series):
            self.pin.forwardKinematics(self.model, self.data, self.full_q(q))
            self.pin.updateFramePlacements(self.model, self.data)
            for frame, fid in ids.items():
                placement = self.data.oMf[fid]
                quat = self.pin.Quaternion(placement.rotation)
                out[frame][n] = [*placement.translation, quat.x, quat.y, quat.z, quat.w]
        return out

    def fk_pose(self, q: np.ndarray, arm: str):
        """SE3 of one arm's end effector for one configuration."""
        self.pin.forwardKinematics(self.model, self.data, self.full_q(q))
        self.pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.frame_ids[arm]]

    def jac_pos_series(self, q_series: np.ndarray, arm: ArmSpec) -> np.ndarray:
        """(N,3,7) position Jacobian of the arm's end effector, base-aligned, columns = arm joints."""
        cols = self.idx_v[self.profile.joint_index(arm)]
        fid = self.frame_ids[arm.name]
        out = np.zeros((len(q_series), 3, len(cols)))
        for n, q in enumerate(q_series):
            J = self.pin.computeFrameJacobian(self.model, self.data, self.full_q(q), fid, self.pin.LOCAL_WORLD_ALIGNED)
            out[n] = J[:3][:, cols]
        return out

    def gravity_series(self, q_series: np.ndarray) -> np.ndarray:
        out = np.zeros((len(q_series), self.n))
        for n, q in enumerate(q_series):
            out[n] = np.asarray(self.pin.computeGeneralizedGravity(self.model, self.data, self.full_q(q)))[self.idx_v]
        return out

    def rnea_series(self, q_series: np.ndarray, qd_series: np.ndarray, qdd_series: np.ndarray) -> np.ndarray:
        """Full inverse dynamics torque (gravity + inertia + coriolis) per joint."""
        out = np.zeros((len(q_series), self.n))
        for n in range(len(q_series)):
            tau = self.pin.rnea(
                self.model, self.data, self.full_q(q_series[n]), self.full_v(qd_series[n]), self.full_v(qdd_series[n])
            )
            out[n] = np.asarray(tau)[self.idx_v]
        return out
