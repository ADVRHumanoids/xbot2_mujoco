"""Publishing the poses of MuJoCo bodies with simpleposes, each relative to a reference body."""
from __future__ import annotations

import mujoco
import numpy as np

try:
    from simpleposes import PosePublisher
except ImportError as e:
    raise ImportError("publishing link poses needs the simpleposes package "
                      "(uv pip install -e path/to/simpleposes)") from e


def _quat_mul(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """Hamilton product of (n, 4) wxyz quaternions."""
    aw, ax, ay, az = a.T
    bw, bx, by, bz = b.T
    return np.stack([aw*bw - ax*bx - ay*by - az*bz,
                     aw*bx + ax*bw + ay*bz - az*by,
                     aw*by - ax*bz + ay*bw + az*bx,
                     aw*bz + ax*by - ay*bx + az*bw], axis=1)


class BodyPosePublisher:
    """Publishes the pose of bodies, each relative to its own reference body ("world" is the world frame)."""

    def __init__(self, model: mujoco.MjModel, links: dict[str, str], endpoint: str):
        """links maps each body to the body its pose is expressed in. endpoint is the address the publisher binds
        to, e.g. "tcp://*:5800"."""
        if not links:
            raise ValueError("no links to publish")
        self._body_ids = np.array([self._body_id(model, link) for link in links])
        self._ref_ids = np.array([self._body_id(model, frame) for frame in links.values()])
        # mj_step leaves in data the kinematics of the state it started from: publish() recomputes them for the
        # current state here, without touching the simulator's data
        self._model = model
        self._data = mujoco.MjData(model)
        self.publisher = PosePublisher(links, endpoint)
        self.endpoint = self.publisher.endpoint

    @staticmethod
    def _body_id(model: mujoco.MjModel, name: str) -> int:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise ValueError(f"pose link or frame '{name}' is not a body of the model")
        return body_id

    def poses(self, data: mujoco.MjData) -> np.ndarray:
        """(n_links, 7) x, y, z, qx, qy, qz, qw of each link in its reference body, from the kinematics in data."""
        ref_mat = data.xmat[self._ref_ids].reshape(-1, 3, 3)
        ref_quat_conj = data.xquat[self._ref_ids] * [1.0, -1.0, -1.0, -1.0]
        pos = np.einsum('nji,nj->ni', ref_mat, data.xpos[self._body_ids] - data.xpos[self._ref_ids])
        quat = _quat_mul(ref_quat_conj, data.xquat[self._body_ids])
        return np.concatenate([pos, quat[:, [1, 2, 3, 0]]], axis=1)

    def publish(self, data: mujoco.MjData):
        """Publish the poses of the current state of data (its qpos and mocap bodies), stamped with data.time."""
        self._data.qpos[:] = data.qpos
        self._data.mocap_pos[:] = data.mocap_pos
        self._data.mocap_quat[:] = data.mocap_quat
        mujoco.mj_kinematics(self._model, self._data)
        self.publisher.publish(self.poses(self._data), data.time)

    def close(self):
        self.publisher.close()
