"""ROS-independent odometry samples of a body (ground truth), at a fixed rate."""

from dataclasses import dataclass

import mujoco
import numpy as np

from xbot2_py_mujoco.sensor_utils import (
    RateSchedule, check_frame, check_rate, check_topic, check_unique_topics,
)


@dataclass(frozen=True)
class OdometryCfg:
    body: str
    topic: str = '/odometry'
    frame_id: str = 'world'
    child_frame_id: str | None = None
    rate: float = 200.0

    def __post_init__(self):
        if not self.body:
            raise ValueError('Odometry body is required')
        check_topic(self.topic, 'Odometry')
        check_rate(self.rate, 'Odometry')
        child = self.child_frame_id if self.child_frame_id is not None else self.body
        check_frame(self.frame_id, 'Odometry')
        check_frame(child, 'Odometry')
        object.__setattr__(self, 'child_frame_id', child)

    @classmethod
    def from_xml(cls, element):
        allowed = {'body', 'topic', 'frame_id', 'child_frame_id', 'rate'}
        if set(element.attrib) - allowed or len(element):
            raise ValueError('Unknown odometry attributes or child elements')
        kwargs = {k: element.get(k) for k in ('body', 'topic', 'frame_id', 'child_frame_id')
                  if element.get(k) is not None}
        if element.get('rate') is not None:
            kwargs['rate'] = float(element.get('rate'))
        return cls(**{'body': '', **kwargs})


def validate_odometries(model, configs):
    for cfg in configs:
        if not isinstance(cfg, OdometryCfg):
            raise TypeError('odometries must contain OdometryCfg instances')
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, cfg.body) < 0:
            raise ValueError(f'Unknown odometry body: {cfg.body}')
    check_unique_topics(configs, 'odometry')


@dataclass(frozen=True)
class OdometrySample:
    time: float
    position: np.ndarray          # body in world
    orientation: np.ndarray       # w x y z
    linear_velocity: np.ndarray   # body frame
    angular_velocity: np.ndarray  # body frame


class Odometry:
    """Call step after every mj_step, from the physics thread: the samples are of the step's start,
    as MuJoCo's derived quantities."""

    def __init__(self, model, cfg: OdometryCfg):
        validate_odometries(model, [cfg])
        self.cfg = cfg
        self._model = model
        self._schedule = RateSchedule(cfg.rate, model.opt.timestep)

    def step(self, data) -> OdometrySample | None:
        time = float(data.time) - self._model.opt.timestep
        if not self._schedule.due(time):
            return None
        body = data.body(self.cfg.body)
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(self._model, data, mujoco.mjtObj.mjOBJ_BODY, body.id, velocity, 1)
        return OdometrySample(time, body.xpos.copy(), body.xquat.copy(), velocity[3:].copy(),
                              velocity[:3].copy())
