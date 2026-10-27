"""ROS-independent IMU samples of a site's accelerometer and gyro, at a fixed rate."""

from dataclasses import dataclass

import mujoco
import numpy as np

from xbot2_py_mujoco.sensor_utils import (
    RateSchedule, check_frame, check_rate, check_topic, check_unique_topics,
)


@dataclass(frozen=True)
class ImuCfg:
    site: str            # frame and orientation
    accelerometer: str   # sensor names
    gyro: str
    topic: str = '/imu'
    frame_id: str | None = None
    rate: float = 200.0

    def __post_init__(self):
        for name in ('site', 'accelerometer', 'gyro'):
            if not getattr(self, name):
                raise ValueError(f'IMU {name} is required')
        check_topic(self.topic, 'IMU')
        check_rate(self.rate, 'IMU')
        frame = self.frame_id if self.frame_id is not None else self.site
        check_frame(frame, 'IMU')
        object.__setattr__(self, 'frame_id', frame)

    @classmethod
    def from_xml(cls, element):
        allowed = {'site', 'accelerometer', 'gyro', 'topic', 'frame_id', 'rate'}
        if set(element.attrib) - allowed or len(element):
            raise ValueError('Unknown imu attributes or child elements')
        kwargs = {k: element.get(k) for k in ('site', 'accelerometer', 'gyro', 'topic', 'frame_id')
                  if element.get(k) is not None}
        if element.get('rate') is not None:
            kwargs['rate'] = float(element.get('rate'))
        return cls(**{'site': '', 'accelerometer': '', 'gyro': '', **kwargs})


def validate_imus(model, configs):
    for cfg in configs:
        if not isinstance(cfg, ImuCfg):
            raise TypeError('imus must contain ImuCfg instances')
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, cfg.site) < 0:
            raise ValueError(f'Unknown IMU site: {cfg.site}')
        for name in (cfg.accelerometer, cfg.gyro):
            sensor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sensor < 0 or model.sensor_dim[sensor] != 3:
                raise ValueError(f'IMU sensor {name} must be a 3D sensor of the model')
    check_unique_topics(configs, 'IMU')


@dataclass(frozen=True)
class ImuSample:
    time: float
    orientation: np.ndarray       # w x y z, site in world
    angular_velocity: np.ndarray  # site frame
    linear_acceleration: np.ndarray


class Imu:
    """Call step after every mj_step, from the physics thread: the samples are of the step's start,
    as MuJoCo's sensors."""

    def __init__(self, model, cfg: ImuCfg):
        validate_imus(model, [cfg])
        self.cfg = cfg
        self._model = model
        self._schedule = RateSchedule(cfg.rate, model.opt.timestep)

    def step(self, data) -> ImuSample | None:
        time = float(data.time) - self._model.opt.timestep
        if not self._schedule.due(time):
            return None
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, data.site(self.cfg.site).xmat)
        return ImuSample(time, quat, data.sensor(self.cfg.gyro).data.copy(),
                         data.sensor(self.cfg.accelerometer).data.copy())
