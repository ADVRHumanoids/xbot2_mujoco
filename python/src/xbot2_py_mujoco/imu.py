"""IMUs of the model (a site with a gyro and an accelerometer), as XBot2's bridge reads them, and
ROS-independent samples of one at a fixed rate."""

from dataclasses import dataclass

import mujoco
import numpy as np

from xbot2_py_mujoco.sensor_utils import (
    RateSchedule, check_frame, check_rate, check_topic, check_unique_topics,
)


def discover_imus(model: mujoco.MjModel) -> dict[str, dict[str, int]]:
    """{site name: {'gyro_id', 'acc_id', 'site_id'}} of the sites with both sensors."""
    site_imu_map = {}
    for i in range(model.nsensor):
        if model.sensor_objtype[i] != mujoco.mjtObj.mjOBJ_SITE:
            continue
        site_id = int(model.sensor_objid[i])
        site_name = model.site(site_id).name
        if model.sensor_type[i] == mujoco.mjtSensor.mjSENS_GYRO:
            site_imu_map.setdefault(site_name, {})['gyro_id'] = i
            site_imu_map[site_name]['site_id'] = site_id
        elif model.sensor_type[i] == mujoco.mjtSensor.mjSENS_ACCELEROMETER:
            site_imu_map.setdefault(site_name, {})['acc_id'] = i
            site_imu_map[site_name]['site_id'] = site_id
    return {name: ids for name, ids in site_imu_map.items()
            if 'gyro_id' in ids and 'acc_id' in ids}


def read_imu(model, data, ids) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Orientation (w x y z, site in world), angular velocity and linear acceleration (site
    frame) of a discover_imus entry."""
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[ids['site_id']])
    acc, gyro = model.sensor_adr[ids['acc_id']], model.sensor_adr[ids['gyro_id']]
    return quat, data.sensordata[gyro:gyro + 3].copy(), data.sensordata[acc:acc + 3].copy()


@dataclass(frozen=True)
class ImuCfg:
    site: str            # an IMU of discover_imus: frame, orientation and sensors
    topic: str = '/imu'
    frame_id: str | None = None
    rate: float = 200.0

    def __post_init__(self):
        if not self.site:
            raise ValueError('IMU site is required')
        check_topic(self.topic, 'IMU')
        check_rate(self.rate, 'IMU')
        frame = self.frame_id if self.frame_id is not None else self.site
        check_frame(frame, 'IMU')
        object.__setattr__(self, 'frame_id', frame)

    @classmethod
    def from_xml(cls, element):
        allowed = {'site', 'topic', 'frame_id', 'rate'}
        if set(element.attrib) - allowed or len(element):
            raise ValueError('Unknown imu attributes or child elements')
        kwargs = {k: element.get(k) for k in ('site', 'topic', 'frame_id')
                  if element.get(k) is not None}
        if element.get('rate') is not None:
            kwargs['rate'] = float(element.get('rate'))
        return cls(**{'site': '', **kwargs})


def validate_imus(model, configs):
    imus = discover_imus(model)
    for cfg in configs:
        if not isinstance(cfg, ImuCfg):
            raise TypeError('imus must contain ImuCfg instances')
        if cfg.site not in imus:
            raise ValueError(f'IMU site {cfg.site} has no gyro and accelerometer')
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
        self._ids = discover_imus(model)[cfg.site]
        self._schedule = RateSchedule(cfg.rate, model.opt.timestep)

    def step(self, data) -> ImuSample | None:
        time = float(data.time) - self._model.opt.timestep
        if not self._schedule.due(time):
            return None
        return ImuSample(time, *read_imu(self._model, data, self._ids))
