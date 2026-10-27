"""ROS-independent spinning lidar on a site, ray traced with mujoco-lidar."""

from dataclasses import dataclass
import math
import re

import mujoco
import numpy as np


BACKENDS = ('cpu', 'taichi', 'warp')


@dataclass(frozen=True)
class LidarScan:
    lidar: str
    time: float         # sweep start: the time of its first slice
    points: np.ndarray  # (columns * returns * rings, 3) in the site frame, ring fastest; misses 0
    rings: np.ndarray   # ring index of each point
    times: np.ndarray   # sim time of the pose each point was traced at


@dataclass(frozen=True)
class LidarCfg:
    site: str
    elevations_deg: tuple[float, ...]
    rate: float = 10.0
    topic: str = '/lidar_points'
    frame_id: str | None = None
    backend: str = 'cpu'
    cutoff: float = 50.0
    min_range: float = 0.05
    horizontal_samples: int = 900
    start_azimuth_deg: float = 0.0
    geom_groups: tuple[int, ...] = (0, 2)
    exclude_body: str | None = None  # e.g. the body enclosing the sensor
    slices: int = 20  # traces per sweep; columns of a slice share one pose and time
    returns: int = 1  # each traced column written this many times (2: a dual return driver's)

    def __post_init__(self):
        if not isinstance(self.site, str) or not self.site.strip():
            raise ValueError('Lidar site name is required')
        elevations = tuple(float(e) for e in self.elevations_deg)
        if not elevations or any(not math.isfinite(e) or abs(e) > 90 for e in elevations):
            raise ValueError('Lidar elevations_deg must be finite angles within [-90, 90]')
        for name in ('rate', 'cutoff'):
            value = getattr(self, name)
            if isinstance(value, bool) or not math.isfinite(value) or value <= 0:
                raise ValueError(f'Lidar {name} must be finite and positive')
        if not math.isfinite(self.min_range) or not 0 <= self.min_range < self.cutoff:
            raise ValueError('Lidar min_range must be in [0, cutoff)')
        if (isinstance(self.horizontal_samples, bool) or not isinstance(self.horizontal_samples, int)
                or self.horizontal_samples <= 0):
            raise ValueError('Lidar horizontal_samples must be a positive integer')
        if (isinstance(self.slices, bool) or not isinstance(self.slices, int)
                or not 0 < self.slices <= self.horizontal_samples):
            raise ValueError('Lidar slices must be an integer in [1, horizontal_samples]')
        if isinstance(self.returns, bool) or not isinstance(self.returns, int) or self.returns < 1:
            raise ValueError('Lidar returns must be a positive integer')
        if not math.isfinite(self.start_azimuth_deg):
            raise ValueError('Lidar start_azimuth_deg must be finite')
        if self.backend not in BACKENDS:
            raise ValueError(f'Lidar backend must be one of {", ".join(BACKENDS)}')
        if not re.fullmatch(r'/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*', self.topic):
            raise ValueError('Lidar topic must be a valid ROS topic')
        frame = self.frame_id if self.frame_id is not None else self.site
        if not frame or frame.startswith('/') or any(c.isspace() for c in frame):
            raise ValueError('Lidar frame_id must be nonempty without a leading slash')
        groups = tuple(self.geom_groups)
        if not groups or any(isinstance(g, bool) or not isinstance(g, int) or not 0 <= g < 6 for g in groups):
            raise ValueError('Lidar geom_groups must contain integers from 0 to 5')
        object.__setattr__(self, 'elevations_deg', elevations)
        object.__setattr__(self, 'frame_id', frame)
        object.__setattr__(self, 'geom_groups', groups)

    @classmethod
    def from_xml(cls, element):
        allowed = {'site', 'elevations_deg', 'rate', 'topic', 'frame_id', 'backend', 'cutoff',
                   'min_range', 'horizontal_samples', 'start_azimuth_deg', 'geom_groups',
                   'exclude_body', 'slices', 'returns'}
        if set(element.attrib) - allowed or len(element):
            raise ValueError('Unknown lidar attributes or child elements')
        optional = {}
        for name, convert in (('rate', float), ('topic', str), ('frame_id', str),
                              ('backend', str), ('cutoff', float), ('min_range', float),
                              ('horizontal_samples', int), ('start_azimuth_deg', float),
                              ('exclude_body', str), ('slices', int), ('returns', int)):
            if element.get(name) is not None:
                optional[name] = convert(element.get(name))
        if element.get('geom_groups') is not None:
            optional['geom_groups'] = tuple(int(g) for g in element.get('geom_groups').split())
        return cls(site=element.get('site', ''),
                   elevations_deg=tuple(float(e) for e in element.get('elevations_deg', '').split()),
                   **optional)


def validate_lidars(model, configs):
    seen = [set(), set()]
    for cfg in configs:
        if not isinstance(cfg, LidarCfg):
            raise TypeError('lidars must contain LidarCfg instances')
        for value, values in zip((cfg.site, cfg.topic.lstrip('/')), seen):
            if value in values:
                raise ValueError(f'Duplicate lidar site or topic: {value}')
            values.add(value)
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, cfg.site) < 0:
            raise ValueError(f'Unknown lidar site: {cfg.site}')
        if (cfg.exclude_body is not None
                and mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, cfg.exclude_body) < 0):
            raise ValueError(f'Unknown lidar exclude_body: {cfg.exclude_body}')


class Lidar:
    """Sweeps of columns x rings rays: columns clockwise from start_azimuth_deg, ring fastest,
    one sweep per 1/rate s. Each slice of columns is traced at the robot's pose when it is due:
    motion distortion. Call step after every mj_step, from the physics thread."""

    def __init__(self, model, cfg: LidarCfg):
        validate_lidars(model, [cfg])
        from mujoco_lidar import MjLidarWrapper
        self.cfg = cfg
        self._model = model
        rings = len(cfg.elevations_deg)
        azimuth = (math.radians(cfg.start_azimuth_deg)
                   - np.arange(cfg.horizontal_samples) * 2 * math.pi / cfg.horizontal_samples)
        theta = np.repeat(azimuth, rings)
        phi = np.tile(np.radians(cfg.elevations_deg), cfg.horizontal_samples)
        self._theta = theta.astype(np.float32)
        self._phi = phi.astype(np.float32)
        self._directions = np.stack((np.cos(phi) * np.cos(theta), np.cos(phi) * np.sin(theta),
                                     np.sin(phi)), axis=-1)
        self._rings = np.tile(np.arange(rings, dtype=np.uint16), cfg.horizontal_samples * cfg.returns)
        # Ray index where each slice starts, and the end of the last one.
        self._slice_rays = [rings * (k * cfg.horizontal_samples // cfg.slices)
                            for k in range(cfg.slices + 1)]
        geomgroup = np.zeros(mujoco.mjNGROUP, dtype=np.uint8)
        geomgroup[list(cfg.geom_groups)] = 1
        exclude = -1 if cfg.exclude_body is None else model.body(cfg.exclude_body).id
        self._wrapper = MjLidarWrapper(model, cfg.site, backend=cfg.backend, cutoff_dist=cfg.cutoff,
                                       args={'geomgroup': geomgroup, 'bodyexclude': exclude})
        self._points = np.zeros((len(theta), 3), dtype=np.float32)
        self._times = np.zeros(len(theta))
        self._sweep_start = None
        self._next_slice = 0
        self._last_time = None

    def step(self, data) -> LidarScan | None:
        """Trace the slices due by now; the sweep once its last slice is traced."""
        # After mj_step, poses are those of the step's start.
        time = float(data.time) - self._model.opt.timestep
        # Sim time is a sum of steps: a slice is due at the step nearest its time.
        tolerance = self._model.opt.timestep / 2
        period = 1.0 / self.cfg.rate
        if (self._sweep_start is None or time < self._last_time
                or time >= self._sweep_start + period):
            self._sweep_start = time
            self._next_slice = 0
        self._last_time = time
        due = min(self.cfg.slices,
                  math.floor((time + tolerance - self._sweep_start) * self.cfg.slices / period) + 1)
        if due <= self._next_slice:
            return None
        a, b = self._slice_rays[self._next_slice], self._slice_rays[due]
        self._next_slice = due
        distances = np.asarray(self._wrapper.trace_rays(data, self._theta[a:b], self._phi[a:b]),
                               dtype=np.float64)
        hit = (distances >= self.cfg.min_range) & (distances < self.cfg.cutoff)
        self._points[a:b] = np.where(hit[:, None], self._directions[a:b] * distances[:, None], 0.0)
        self._times[a:b] = time
        if due < self.cfg.slices:
            return None
        scan = LidarScan(self.cfg.site, float(self._times[0]), self._columns(self._points),
                         self._rings, self._columns(self._times))
        self._sweep_start += period
        self._next_slice = 0
        return scan

    def _columns(self, per_ray):
        """Each column (its rings) repeated `returns` times, the copies next to it."""
        columns = per_ray.reshape(self.cfg.horizontal_samples, -1, *per_ray.shape[1:])
        return np.repeat(columns, self.cfg.returns, axis=0).reshape(-1, *per_ray.shape[1:])
