"""ROS-independent RGB-D capture with bounded, simulation-time scheduling."""

from collections import OrderedDict
from copy import copy, deepcopy
from dataclasses import dataclass
import math
import re
import threading

import mujoco
import numpy as np


@dataclass(frozen=True)
class RgbdCameraCfg:
    camera: str
    rate: float = 30.0
    topic: str | None = None
    frame_id: str | None = None
    geom_groups: tuple[int, ...] = (0, 2)

    def __post_init__(self):
        if not isinstance(self.camera, str) or not self.camera.strip():
            raise ValueError('RGB-D camera name is required')
        if isinstance(self.rate, bool) or not math.isfinite(self.rate) or self.rate <= 0:
            raise ValueError('RGB-D rate must be finite and positive')
        topic = self.topic if self.topic is not None else '/' + self.camera
        frame = self.frame_id if self.frame_id is not None else self.camera + '_optical_frame'
        topic = topic.rstrip('/')
        if not re.fullmatch(r'/?[A-Za-z_][A-Za-z_0-9]*(/[A-Za-z_][A-Za-z_0-9]*)*', topic):
            raise ValueError('RGB-D topic must be a valid ROS topic prefix')
        if not frame or frame.startswith('/') or any(c.isspace() for c in frame):
            raise ValueError('RGB-D frame_id must be nonempty without a leading slash')
        groups = tuple(self.geom_groups)
        if not groups or any(isinstance(g, bool) or not isinstance(g, int) or not 0 <= g < 6 for g in groups):
            raise ValueError('RGB-D geom_groups must contain integers from 0 to 5')
        object.__setattr__(self, 'topic', topic)
        object.__setattr__(self, 'frame_id', frame)
        object.__setattr__(self, 'geom_groups', groups)

    @classmethod
    def from_xml(cls, element):
        allowed = {'camera', 'rate', 'topic', 'frame_id', 'geom_groups'}
        if set(element.attrib) - allowed or len(element):
            raise ValueError('Unknown RGB-D attributes or child elements')
        return cls(
            camera=element.get('camera', ''), rate=float(element.get('rate', '30')),
            topic=element.get('topic'), frame_id=element.get('frame_id'),
            geom_groups=tuple(int(g) for g in element.get('geom_groups', '0 2').split()),
        )


def validate_cameras(model, configs):
    """Resolve cameras and reject ambiguity before any render thread is created."""
    seen = [set(), set(), set()]
    for cfg in configs:
        if not isinstance(cfg, RgbdCameraCfg):
            raise TypeError('rgbd_cameras must contain RgbdCameraCfg instances')
        for value, values in zip((cfg.camera, cfg.topic.lstrip('/'), cfg.frame_id), seen):
            if value in values:
                raise ValueError(f'Duplicate RGB-D camera, topic or frame: {value}')
            values.add(value)
        camera_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cfg.camera)
        if camera_id < 0:
            raise ValueError(f'Unknown RGB-D camera: {cfg.camera}')
        if model.cam_projection[camera_id] != 0:
            raise ValueError(f'RGB-D camera {cfg.camera} must use perspective projection')
        resolution = model.cam_resolution[camera_id]
        if not (np.all(resolution == 1) or np.all(resolution > 1)):
            raise ValueError(f'RGB-D camera {cfg.camera} needs both resolution dimensions > 1')
        parent_id = int(model.cam_bodyid[camera_id])
        parent = model.body(parent_id).name if parent_id else 'world'
        if not parent or cfg.frame_id == parent:
            raise ValueError(f'RGB-D camera {cfg.camera} needs a named parent distinct from its optical frame')


@dataclass(frozen=True)
class RgbdFrame:
    camera: str
    time: float
    rgb: np.ndarray
    depth: np.ndarray
    intrinsic: np.ndarray
    position: np.ndarray
    rotation: np.ndarray
    parent_frame: str
    parent_position: np.ndarray
    parent_rotation: np.ndarray


def calibration_from_scene(scene, width, height):
    """Intrinsics of MuJoCo's mono projection, with integer pixel centers."""
    camera = scene.camera[0]
    near = camera.frustum_near
    bottom, top = camera.frustum_bottom, camera.frustum_top
    halfwidth = camera.frustum_width or (top - bottom) * width / height / 2
    fx = width * near / (2 * halfwidth)
    fy = height * near / (top - bottom)
    cx = (width - 1) / 2 - width * camera.frustum_center / (2 * halfwidth)
    cy = (height - 1) / 2 + height * (top + bottom) / (2 * (top - bottom))
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=float)


class _Capture:
    """Created, used and closed exclusively by the rendering thread."""

    def __init__(self, model, configs):
        self.model = model
        self.data = mujoco.MjData(model)
        self.renderers = {}
        for cfg in configs:
            camera_id = model.camera(cfg.camera).id
            if np.all(model.cam_resolution[camera_id] == 1):
                model.cam_resolution[camera_id] = (640, 480)
        resolutions = [model.cam_resolution[model.camera(cfg.camera).id] for cfg in configs]
        model.vis.global_.offwidth = max(model.vis.global_.offwidth, max(int(r[0]) for r in resolutions))
        model.vis.global_.offheight = max(model.vis.global_.offheight, max(int(r[1]) for r in resolutions))
        self.configs = {cfg.camera: cfg for cfg in configs}

    def render(self, name, state, state_spec):
        m, d = self.model, self.data
        mujoco.mj_setState(m, d, state, state_spec)
        mujoco.mj_forward(m, d)
        cfg = self.configs[name]
        camera_id = m.camera(name).id
        width, height = (int(v) for v in m.cam_resolution[camera_id])
        key = (width, height)
        if key not in self.renderers:
            self.renderers[key] = mujoco.Renderer(m, width=width, height=height)
        renderer = self.renderers[key]
        option = mujoco.MjvOption()
        option.geomgroup[:] = 0
        option.geomgroup[list(cfg.geom_groups)] = 1
        renderer.disable_depth_rendering()
        renderer.update_scene(d, camera=name, scene_option=option)
        intrinsic = calibration_from_scene(renderer.scene, width, height)
        rgb = renderer.render()
        renderer.enable_depth_rendering()
        try:
            depth = renderer.render()
        finally:
            renderer.disable_depth_rendering()
        far = m.stat.extent * m.vis.map.zfar
        depth[(~np.isfinite(depth)) | (depth >= far * (1 - 1e-6)) | (depth <= 0)] = np.nan
        parent_id = int(m.cam_bodyid[camera_id])
        parent_rotation = d.xmat[parent_id].reshape(3, 3)
        rotation = d.cam_xmat[camera_id].reshape(3, 3) @ np.diag([1., -1., -1.])
        position = d.cam_xpos[camera_id].copy()
        frame = RgbdFrame(
            name, float(d.time), rgb, depth, intrinsic, position, rotation,
            m.body(parent_id).name if parent_id else 'world',
            parent_rotation.T @ (position - d.xpos[parent_id]),
            parent_rotation.T @ rotation,
        )
        for value in vars(frame).values():
            if isinstance(value, np.ndarray):
                value.flags.writeable = False
        return frame

    def close(self):
        for renderer in self.renderers.values():
            renderer.close()
        self.renderers.clear()


class RgbdCameraManager:
    """One GL worker; at most one pending snapshot and result per camera.

    Call submit/reset/poll from the physics thread. latest_frames is safe to read
    from any thread and returns independent copies. Model changes after startup
    are not propagated to the worker's private model.
    """

    def __init__(self, model, configs, *, capture_factory=_Capture):
        self.configs = tuple(configs)
        validate_cameras(model, self.configs)
        self._source_model = model
        self._model = copy(model) if self.configs else None
        self._state_spec = mujoco.mjtState.mjSTATE_INTEGRATION
        self._state_size = mujoco.mj_stateSize(model, self._state_spec)
        self._condition = threading.Condition()
        self._pending = OrderedDict()
        self._completed = {}
        self._latest = {}
        self._deadlines = {cfg.camera: 0.0 for cfg in self.configs}
        self._epoch = 0
        self._last_time = None
        self._closed = False
        self._error = None
        self._capture_factory = capture_factory
        self._thread = None
        if self.configs:
            self._thread = threading.Thread(target=self._run, name='mujoco-rgbd', daemon=True)
            self._thread.start()

    def check_error(self):
        with self._condition:
            error = self._error
        if error is not None:
            raise RuntimeError('RGB-D rendering worker failed') from error

    def submit(self, data):
        self.check_error()
        now = float(data.time)
        if self._last_time is not None and now < self._last_time:
            self.reset()
        self._last_time = now
        due = [cfg for cfg in self.configs if now + 1e-12 >= self._deadlines[cfg.camera]]
        if not due:
            return
        state = np.empty(self._state_size)
        mujoco.mj_getState(self._source_model, data, state, self._state_spec)
        state.flags.writeable = False
        with self._condition:
            if self._closed:
                return
            for cfg in due:
                self._pending[cfg.camera] = (self._epoch, state)
                deadline = self._deadlines[cfg.camera]
                self._deadlines[cfg.camera] = deadline + (math.floor((now - deadline) * cfg.rate + 1e-9) + 1) / cfg.rate
            self._condition.notify()

    def reset(self):
        with self._condition:
            self._epoch += 1
            self._pending.clear()
            self._completed.clear()
            self._latest.clear()
            self._deadlines = {cfg.camera: 0.0 for cfg in self.configs}
            self._last_time = None

    def poll(self):
        self.check_error()
        with self._condition:
            frames = list(self._completed.values())
            self._completed.clear()
        return frames

    @property
    def latest_frames(self):
        self.check_error()
        with self._condition:
            latest = dict(self._latest)
        return deepcopy(latest)

    def _run(self):
        capture = None
        try:
            capture = self._capture_factory(self._model, self.configs)
            while True:
                with self._condition:
                    self._condition.wait_for(lambda: self._closed or self._pending)
                    if self._closed:
                        break
                    name, (epoch, state) = self._pending.popitem(last=False)
                frame = capture.render(name, state, self._state_spec)
                with self._condition:
                    if epoch == self._epoch and not self._closed:
                        self._completed[name] = frame
                        self._latest[name] = frame
        except Exception as error:
            with self._condition:
                self._error = error
                self._condition.notify_all()
        finally:
            if capture is not None:
                try:
                    capture.close()
                except Exception as error:
                    with self._condition:
                        self._error = self._error or error

    def close(self):
        with self._condition:
            self._closed = True
            self._pending.clear()
            self._condition.notify_all()
        if self._thread is not None:
            self._thread.join()
