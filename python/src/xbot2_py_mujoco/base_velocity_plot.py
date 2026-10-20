"""Rolling link-frame velocity figures for the passive viewer."""
from collections import deque

import mujoco
import numpy as np


class BaseVelocityPlot:
    window = 10.0
    max_samples = 1000

    def __init__(self, model, body_name):
        self.model = model
        self.body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        if self.body_id <= 0:
            raise ValueError(f"Base velocity plot body {body_name!r} is not a robot body")
        self.samples = deque(maxlen=self.max_samples)
        self.figures = []
        for quantity, unit, prefix in [('Linear', 'm/s', 'v'), ('Angular', 'rad/s', 'w')]:
            figure = mujoco.MjvFigure()
            figure.title = f'{quantity} ({unit}), base frame'
            figure.xlabel = 'Time (s)'
            figure.gridsize[:] = (5, 5)
            figure.xformat = '%.1f'
            figure.flg_extend = 0
            figure.flg_legend = 1
            for axis, name in enumerate('xyz'):
                figure.linename[axis] = f'{prefix}{name}'.encode()
            figure.linergb[:3] = [(1, .3, .3), (.1, 1, .1), (.3, .5, 1)]
            self.figures.append(figure)

    def clear(self):
        self.samples.clear()
        for figure in self.figures:
            figure.linepnt[:] = 0

    def sample(self, data):
        if self.samples and data.time < self.samples[-1][0]:
            self.clear()
        if self.samples and data.time == self.samples[-1][0]:
            return
        # mj_step integrates qpos/qvel after computing derived velocities.
        # Refresh only kinematics so the sample matches the current timestamp.
        mujoco.mj_kinematics(self.model, data)
        mujoco.mj_comPos(self.model, data)
        mujoco.mj_comVel(self.model, data)
        velocity = np.empty(6)
        mujoco.mj_objectVelocity(self.model, data, mujoco.mjtObj.mjOBJ_XBODY,
                                self.body_id, velocity, 1)
        self.samples.append((float(data.time), velocity))
        while self.samples[0][0] < data.time - self.window:
            self.samples.popleft()

    def overlays(self, viewport):
        if viewport is None or viewport.width <= 0 or viewport.height <= 0:
            return []
        width = min(800, max(1, viewport.width // 2))
        # Leave room for the native renderer's large-font title, axes and legend.
        height = min(450, max(1, viewport.height // 2))
        left = viewport.left + viewport.width - width
        rows = list(self.samples)
        if rows:
            times = np.array([row[0] for row in rows])
            velocities = np.array([row[1] for row in rows])
            for figure, start in zip(self.figures, (3, 0)):
                values = velocities[:, start:start + 3]
                figure.range[0] = (max(0, times[-1] - self.window),
                                   max(self.window, times[-1]))
                low, high = min(0., values.min()), max(0., values.max())
                padding = max(.05, .1 * (high - low))
                figure.range[1] = (low - padding, high + padding)
                figure.linepnt[:3] = len(rows)
                for axis in range(3):
                    figure.linedata[axis, :2 * len(rows):2] = times
                    figure.linedata[axis, 1:2 * len(rows):2] = values[:, axis]
        return [(mujoco.MjrRect(left, viewport.bottom + viewport.height - (i + 1) * height,
                               width, height), figure)
                for i, figure in enumerate(self.figures)]
