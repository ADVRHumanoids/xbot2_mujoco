"""Exercise native rendering: array assertions cannot detect collapsed plot areas."""
import os
import subprocess
import sys

import pytest


def test_large_font_figures_have_visible_curves():
    script = r''' 
import mujoco
import numpy as np
from xbot2_py_mujoco.base_velocity_plot import BaseVelocityPlot
model = mujoco.MjModel.from_xml_string("""<mujoco>
  <visual><global offwidth="2208" offheight="1840"/></visual>
  <worldbody><body name="pelvis"><freejoint/><geom size=".1"/></body></worldbody>
</mujoco>""")
try:
    gl = mujoco.GLContext(2208, 1840)
    gl.make_current()
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_250)
except Exception as exc:
    print(f'EGL rendering unavailable: {exc}')
    raise SystemExit(77)
try:
    mujoco.mjr_setBuffer(mujoco.mjtFramebuffer.mjFB_OFFSCREEN, context)
    data = mujoco.MjData(model)
    plot = BaseVelocityPlot(model, 'pelvis')
    for i in range(200):
        data.time = i * .05
        data.qvel[:] = [np.sin(data.time), np.cos(data.time), .2,
                        .5 * np.sin(2 * data.time), .4 * np.cos(data.time), -.2]
        plot.sample(data)
    for rect, figure in plot.overlays(mujoco.MjrRect(0, 0, 2208, 1840)):
        mujoco.mjr_figure(rect, figure, context)
        assert figure.xaxispixel[1] - figure.xaxispixel[0] > 300
        assert figure.yaxispixel[1] - figure.yaxispixel[0] > 150
        pixels = np.empty((rect.height, rect.width, 3), dtype=np.uint8)
        mujoco.mjr_readPixels(pixels, None, rect, context)
        x0, x1 = figure.xaxispixel - rect.left
        y0, y1 = figure.yaxispixel - rect.bottom
        pane = pixels[y0:y1, x0:x1].astype(int)
        for axis in range(3):
            other = np.delete(pane, axis, axis=2).max(axis=2)
            assert np.count_nonzero(pane[:, :, axis] - other > 70) > 100
finally:
    context.free()
    gl.free()
'''
    result = subprocess.run([sys.executable, '-c', script],
                            env={**os.environ, 'MUJOCO_GL': 'egl'},
                            capture_output=True, text=True, timeout=30)
    if result.returncode == 77:
        pytest.skip(result.stdout)
    assert result.returncode == 0, result.stdout + result.stderr
