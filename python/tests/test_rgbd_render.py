"""Run with MUJOCO_GL=egl for headless rendering verification."""
import os
import time

import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco.rgbd import RgbdCameraCfg, RgbdCameraManager, _Capture

pytestmark = pytest.mark.skipif(
    not os.environ.get('MUJOCO_GL') and not os.environ.get('DISPLAY'),
    reason='Rendering requires MUJOCO_GL=egl (or a working display)',
)


def capture_model(intrinsic='fovy="60"', mounted=False):
    camera = f'<camera name="front" resolution="160 120" {intrinsic}/>'
    if mounted:
        camera = '<body name="mount" mocap="true">' + camera + '</body>'
    return mujoco.MjModel.from_xml_string('''<mujoco>
      <visual><headlight ambient="1 1 1" diffuse="0 0 0"/></visual>
      <worldbody>''' + camera + '''
        <geom name="target" type="box" pos="0 0 -2" size=".04 .04 .02" rgba="1 0 0 1" group="2"/>
      </worldbody></mujoco>''')


def render(m, d=None, cfg=None):
    d = d or mujoco.MjData(m)
    d.time = 1.25
    spec = mujoco.mjtState.mjSTATE_INTEGRATION
    state = np.empty(mujoco.mj_stateSize(m, spec))
    mujoco.mj_getState(m, d, state, spec)
    capture = _Capture(m, [cfg or RgbdCameraCfg('front')])
    try:
        return capture.render('front', state, spec)
    finally:
        capture.close()


@pytest.mark.parametrize('intrinsic, expected', [
    ('fovy="60"', (120 / (2 * np.tan(np.pi / 6)),) * 2 + (79.5, 59.5)),
    ('sensorsize=".16 .12" focalpixel="130 135" principalpixel="8 4"', (130, 135, 71.5, 55.5)),
])
def test_actual_projection_depth_alignment_and_pixel_conventions(intrinsic, expected):
    frame = render(capture_model(intrinsic))
    assert frame.time == 1.25
    assert frame.rgb.shape == (120, 160, 3) and frame.rgb.dtype == np.uint8
    assert frame.depth.shape == (120, 160) and frame.depth.dtype == np.float32
    fx, fy, cx, cy = expected
    np.testing.assert_allclose(frame.intrinsic, [[fx, 0, cx], [0, fy, cy], [0, 0, 1]], rtol=1e-6)
    red = frame.rgb[:, :, 0] > 100
    center = np.argwhere(red).mean(axis=0)
    np.testing.assert_allclose(center, [cy, cx], atol=.6)
    assert np.isnan(frame.depth[0, 0])
    assert np.isfinite(frame.depth[red]).any()
    np.testing.assert_allclose(frame.depth[np.isfinite(frame.depth)], 1.98, atol=1e-4)
    np.testing.assert_array_equal(frame.rotation, np.diag([1, -1, -1]))
    assert frame.parent_frame == 'world'


def test_geometry_visibility_and_parent_optical_transform():
    m = capture_model(mounted=True)
    m.cam_pos[0] = [.1, .2, .3]
    d = mujoco.MjData(m)
    # Rotate the mount around its optical Z axis; projection remains centered.
    d.mocap_quat[0] = [np.cos(.3), 0, 0, np.sin(.3)]
    frame = render(m, d)
    mujoco.mj_forward(m, d)
    parent_rotation = d.xmat[m.body('mount').id].reshape(3, 3)
    np.testing.assert_allclose(parent_rotation @ frame.parent_rotation, frame.rotation, atol=1e-12)
    np.testing.assert_allclose(frame.parent_rotation, np.diag([1, -1, -1]), atol=1e-12)
    np.testing.assert_allclose(frame.parent_position, [.1, .2, .3], atol=1e-12)
    assert frame.parent_frame == 'mount'
    hidden = render(capture_model(), cfg=RgbdCameraCfg('front', geom_groups=(0,)))
    assert np.isnan(hidden.depth).all()


def test_rendered_optical_axes_match_projection():
    m = capture_model()
    m.geom_pos[m.geom('target').id] = [.2, .1, -2]
    frame = render(m)
    rows, columns = np.nonzero(frame.rgb[:, :, 0] > 100)
    # World +X is image right; world +Y is image up for this camera.
    expected = frame.intrinsic @ np.array([.2, -.1, 1.98])
    np.testing.assert_allclose([columns.mean(), rows.mean()], expected[:2] / expected[2], atol=.7)


def test_actual_background_worker_multiple_rates_and_independent_frames():
    m = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <camera name="a" resolution="80 60" fovy="60"/>
      <camera name="b" resolution="64 48" fovy="60"/>
      <geom type="plane" pos="0 0 -2" size="10 10 .1" group="2"/>
    </worldbody></mujoco>''')
    original_width = m.vis.global_.offwidth
    manager = RgbdCameraManager(m, [RgbdCameraCfg('a', 20), RgbdCameraCfg('b', 10)])
    d = mujoco.MjData(m)
    try:
        manager.submit(d)
        deadline = time.monotonic() + 10
        while len(manager.latest_frames) < 2:
            if time.monotonic() > deadline:
                pytest.fail('Headless RGB-D worker timed out')
            time.sleep(.01)
        frames = manager.latest_frames
        assert frames['a'].rgb.shape == (60, 80, 3)
        assert frames['b'].rgb.shape == (48, 64, 3)
        d.time = .05
        manager.submit(d)
        while manager.latest_frames['a'].time != .05:
            if time.monotonic() > deadline:
                pytest.fail('Headless RGB-D worker timed out')
            time.sleep(.01)
        assert manager.latest_frames['b'].time == 0
        assert m.vis.global_.offwidth == original_width
    finally:
        manager.close()
    assert not manager._thread.is_alive()


def test_default_resolution_and_private_framebuffer_growth():
    m = mujoco.MjModel.from_xml_string('<mujoco><visual><global offwidth="64" offheight="48"/></visual><worldbody><camera name="front"/></worldbody></mujoco>')
    frame = render(m)
    assert frame.rgb.shape == (480, 640, 3)
