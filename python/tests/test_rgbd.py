from collections import deque
from pathlib import Path
from threading import Event, get_ident
import time

from lxml import etree
import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco.mjcf_tools import MjcfGenerator
from xbot2_py_mujoco.rgbd import RgbdCameraCfg, RgbdCameraManager, validate_cameras


def model():
    return mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="mount" mocap="true"><camera name="front" resolution="160 120"/>
        <camera name="rear" resolution="160 120"/></body>
    </worldbody></mujoco>''')


def wait_for(predicate):
    deadline = time.monotonic() + 5
    while not predicate():
        if time.monotonic() > deadline:
            pytest.fail('RGB-D worker did not finish')
        time.sleep(.005)


@pytest.fixture
def generator(tmp_path):
    return MjcfGenerator('rgbd', (Path(__file__).parent / 'resources/nice_robot.urdf').read_text(),
                         output_dir=str(tmp_path / 'robot'))


def test_xml_reference_defaults_merge_and_repeated_finalization(generator):
    generator.merge_xml('''<mujoco>
      <reference element="body" name="base_link"><camera name="front" resolution="320 240"/></reference>
      <rgbd camera="front"/>
    </mujoco>''')
    # Use an existing body independent of the fixture robot's naming.
    ref = generator.mj_xml_tree.find('reference')
    ref.set('name', generator.mj_xml_tree.find('worldbody/body').get('name'))
    generator.merge_xml('''<mujoco><worldbody><camera name="rear"/></worldbody>
      <rgbd camera="rear" rate="15" topic="/back" frame_id="back_optical" geom_groups="0 2 3"/>
    </mujoco>''')
    xml = generator.generate_mjcf_string()
    assert '<rgbd' not in xml
    assert mujoco.MjModel.from_xml_string(xml).ncam == 2
    assert generator.rgbd_cameras == [RgbdCameraCfg('front'), RgbdCameraCfg('rear', 15, '/back', 'back_optical', (0, 2, 3))]
    assert generator.generate_mjcf_string() == xml
    assert len(generator.rgbd_cameras) == 2


@pytest.mark.parametrize('attributes', [
    '', 'camera="front" rate="0"', 'camera="front" rate="nan"',
    'camera="front" rate="inf"', 'camera="front" geom_groups="6"',
    'camera="front" geom_groups=""', 'camera="front" extra="1"',
    'camera="front" frame_id="/optical"', 'camera="front" topic="/"',
])
def test_invalid_xml_settings(attributes):
    with pytest.raises((ValueError, TypeError)):
        RgbdCameraCfg.from_xml(etree.fromstring(f'<rgbd {attributes}/>'))


@pytest.mark.parametrize('settings', [
    '<rgbd camera="missing"/>',
    '<rgbd camera="front"/><rgbd camera="front"/>',
    '<rgbd camera="front" topic="/image"/><rgbd camera="rear" topic="image/"/>',
    '<rgbd camera="front" frame_id="optical"/><rgbd camera="rear" frame_id="optical"/>',
])
def test_invalid_camera_references_and_duplicates(generator, settings):
    generator.merge_xml('<mujoco><worldbody><camera name="front"/><camera name="rear"/></worldbody>' + settings + '</mujoco>')
    with pytest.raises(ValueError):
        generator.generate_mjcf_string()


@pytest.mark.parametrize('attributes', ['projection="orthographic"', 'resolution="1 20"'])
def test_invalid_projection_or_resolution(attributes):
    m = mujoco.MjModel.from_xml_string(f'<mujoco><worldbody><camera name="front" {attributes}/></worldbody></mujoco>')
    with pytest.raises(ValueError):
        validate_cameras(m, [RgbdCameraCfg('front')])


def test_no_cameras_creates_no_worker_or_model_copy():
    manager = RgbdCameraManager(model(), [])
    assert manager._thread is None and manager._model is None
    manager.close()


class FakeCapture:
    def __init__(self, m, configs):
        self.model = m
        self.started = Event()
        self.release = Event()
        self.calls = deque()
        self.closed = False
        self.owner = get_ident()

    def render(self, name, state, spec):
        assert get_ident() == self.owner
        self.started.set()
        assert self.release.wait(5)
        data = mujoco.MjData(self.model)
        mujoco.mj_setState(self.model, data, state, spec)
        frame = (name, data.time, data.mocap_pos.copy())
        self.calls.append(frame)
        return frame

    def close(self):
        assert get_ident() == self.owner
        self.closed = True


def test_worker_latest_snapshot_fairness_rates_reset_and_cleanup():
    m = model()
    data = mujoco.MjData(m)
    made = []
    def factory(*args):
        capture = FakeCapture(*args)
        made.append(capture)
        return capture
    manager = RgbdCameraManager(m, [RgbdCameraCfg('front', 10), RgbdCameraCfg('rear', 5)], capture_factory=factory)
    try:
        manager.submit(data)
        wait_for(lambda: made and made[0].started.is_set())
        capture = made[0]
        assert capture.owner != get_ident() and capture.model is not m
        for tick in range(1, 11):
            data.time = tick / 10
            data.mocap_pos[:] = tick
            manager.submit(data)
        assert len(manager._pending) == 2
        # In-place replacement preserves the rear camera's place in the queue.
        assert list(manager._pending) == ['rear', 'front']
        data.mocap_pos[:] = 999
        capture.release.set()
        wait_for(lambda: len(capture.calls) == 3)
        assert [f[0] for f in capture.calls] == ['front', 'rear', 'front']
        assert capture.calls[1][1] == 1.0
        np.testing.assert_array_equal(capture.calls[1][2], 10)
        assert len(manager.poll()) == 2
        latest = manager.latest_frames
        latest['front'][2][:] = -100
        assert np.all(manager.latest_frames['front'][2] == 10)
        manager.reset()
        assert manager.latest_frames == {} and manager.poll() == []
        # Backward time and rate deadlines are based on simulation time.
        data.time = .01
        manager.submit(data)
        wait_for(lambda: len(capture.calls) == 5)
        count = len(capture.calls)
        manager.submit(data)
        time.sleep(.02)
        assert len(capture.calls) == count
    finally:
        if made:
            made[0].release.set()
        manager.close()
    assert made[0].closed and not manager._thread.is_alive()
    manager.close()


def test_reset_discards_inflight_results():
    m = model()
    made = []
    def factory(*args):
        capture = FakeCapture(*args)
        made.append(capture)
        return capture
    manager = RgbdCameraManager(m, [RgbdCameraCfg('front')], capture_factory=factory)
    try:
        manager.submit(mujoco.MjData(m))
        wait_for(lambda: made and made[0].started.is_set())
        manager.reset()
        made[0].release.set()
        wait_for(lambda: made[0].calls)
        assert manager.poll() == [] and manager.latest_frames == {}
    finally:
        if made:
            made[0].release.set()
        manager.close()


def test_worker_failure_surfaces_with_cause_and_closes():
    made = []
    class BrokenCapture(FakeCapture):
        def render(self, *args):
            raise ValueError('render failed')
    def factory(*args):
        capture = BrokenCapture(*args)
        made.append(capture)
        return capture
    m = model()
    manager = RgbdCameraManager(m, [RgbdCameraCfg('front')], capture_factory=factory)
    try:
        manager.submit(mujoco.MjData(m))
        wait_for(lambda: manager._error is not None)
        with pytest.raises(RuntimeError, match='RGB-D') as error:
            manager.poll()
        assert isinstance(error.value.__cause__, ValueError)
    finally:
        manager.close()
    assert made[0].closed
