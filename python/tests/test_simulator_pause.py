from contextlib import nullcontext
from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco import simulator_wrapper as wrapper


class Clock:
    def __init__(self):
        self.now = 1.0
        self.sleeps = []

    def perf_counter(self):
        return self.now

    def sleep(self, seconds):
        assert seconds >= 0
        self.sleeps.append(seconds)
        self.now += seconds


class Viewer:
    def __init__(self):
        self.running = True
        self.sync = Mock()
        self.user_scn = SimpleNamespace(ngeom=0)

    def is_running(self):
        return self.running

    def lock(self):
        return nullcontext()

    def close(self):
        self.running = False


@pytest.fixture
def make_sim(monkeypatch):
    clock = Clock()
    monkeypatch.setattr(wrapper, 'time', clock)
    sims = []

    def create(*, viewer=False, callback=None, timestep=0.002, on_launch=None, scanner=False):
        model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
          <geom type="plane" size="10 10 .1" group="2"/>
          <body name="robot" pos="0 0 1"><freejoint/>
            <geom type="box" size=".1 .1 .1" group="3"/>
          </body></worldbody></mujoco>''')
        model.opt.timestep = timestep
        bridge = SimpleNamespace(receive=Mock(), send_state=Mock())
        monkeypatch.setattr(wrapper, 'MjXbot2Bridge', lambda **kwargs: bridge)
        fake_viewer = Viewer()
        keys = []

        def launch(*args, key_callback):
            keys.append(key_callback)
            if on_launch:
                on_launch(key_callback)
            return fake_viewer

        monkeypatch.setattr(wrapper, 'launch_passive', launch)
        terrain = wrapper.TerrainScanner(model, 'robot', shape=(1, 1), excluded_geom_groups=(3,)) if scanner else None
        sim = wrapper.SimulatorWrapper(
            model, ros=False, viewer=viewer, viewer_key_callback=callback,
            terrain_scanner=terrain, terrain_scan_decimation=2,
            send_state_decimation=2,
        )
        sims.append(sim)
        return sim, bridge, fake_viewer, keys, clock

    yield create
    for sim in sims:
        sim.close()


def run_advance(sim, maximum=1000):
    for _ in range(maximum):
        sim.run()
        if not sim._advance_requests and sim._advance_target is None:
            return
    pytest.fail('advance failed to complete')


def test_default_keys_initialized_before_viewer_launch(make_sim):
    sim, bridge, viewer, keys, clock = make_sim(
        viewer=True, on_launch=lambda callback: callback(ord(' ')),
    )
    assert sim.paused
    keys[0](262)
    assert list(sim._advance_requests) == [0.01666]
    keys[0](ord(' '))
    assert not sim.paused
    assert not sim._advance_requests
    keys[0](262)
    assert not sim._advance_requests
    with pytest.raises(AttributeError):
        sim.paused = True


def test_custom_callback_fully_replaces_default(make_sim):
    callback = Mock()
    sim, bridge, viewer, keys, clock = make_sim(viewer=True, callback=callback)
    assert keys[0] is callback
    keys[0](ord(' '))
    keys[0](262)
    keys[0](ord('+'))
    keys[0](ord('-'))
    assert sim.target_rtf == 1.0
    assert not sim.paused
    assert not sim._advance_requests
    assert callback.call_count == 4
    sim.set_paused()
    sim.advance()
    sim.set_paused()  # Idempotence must preserve a queued request.
    assert list(sim._advance_requests) == [0.1]
    sim.set_paused(False)
    sim.set_paused(False)
    assert not sim.paused
    assert not sim._advance_requests


def test_idle_pause_freezes_all_updates_and_renders(make_sim, monkeypatch):
    sim, bridge, viewer, keys, clock = make_sim(viewer=True)
    sim.remote_control = Mock()
    sim.clock_pub = Mock()
    sim.terrain_scanner = Mock()
    sim.terrain_scan_pub = Mock()
    step = Mock()
    monkeypatch.setattr(wrapper.mujoco, 'mj_step', step)
    qpos, qvel = sim.data.qpos.copy(), sim.data.qvel.copy()
    sim.set_paused()
    for _ in range(5):
        sim.pre_step()
        sim.step()
        sim.post_step()
    assert sim.data.time == 0
    assert sim.iter_counter == 0
    assert sim.last_sim_time == 0
    np.testing.assert_array_equal(sim.data.qpos, qpos)
    np.testing.assert_array_equal(sim.data.qvel, qvel)
    step.assert_not_called()
    bridge.receive.assert_not_called()
    bridge.send_state.assert_not_called()
    sim.remote_control.process_requests.assert_not_called()
    sim.remote_control.apply_wrenches.assert_not_called()
    sim.clock_pub.publish.assert_not_called()
    sim.terrain_scanner.scan.assert_not_called()
    sim.terrain_scan_pub.publish.assert_not_called()
    assert viewer.sync.call_count == 5
    assert len(clock.sleeps) == 4
    assert all(abs(delay - 1 / 60) < 1e-10 for delay in clock.sleeps)
    viewer.running = False
    sim.run()
    assert not sim.running


@pytest.mark.parametrize('separate_phases', [False, True])
def test_queued_advances_normal_updates_and_step_resolution(make_sim, separate_phases):
    sim, bridge, viewer, keys, clock = make_sim(timestep=0.007, scanner=True)
    sim.remote_control = Mock()
    sim.set_paused()
    sim.advance()
    sim.advance()
    for _ in range(30):
        if separate_phases:
            sim.pre_step()
            sim.step()
            sim.post_step()
        else:
            sim.run()
        assert sim.paused
    assert abs(sim.data.time - 0.21) < 1e-12
    assert sim.iter_counter == 30
    assert sim.terrain_scan.time == sim.data.time
    assert bridge.receive.call_count == 30
    assert bridge.send_state.call_count == 15
    assert sim.remote_control.process_requests.call_count == 30
    assert sim.remote_control.apply_wrenches.call_count == 30
    sim.run()
    assert sim.iter_counter == 30
    assert bridge.receive.call_count == 30
    assert bridge.send_state.call_count == 15
    assert not sim._advance_requests and sim._advance_target is None


def test_respawn_discarded_during_pause_and_advance(make_sim):
    sim, bridge, viewer, keys, clock = make_sim()
    sim._respawn = Mock()
    sim._viewer_key_callback(ord('R'))
    assert sim._respawn_requested
    sim.set_paused()
    assert not sim._respawn_requested
    sim._viewer_key_callback(ord('r'))
    sim.advance()
    sim.run()
    sim._viewer_key_callback(ord('R'))
    sim.set_paused(False)
    sim.run()
    sim._respawn.assert_not_called()
    sim._viewer_key_callback(ord('R'))
    sim.run()
    sim._respawn.assert_called_once()


def test_resume_and_advance_reset_timing_without_catchup(make_sim):
    sim, bridge, viewer, keys, clock = make_sim()
    sim.set_paused()
    clock.now += 100
    sim.sync_error_integral = 0.1
    sim.advance()
    sim.pre_step()
    assert sim.last_sync_wall == clock.now
    assert sim.last_print_wall == clock.now
    assert sim.sync_error_integral == 0
    sim.step()
    sim.post_step()
    assert clock.sleeps[-1] == pytest.approx(sim.model.opt.timestep)
    sim.advance(0.2)
    sim.set_paused(False)
    assert sim._advance_target is None and not sim._advance_requests
    clock.now += 100
    sim.sync_error_integral = -0.1
    count, start = sim.iter_counter, sim.data.time
    sim.run()
    assert sim.iter_counter == count + 1
    assert sim.data.time == pytest.approx(start + sim.model.opt.timestep)
    assert sim.sync_error_integral == 0
    assert clock.sleeps[-1] == pytest.approx(sim.model.opt.timestep)


@pytest.mark.parametrize('duration', [0, -1, float('nan'), float('inf')])
def test_advance_rejects_invalid_duration(make_sim, duration):
    sim, *_ = make_sim()
    with pytest.raises(ValueError, match='finite and positive'):
        sim.advance(duration)


def test_advance_ros_publications(make_sim):
    pytest.importorskip('sensor_msgs.msg')
    sim, bridge, viewer, keys, clock = make_sim(scanner=True)
    sim.clock_pub = Mock()
    sim.terrain_scan_pub = Mock()
    sim.set_paused()
    sim.advance(0.01)
    run_advance(sim)
    assert sim.clock_pub.publish.call_count == 5
    assert sim.terrain_scan_pub.publish.call_count == 2
    sim.run()
    assert sim.clock_pub.publish.call_count == 5
    assert sim.terrain_scan_pub.publish.call_count == 2


def test_keys_between_phases_wait_for_complete_cycle(make_sim):
    sim, bridge, viewer, keys, clock = make_sim()
    sim.set_paused()
    sim.pre_step()
    sim.advance()
    sim.step()
    sim.post_step()
    assert sim.data.time == 0
    bridge.receive.assert_not_called()
    bridge.send_state.assert_not_called()
    run_advance(sim)
    assert sim.data.time == pytest.approx(0.1)
    assert bridge.receive.call_count == 50

    sim.set_paused(False)
    sim.pre_step()
    sim.set_paused()
    sim.step()
    sim.set_paused(False)
    count = sim.iter_counter
    sim.post_step()
    assert sim.iter_counter == count  # No post-step updates for skipped physics.
    sim.run()
    assert sim.iter_counter == count + 1


@pytest.mark.parametrize('key', [ord('+'), ord('='), 334])
def test_rtf_increase_keys(make_sim, key):
    sim, bridge, viewer, keys, clock = make_sim(viewer=True)
    keys[0](key)
    assert sim.target_rtf == pytest.approx(1.1)
    assert sim._timing_reset_requested
    sim.run()
    assert clock.sleeps[-1] == pytest.approx(sim.model.opt.timestep / 1.1)
    assert not sim._timing_reset_requested


@pytest.mark.parametrize('key', [ord('-'), 333])
def test_rtf_decrease_keys_and_lower_limit(make_sim, key):
    sim, bridge, viewer, keys, clock = make_sim(viewer=True)
    keys[0](key)
    assert sim.target_rtf == pytest.approx(0.9)
    sim.set_paused()
    for _ in range(20):
        keys[0](key)
    assert sim.target_rtf == pytest.approx(0.1)
    assert sim.paused
    assert not sim._advance_requests
    bridge.receive.assert_not_called()
    sim.advance(0.002)
    sim.run()
    assert clock.sleeps[-1] == pytest.approx(sim.model.opt.timestep / 0.1)


@pytest.mark.parametrize('target', [0, -1, float('nan'), float('inf')])
def test_target_rtf_rejects_invalid_values(make_sim, target):
    sim, *_ = make_sim()
    with pytest.raises(ValueError, match='finite and positive'):
        sim.set_target_rtf(target)
    assert sim.target_rtf == 1.0


def test_rtf_keys_initialized_before_viewer_launch(make_sim):
    sim, *_ = make_sim(viewer=True, on_launch=lambda callback: callback(ord('+')))
    assert sim.target_rtf == pytest.approx(1.1)
