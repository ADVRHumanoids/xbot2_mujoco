from contextlib import nullcontext
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco.base_velocity_plot import BaseVelocityPlot
from xbot2_py_mujoco import simulator_wrapper as wrapper


def scene():
    model = mujoco.MjModel.from_xml_string('''<mujoco><worldbody>
      <body name="pelvis" pos="0 0 1" quat=".70710678 0 0 .70710678">
        <freejoint/>
        <inertial pos=".3 -.2 .1" quat=".70710678 .70710678 0 0"
                  mass="1" diaginertia=".1 .2 .3"/>
        <geom type="box" size=".1 .1 .1"/>
      </body>
    </worldbody></mujoco>''')
    return model, mujoco.MjData(model)


def test_velocity_at_link_origin_in_base_coordinates():
    model, data = scene()
    plot = BaseVelocityPlot(model, 'pelvis')
    data.qvel[:] = [1, 2, 3, .4, .5, .6]
    # Deliberately leave derived kinematics uninitialized.
    plot.sample(data)
    velocity = plot.samples[-1][1]
    np.testing.assert_allclose(velocity[:3], [.4, .5, .6], atol=1e-7)
    np.testing.assert_allclose(velocity[3:], [2, -1, 3], atol=1e-7)
    overlays = plot.overlays(mujoco.MjrRect(50, 20, 900, 600))
    for i, (rect, figure) in enumerate(overlays):
        assert rect.left + rect.width == 950
        assert rect.bottom >= 20
        assert figure.linepnt[:3].tolist() == [1, 1, 1]
        np.testing.assert_allclose(figure.linedata[:3, 1], velocity[3:] if i == 0 else velocity[:3])
    assert '(m/s)' in plot.figures[0].title
    assert '(rad/s)' in plot.figures[1].title
    assert overlays[0][0].bottom >= overlays[1][0].bottom + overlays[1][0].height
    smaller = plot.overlays(mujoco.MjrRect(0, 0, 300, 150))
    assert smaller[0][0].width == 150 and smaller[0][0].height == 75
    assert plot.overlays(None) == []


def test_history_limits_duplicate_times_and_time_reversal():
    model, data = scene()
    plot = BaseVelocityPlot(model, 'pelvis')
    for i in range(1100):
        data.time = i * .001
        plot.sample(data)
    assert len(plot.samples) == 1000
    plot.sample(data)
    assert len(plot.samples) == 1000
    data.time = 20
    plot.sample(data)
    assert len(plot.samples) == 1
    data.time = 1
    plot.sample(data)
    assert len(plot.samples) == 1 and plot.samples[0][0] == 1
    plot.clear()
    assert not plot.samples and not plot.figures[0].linepnt.any()


def simulator(monkeypatch, viewer=True, body='pelvis'):
    fake_viewer = SimpleNamespace(
        viewport=mujoco.MjrRect(0, 0, 800, 600), user_scn=mujoco.MjvScene(),
        lock=nullcontext, sync=lambda **kwargs: None, is_running=lambda: True,
        set_figures=lambda figures: calls.append(figures), close=lambda: None)
    calls = []
    monkeypatch.setattr(wrapper, 'launch_passive', lambda *a, **kw: fake_viewer)
    monkeypatch.setattr(wrapper, 'MjXbot2Bridge', lambda **kw:
                        SimpleNamespace(receive=lambda: None, send_state=lambda: None))
    model, _ = scene()
    sim = wrapper.SimulatorWrapper(model, viewer=viewer, ros=False,
                                   base_velocity_plot_body=body, target_rtf=1e9)
    sim.time_start -= 1
    return sim, calls


def test_refresh_reset_and_respawn(monkeypatch):
    sim, calls = simulator(monkeypatch)
    sim.data.time = 2
    sim.post_step()
    assert len(calls) == 1 and len(calls[0]) == 2
    plot = sim.base_velocity_plot
    sim.data.time = 0
    sim.pre_step()
    assert not plot.samples
    assert sim.data.time == 2  # Existing reset retains simulation time.
    plot.sample(sim.data)
    sim._respawn()
    assert not plot.samples


def test_paused_plot_refresh_preserves_history(monkeypatch):
    sim, calls = simulator(monkeypatch)
    sim.data.time = 2
    sim.post_step()
    sim.set_paused()
    sim.pre_step()
    sim.step()
    sim.viewer.viewport = mujoco.MjrRect(0, 0, 400, 300)
    sim._next_viewer_sync_wall = 0
    sim.post_step()
    assert len(calls) == 2
    assert calls[-1][0][0].width == 200
    assert len(sim.base_velocity_plot.samples) == 1
    assert sim.data.time == 2


@pytest.mark.parametrize('viewer,body', [(True, None), (True, ''), (False, 'missing')])
def test_disabled_or_headless_skips_plot(monkeypatch, viewer, body):
    sim, calls = simulator(monkeypatch, viewer, body)
    assert sim.base_velocity_plot is None
    sim.data.time = 2
    sim.post_step()
    assert not calls


def test_invalid_body_and_unsupported_api(monkeypatch):
    with pytest.raises(ValueError, match='missing'):
        simulator(monkeypatch, body='missing')
    monkeypatch.setattr(wrapper, 'Handle', SimpleNamespace())
    with pytest.raises(RuntimeError, match='set_figures'):
        simulator(monkeypatch)


def test_figures_coexist_with_force_and_terrain_visuals(monkeypatch):
    sim, calls = simulator(monkeypatch)
    events = []
    sim.remote_control = SimpleNamespace(update_visuals=lambda scene: events.append('force'))
    sim.terrain_scanner = SimpleNamespace(scan=lambda data: sim.terrain_scan)
    sim.terrain_scan = SimpleNamespace(add_visuals=lambda scene: events.append('terrain'))
    sim.data.time = 2
    sim.post_step()
    assert events == ['force', 'terrain']
    assert len(calls) == 1 and len(calls[0]) == 2
