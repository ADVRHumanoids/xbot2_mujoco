from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco.simulate import parse_args
from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper
from xbot2_py_mujoco import web_viewer as web


def test_web_cli_is_opt_in_and_excludes_headless():
    assert not parse_args([]).web_viewer
    args = parse_args(['--web-viewer', '--web-port', '8088', '--web-open-browser'])
    assert args.web_viewer and args.web_port == 8088 and args.web_open_browser
    for options in (['--web-viewer', '--headless'], ['--web-port', '-1'],
                    ['--web-port', '65536']):
        with pytest.raises(SystemExit):
            parse_args(options)


def test_web_sync_does_not_step_and_copies_overlays(monkeypatch):
    model = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size="1"/></worldbody></mujoco>')
    data = mujoco.MjData(model)
    simulator = SimpleNamespace(paused=True, target_rtf=.7)
    handle = Mock()
    monkeypatch.setattr(web.launch_web, 'launch', Mock(return_value=handle))
    viewer = web.WebViewerHandle(model, data, Mock(), simulator, port=8088)
    try:
        viewer.user_scn.ngeom = 1
        viewer.user_scn.geoms[0].pos[:] = (1, 2, 3)
        viewer.sync()
        assert data.time == 0
        handle.sync.assert_called_once_with()
        overlay, controls = [call.args[0] for call in handle.send_to_viewer.call_args_list]
        viewer.user_scn.geoms[0].pos[:] = 0
        np.testing.assert_array_equal(overlay.geoms[0].pos, [1, 2, 3])
        assert controls.pause_state == web.sim.PauseState.NORMAL_PAUSED
        assert controls.speed == 70
    finally:
        viewer.close()
    handle.close.assert_called_once()


def test_studio_controls_update_wrapper_without_stepping():
    simulator = Mock()
    controls = web._Controls(simulator)
    assert controls._on_controls(web.messages.StepControlSnapshot(
        pause_state=web.sim.PauseState.NORMAL_PAUSED, speed=50,
    ))
    simulator.set_paused.assert_called_once_with(True)
    simulator.set_target_rtf.assert_called_once_with(.5)
    assert controls._on_advance(web.messages.SingleStepEvent())
    simulator.advance.assert_called_once_with(.01666)
    assert controls._on_load(web.messages.LoadModelEvent(path='other.xml'))
    simulator.run.assert_not_called()


def test_web_launch_failure_closes_handle(monkeypatch):
    model = mujoco.MjModel.from_xml_string('<mujoco/>')
    handle = Mock()
    handle.set_model.side_effect = RuntimeError('failed')
    monkeypatch.setattr(web.launch_web, 'launch', Mock(return_value=handle))
    with pytest.raises(RuntimeError, match='failed'):
        web.WebViewerHandle(model, mujoco.MjData(model), Mock(), Mock())
    handle.close.assert_called_once()


@pytest.mark.parametrize('kwargs', [
    {'viewer_backend': 'invalid'}, {'web_port': -1}, {'web_port': True},
])
def test_invalid_viewer_settings_rejected_before_launch(kwargs):
    model = mujoco.MjModel.from_xml_string('<mujoco/>')
    with pytest.raises(ValueError):
        SimulatorWrapper(model, viewer=False, ros=False, **kwargs)


@pytest.mark.parametrize('key,expected', [
    ('Space', 32), ('RightArrow', 262), ('R', ord('R')),
    ('Equal', ord('=')), ('Minus', ord('-')),
    ('KeypadAdd', 334), ('KeypadSubtract', 333),
])
def test_web_keyboard_forwards_wrapper_controls(monkeypatch, key, expected):
    callback = Mock()
    application = web._App.__new__(web._App)
    application._key_callback = callback
    monkeypatch.setattr(web.imgui, 'GetIO', lambda: SimpleNamespace(WantCaptureKeyboard=False))
    monkeypatch.setattr(web.imgui, 'IsKeyPressed',
                        lambda candidate, repeat: candidate == getattr(web.imgui.Key, key))
    fallback = Mock()
    monkeypatch.setattr(web.app.App, 'handle_keyboard_events', fallback)

    application.handle_keyboard_events()

    callback.assert_called_once_with(expected)
    fallback.assert_not_called()


def test_web_keyboard_respects_text_input_capture(monkeypatch):
    callback = Mock()
    application = web._App.__new__(web._App)
    application._key_callback = callback
    monkeypatch.setattr(web.imgui, 'GetIO', lambda: SimpleNamespace(WantCaptureKeyboard=True))
    pressed = Mock(return_value=True)
    monkeypatch.setattr(web.imgui, 'IsKeyPressed', pressed)
    fallback = Mock()
    monkeypatch.setattr(web.app.App, 'handle_keyboard_events', fallback)

    application.handle_keyboard_events()

    callback.assert_not_called()
    pressed.assert_not_called()
    fallback.assert_called_once_with()
