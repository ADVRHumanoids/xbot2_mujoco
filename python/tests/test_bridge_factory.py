"""Custom bridge lifecycle and the transport-free simulator path."""
from types import SimpleNamespace
from unittest.mock import Mock
import subprocess
import sys

import mujoco
import numpy as np
import pytest

from xbot2_py_mujoco import simulator_wrapper as wrapper


def model():
    return mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".001"/>
      <worldbody><body name="robot" pos="0 0 1"><freejoint/>
        <geom type="sphere" size=".1" mass="1"/>
        <body><joint name="hinge"/><geom size=".05" mass="1"/></body>
      </body></worldbody><actuator><position name="hinge" joint="hinge" kp="1"/></actuator>
    </mujoco>''')


@pytest.mark.parametrize('enable_default', [True, False])
def test_custom_factory_shares_data_and_overrides_default(monkeypatch, enable_default):
    monkeypatch.setattr(wrapper, 'MjXbot2Bridge', Mock(side_effect=AssertionError('default bridge selected')))
    monkeypatch.setattr(wrapper.time, 'sleep', Mock(side_effect=AssertionError('pacing enabled')))
    bridge = SimpleNamespace(receive=Mock(), send_state=Mock(), reset=Mock(), close=Mock())
    factory = Mock(return_value=bridge)
    sim = wrapper.SimulatorWrapper(model(), ros=False, viewer=False, bridge_factory=factory,
                                   enable_xbot2_bridge=enable_default, enable_pacing=False,
                                   send_state_decimation=2, q_init={'hinge': 0.3})
    try:
        factory.assert_called_once_with(sim.model, sim.data)
        bridge.reset.assert_called_once()
        assert sim.data.joint('hinge').qpos[0] == pytest.approx(.3)
        for _ in range(3):
            sim.run()
        assert bridge.receive.call_count == 3
        assert bridge.send_state.call_count == 1
        assert bridge.reset.call_count == 1  # No reset on the first receive.
        assert sim.physics_steps == 3
        assert sim.elapsed_time == pytest.approx(.003)
        assert sim.data.time == pytest.approx(.003)
        # An explicit MuJoCo reset notifies the controller without restoring old time.
        mujoco.mj_resetData(sim.model, sim.data)
        sim.run()
        assert bridge.reset.call_count == 2
        assert sim.data.time == pytest.approx(.001)
        assert sim.elapsed_time == pytest.approx(.004)
        sim.spawn_locations = [(1, 2, 3)]
        sim._respawn()
        assert bridge.reset.call_count == 3
        np.testing.assert_allclose(sim.data.qpos[:3], [1, 2, 3])
    finally:
        sim.close()
    bridge.close.assert_called_once()


def test_disabled_bridge_has_no_transport_imports():
    result = subprocess.run([sys.executable, '-c', '''
import sys
import mujoco
from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper
sim = SimulatorWrapper(mujoco.MjModel.from_xml_string('<mujoco/>'), viewer=False,
                       ros=False, enable_xbot2_bridge=False, enable_pacing=False)
sim.run()
sim.close()
assert not any(name.startswith(('xbot2_py_bridge', 'pyxbot2_diagnostics', 'rclpy')) for name in sys.modules)
'''], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr


@pytest.mark.parametrize('supports_tuning', [False, True])
def test_remote_actuator_tuning_is_optional(monkeypatch, supports_tuning):
    bridge = SimpleNamespace(receive=Mock(), send_state=Mock(), close=Mock())
    if supports_tuning:
        bridge.initial_actuator_delay_s = 0
        bridge.set_actuator_delay = Mock()
        bridge.set_velocity_feedback_noise = Mock()
    server = Mock(return_value=SimpleNamespace(close=Mock()))
    monkeypatch.setattr(wrapper, 'RemoteControlServer', server)
    sim = wrapper.SimulatorWrapper(model(), viewer=False, ros=False,
                                   bridge_factory=lambda model, data: bridge,
                                   remote_control_endpoint='inproc://test-bridge')
    try:
        assert server.call_args.kwargs['control'] is sim.control
        assert sim.control.actuator_bridge is bridge
        if supports_tuning:
            sim.set_actuator_delay(0.02)
            bridge.set_actuator_delay.assert_called_once_with(0.02)
        else:
            with pytest.raises(ValueError, match='unavailable'):
                sim.set_actuator_delay(0.02)
    finally:
        sim.close()
