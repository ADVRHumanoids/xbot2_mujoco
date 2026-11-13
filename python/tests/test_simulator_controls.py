"""Python and mjctl share one simulator-owned control state."""
from types import SimpleNamespace
from unittest.mock import Mock

import mujoco
import numpy as np
import pytest
import zmq

from xbot2_py_mujoco.simulator_wrapper import SimulatorWrapper
from xbot2_py_mujoco.remote_control import CommandError


@pytest.fixture
def sim():
    model = mujoco.MjModel.from_xml_string('''<mujoco><option timestep=".001" gravity="0 0 0"/>
      <worldbody><body name="robot" pos="0 0 1"><freejoint/>
        <geom name="shell" size=".1" mass="1" friction=".5 .01 .001"/>
        <body name="left_leg"><joint name="left_knee"/>
          <geom name="left_geom" size=".03" mass=".1" contype="0" conaffinity="0"/></body>
        <body name="right_leg"><joint name="right_knee"/>
          <geom name="right_geom" size=".03" mass=".1" contype="0" conaffinity="0"/></body>
      </body></worldbody><actuator><position name="left_knee" joint="left_knee" kp="1"/>
        <position name="right_knee" joint="right_knee" kp="1"/></actuator></mujoco>''')
    value = SimulatorWrapper(model, viewer=False, ros=False, enable_xbot2_bridge=False, enable_pacing=False)
    try:
        yield value
    finally:
        value.close()


def test_model_operations_without_socket_restore_state(sim):
    assert sim.remote_control is None
    original = {name: getattr(sim.model, name).copy() for name in sim.control._RESTORABLE_MODEL_FIELDS}
    sim.data.time = 1.2
    sim.data.qvel[:] = .2
    pose, velocity = sim.data.qpos.copy(), sim.data.qvel.copy()
    sim.set_friction('robot', np.array([.9, .02, .003]))
    sim.set_contact_parameters('.*_leg', regex=True, solref=(.01, 1), condim=4)
    sim.set_body_properties('robot', mass=2, com=(.01, 0, 0))
    sim.add_payload('robot', mass=.3, position=(.1, 0, 0))
    assert sim.model.body_mass[sim.model.body('robot').id] == pytest.approx(1.3)
    sim.set_joint_torque_limits('.*_knee', 5, regex=True)
    np.testing.assert_array_equal(sim.model.jnt_actfrcrange[1:], [[-5, 5], [-5, 5]])
    sim.data.qfrc_applied[:] = 2
    sim.apply_wrench('robot', force=[1, 0, 0])
    result = sim.restore()
    assert result['restored']
    for name, expected in original.items():
        np.testing.assert_array_equal(getattr(sim.model, name), expected)
    np.testing.assert_array_equal(sim.data.qpos, pose)
    np.testing.assert_array_equal(sim.data.qvel, velocity)
    np.testing.assert_array_equal(sim.data.xfrc_applied, 0)
    np.testing.assert_array_equal(sim.data.qfrc_applied, 0)
    assert sim.data.time == 1.2


def test_persistent_and_timed_wrenches_sum_and_expire(sim):
    body = sim.model.body('robot').id
    sim.apply_wrench('robot', [1, 0, 0])
    sim.apply_wrench('robot', [2, 0, 0], duration=.002)
    sim.apply_wrench('robot', [3, 0, 0], duration=.001)
    sim.run()
    assert sim.data.xfrc_applied[body, 0] == 6
    sim.run()
    assert sim.data.xfrc_applied[body, 0] == 3
    sim.run()
    assert sim.data.xfrc_applied[body, 0] == 1
    sim.apply_wrench('robot', [4, 0, 0])
    assert sim.data.xfrc_applied[body, 0] == 4
    sim.clear_wrenches('robot')
    sim.run()
    np.testing.assert_array_equal(sim.data.xfrc_applied[body], 0)


def test_local_frames_regex_and_invalid_updates_are_atomic(sim):
    body = sim.model.body('robot').id
    sim.data.qpos[3:7] = [2**-.5, 0, 0, 2**-.5]
    sim.apply_wrench('robot', [1, 0, 0], [0, 0, 2], frame='body')
    np.testing.assert_allclose(sim.data.xfrc_applied[body], [0, 1, 0, 0, 0, 2], atol=1e-12)
    sim.apply_wrench('.*_leg', [0, 0, 1], regex=True, frame='world')
    for name in ('left_leg', 'right_leg'):
        assert sim.data.xfrc_applied[sim.model.body(name).id, 2] == 1
    before = sim.data.xfrc_applied.copy()
    for kwargs in ({'body_name': 'missing'}, {'frame': 'invalid'}, {'duration': -1},
                   {'force': [np.nan, 0, 0]}, {'body_name': 'world'}):
        with pytest.raises(CommandError):
            sim.apply_wrench(**({'body_name': 'robot'} | kwargs))
        np.testing.assert_array_equal(sim.data.xfrc_applied, before)


def test_tuning_uses_bridge_capabilities_and_restore(sim):
    bridge = SimpleNamespace(initial_actuator_delay_s=.01,
                             set_actuator_delay=Mock(), set_velocity_feedback_noise=Mock(),
                             reset_velocity_feedback_noise=Mock())
    sim.control.actuator_bridge = bridge
    sim.set_actuator_delay(.02)
    sim.set_velocity_feedback_noise('.*_knee', .1, regex=True)
    bridge.set_actuator_delay.assert_called_once_with(.02)
    bridge.set_velocity_feedback_noise.assert_called_once_with(['left_knee', 'right_knee'], .1)
    sim.restore()
    bridge.set_actuator_delay.assert_called_with(.01)
    bridge.reset_velocity_feedback_noise.assert_called_once()
    sim.control.actuator_bridge = None
    with pytest.raises(CommandError, match='unavailable'):
        sim.set_actuator_delay(0)


def test_mjctl_transport_and_python_share_properties_and_wrenches(sim):
    from xbot2_py_mujoco.remote_control import RemoteControlServer

    server = RemoteControlServer(sim.model, sim.data, 'inproc://shared-python-controls', control=sim.control)
    client = zmq.Context.instance().socket(zmq.REQ)
    client.setsockopt(zmq.LINGER, 0)
    client.setsockopt(zmq.RCVTIMEO, 1000)
    client.connect(server.endpoint)
    try:
        assert server.control is sim.control
        sim.apply_wrench('robot', [1, 0, 0])
        client.send_json({'command': 'apply_wrench', 'body': 'robot', 'force': [2, 0, 0],
                          'torque': [0, 0, 0], 'duration': .002})
        assert server.process_requests() == 1
        assert client.recv_json()['ok']
        sim.control.apply_wrenches()
        assert sim.data.xfrc_applied[sim.model.body('robot').id, 0] == 3
        sim.set_friction('robot', [.7, .01, .001])
        client.send_json({'command': 'restore'})
        server.process_requests()
        assert client.recv_json()['ok']
        np.testing.assert_array_equal(sim.data.xfrc_applied, 0)
        assert sim.model.geom_friction[sim.model.geom('shell').id, 0] == .5
    finally:
        client.close()
        server.close()
